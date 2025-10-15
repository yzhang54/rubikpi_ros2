"""
Waypoint follower for differential drive robot with pose targets.
HW1: Follow waypoints with (x, y, theta) targets.

Based on lecture: "Moving to a Pose" strategy
- rho: distance to goal
- alpha: angle to goal position
- beta: heading error at goal
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
import numpy as np
from scipy.spatial.transform import Rotation
import time

from transforms import SE2Transform
from differential_drive import DifferentialDrive
from pid_controller import PIDController

class WaypointFollower(Node):
    """
    Waypoint following with full pose control (x, y, theta).
    
    Strategy from lecture (Page 14 - "Moving to a Pose"):
    - rho (ρ): distance to goal position
    - alpha (α): angle to approach goal
    - beta (β): final heading error
    
    Control law:
    v = Kρ * ρ
    ω = Kα * α + Kβ * β
    """
    
    def __init__(self):
        super().__init__('waypoint_follower')
        
        # Declare parameters
        self.declare_parameter('waypoint_file', '/home/ubuntu/ros2_ws/waypoints.txt')
        self.declare_parameter('wheel_radius', 0.033)  # meters
        self.declare_parameter('wheelbase', 0.15)      # meters
        self.declare_parameter('max_linear_speed', 0.3)  # m/s
        self.declare_parameter('max_angular_speed', 2.0)  # rad/s
        self.declare_parameter('position_tolerance', 0.1)  # meters
        self.declare_parameter('angle_tolerance', 0.1)  # radians (~5.7 degrees)
        
        # Control gains (tune these!)
        self.declare_parameter('Kp_rho', 0.5)      # Distance gain
        self.declare_parameter('Kp_alpha', 1.0)    # Approach angle gain
        self.declare_parameter('Kp_beta', -0.5)    # Heading gain
        
        # Get parameters
        waypoint_file = self.get_parameter('waypoint_file').value
        wheel_radius = self.get_parameter('wheel_radius').value
        wheelbase = self.get_parameter('wheelbase').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.angle_tolerance = self.get_parameter('angle_tolerance').value
        
        # Control gains
        self.Kp_rho = self.get_parameter('Kp_rho').value
        self.Kp_alpha = self.get_parameter('Kp_alpha').value
        self.Kp_beta = self.get_parameter('Kp_beta').value
        
        # Initialize differential drive kinematics
        self.drive = DifferentialDrive(wheel_radius, wheelbase)
        
        # Robot state
        self.pose = SE2Transform(0.0, 0.0, 0.0)
        self.velocity = (0.0, 0.0)  # (v, omega)
        
        # Load waypoints (x, y, theta)
        self.waypoints = self.load_waypoints(waypoint_file)
        self.current_waypoint_idx = 0
        self.state = 'APPROACH'  # APPROACH or ALIGN
        
        # ROS2 interfaces
        self.motor_pub = self.create_publisher(
            Float32MultiArray,
            '/motor_commands',
            10
        )
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu_data',
            self.imu_callback,
            10
        )
        
        # Control loop
        self.control_dt = 0.1  # 10 Hz
        self.timer = self.create_timer(self.control_dt, self.control_loop)
        self.last_time = self.get_clock().now()
        
        # Logging
        self.mission_complete = False
        self.start_time = time.time()
        self.log_counter = 0
        
        # Statistics for report
        self.stats = {
            'waypoints_reached': 0,
            'total_distance': 0.0,
            'max_error': 0.0,
        }
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('HW1: WAYPOINT FOLLOWER INITIALIZED')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Waypoints loaded: {len(self.waypoints)}')
        self.get_logger().info(f'Control gains: Kρ={self.Kp_rho}, Kα={self.Kp_alpha}, Kβ={self.Kp_beta}')
        self.get_logger().info(f'Tolerances: position={self.position_tolerance}m, angle={np.degrees(self.angle_tolerance):.1f}°')
        self.get_logger().info('=' * 60)
    
    def load_waypoints(self, filename):
        """
        Load waypoints from file.
        Format: x, y, angle (meters, meters, radians)
        """
        waypoints = []
        try:
            with open(filename, 'r') as f:
                for i, line in enumerate(f):
                    line = line.strip()
                    if line and not line.startswith('#'):
                        parts = line.split(',')
                        if len(parts) == 3:
                            x, y, theta = map(float, parts)
                            waypoints.append((x, y, theta))
                        else:
                            self.get_logger().warn(f'Invalid waypoint format on line {i+1}: {line}')
            
            self.get_logger().info(f'Successfully loaded {len(waypoints)} waypoints')
            for i, wp in enumerate(waypoints):
                self.get_logger().info(f'  WP{i}: x={wp[0]:.2f}, y={wp[1]:.2f}, θ={np.degrees(wp[2]):.1f}°')
        
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {e}')
            # Default test waypoints (square with angles)
            waypoints = [
                (1.0, 0.0, 0.0),
                (1.0, 1.0, np.pi/2),
                (0.0, 1.0, np.pi),
                (0.0, 0.0, -np.pi/2)
            ]
            self.get_logger().warn('Using default test waypoints (1m square)')
        
        return waypoints
    
    def imu_callback(self, msg):
        """Update robot orientation from IMU"""
        quat = msg.orientation
        r = Rotation.from_quat([quat.x, quat.y, quat.z, quat.w])
        euler = r.as_euler('xyz')
        self.pose.theta = euler[2]  # Yaw
    
    def control_loop(self):
        """
        Main control loop implementing "Moving to a Pose" strategy.
        
        Two-stage approach:
        1. APPROACH: Move to target position (x, y)
        2. ALIGN: Rotate to target orientation (theta)
        """
        if self.mission_complete:
            return
        
        # Check if mission complete
        if self.current_waypoint_idx >= len(self.waypoints):
            self.mission_complete = True
            self.send_motor_commands(0.0, 0.0)
            elapsed = time.time() - self.start_time
            self.print_mission_summary(elapsed)
            return
        
        # Get current goal
        goal_x, goal_y, goal_theta = self.waypoints[self.current_waypoint_idx]
        
        # Calculate errors
        dx = goal_x - self.pose.x
        dy = goal_y - self.pose.y
        rho = np.sqrt(dx**2 + dy**2)  # Distance to goal
        
        # Time step
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        if dt <= 0 or dt > 1.0:
            dt = self.control_dt
        
        # State machine: APPROACH or ALIGN
        if self.state == 'APPROACH':
            if rho < self.position_tolerance:
                # Position reached, switch to alignment
                self.state = 'ALIGN'
                self.get_logger().info(
                    f'✓ Position reached for WP{self.current_waypoint_idx}, '
                    f'now aligning to θ={np.degrees(goal_theta):.1f}°'
                )
                self.send_motor_commands(0.0, 0.0)
                return
            
            # Approach control (from lecture)
            v, omega = self.approach_control(goal_x, goal_y, goal_theta, dt)
        
        else:  # ALIGN
            # Check if aligned
            angle_error = SE2Transform.normalize_angle(goal_theta - self.pose.theta)
            if abs(angle_error) < self.angle_tolerance:
                # Waypoint complete!
                self.state = 'APPROACH'
                self.stats['waypoints_reached'] += 1
                self.get_logger().info(
                    f'✅ WAYPOINT {self.current_waypoint_idx} COMPLETE '
                    f'at ({self.pose.x:.2f}, {self.pose.y:.2f}, {np.degrees(self.pose.theta):.1f}°)'
                )
                self.current_waypoint_idx += 1
                self.send_motor_commands(0.0, 0.0)
                time.sleep(0.5)  # Brief pause
                return
            
            # Alignment control (pure rotation)
            v, omega = self.align_control(goal_theta)
        
        # Update odometry
        self.pose = self.drive.update_odometry(self.pose, v, omega, dt)
        self.stats['total_distance'] += abs(v) * dt
        self.stats['max_error'] = max(self.stats['max_error'], rho)
        
        # Send commands
        self.send_motor_commands(v, omega)
        
        # Periodic logging
        self.log_counter += 1
        if self.log_counter % 20 == 0:  # Every 2 seconds
            self.get_logger().info(
                f'[{self.state}] WP{self.current_waypoint_idx}: '
                f'pos=({self.pose.x:.2f},{self.pose.y:.2f}) '
                f'θ={np.degrees(self.pose.theta):.1f}° '
                f'ρ={rho:.3f}m v={v:.2f} ω={omega:.2f}'
            )
    
    def approach_control(self, goal_x, goal_y, goal_theta, dt):
        """
        Approach control using "Moving to a Pose" strategy.
        
        From lecture (Page 14):
        - ρ (rho): distance to goal
        - α (alpha): angle to goal relative to robot heading
        - β (beta): goal orientation relative to approach angle
        
        Control law:
        v = Kρ * ρ
        ω = Kα * α + Kβ * β
        """
        # Calculate errors in world frame
        dx = goal_x - self.pose.x
        dy = goal_y - self.pose.y
        
        # Transform to robot frame
        rho = np.sqrt(dx**2 + dy**2)
        
        # Angle to goal (in world frame)
        theta_goal = np.arctan2(dy, dx)
        
        # Alpha: difference between robot heading and direction to goal
        alpha = SE2Transform.normalize_angle(theta_goal - self.pose.theta)
        
        # Beta: final heading error
        beta = SE2Transform.normalize_angle(goal_theta - theta_goal)
        
        # Control law from lecture
        v = self.Kp_rho * rho
        omega = self.Kp_alpha * alpha + self.Kp_beta * beta
        
        # Slow down when approaching
        if rho < 0.3:
            v *= 0.5
        
        # Reduce speed for sharp turns
        if abs(alpha) > np.pi/4:  # > 45 degrees
            v *= 0.5
        
        # Clamp velocities
        v = np.clip(v, 0, self.max_linear_speed)
        omega = np.clip(omega, -self.max_angular_speed, self.max_angular_speed)
        
        return v, omega
    
    def align_control(self, goal_theta):
        """
        Pure rotational control to align with target heading.
        """
        angle_error = SE2Transform.normalize_angle(goal_theta - self.pose.theta)
        
        # Pure rotation (no forward motion)
        v = 0.0
        omega = self.Kp_alpha * angle_error * 2  # Higher gain for alignment
        
        # Clamp
        omega = np.clip(omega, -self.max_angular_speed, self.max_angular_speed)
        
        return v, omega
    
    def send_motor_commands(self, v, omega):
        """Send motor commands using differential drive inverse kinematics"""
        L_cmd, R_cmd = self.drive.velocity_to_motor_commands(v, omega)
        
        msg = Float32MultiArray()
        msg.data = [float(L_cmd), float(R_cmd)]
        self.motor_pub.publish(msg)
        
        self.velocity = (v, omega)
    
    def print_mission_summary(self, elapsed_time):
        """Print summary statistics for report"""
        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info('MISSION COMPLETE - SUMMARY')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Waypoints reached: {self.stats["waypoints_reached"]}/{len(self.waypoints)}')
        self.get_logger().info(f'Total time: {elapsed_time:.1f} seconds')
        self.get_logger().info(f'Total distance: {self.stats["total_distance"]:.2f} meters')
        self.get_logger().info(f'Max position error: {self.stats["max_error"]:.3f} meters')
        self.get_logger().info(f'Final position: ({self.pose.x:.2f}, {self.pose.y:.2f}, {np.degrees(self.pose.theta):.1f}°)')
        self.get_logger().info('=' * 60)
    
    def shutdown(self):
        """Clean shutdown"""
        self.send_motor_commands(0.0, 0.0)
        self.get_logger().info('Waypoint follower shutdown')

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received')
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()