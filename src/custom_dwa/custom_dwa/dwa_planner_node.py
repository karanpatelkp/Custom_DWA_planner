# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String

# class dwa_planner_nodeNode(Node):
#     def __init__(self):
#         super().__init__('dwa_planner_node')
#         self.publisher_ = self.create_publisher(String, 'chatter', 10)
#         timer_period = 1.0  # seconds
#         self.timer = self.create_timer(timer_period, self.timer_callback)
#         self.count = 0

#     def timer_callback(self):
#         msg = String()
#         msg.data = f'Hello ROS2: {self.count}'
#         self.publisher_.publish(msg)
#         self.get_logger().info(f'Publishing: "{msg.data}"')
#         self.count += 1

# def main(args=None):
#     rclpy.init(args=args)
#     node = dwa_planner_nodeNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import math
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import TransformException

from geometry_msgs.msg import Twist, PoseStamped, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header

from .dwa_algorithm import DWAPlanner, DWAConfig

class DWAPlannerNode(Node):
    """ROS2 Node for DWA Local Planner"""
    
    def __init__(self):
        super().__init__('dwa_planner_node')
        
        # Initialize DWA planner
        self.config = DWAConfig()
        self.planner = DWAPlanner(self.config)
        
        # Robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.robot_v = 0.0
        self.robot_yaw_rate = 0.0
        
        # Goal state
        self.goal_received = False
        
        # Control flag
        self.planning_active = False
        
        # Initialize TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.trajectory_pub = self.create_publisher(MarkerArray, '/dwa_trajectories', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)
        
        # Timer for control loop
        self.control_timer = self.create_timer(0.1, self.control_callback)
        
        self.get_logger().info('DWA Planner Node initialized')
        self.get_logger().info('Waiting for goal pose on /goal_pose topic...')
        self.get_logger().info('You can set a goal in RViz using "2D Nav Goal" tool')
    
    def odom_callback(self, msg: Odometry):
        """Update robot state from odometry"""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        # Convert quaternion to yaw
        orientation = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Get velocities
        self.robot_v = msg.twist.twist.linear.x
        self.robot_yaw_rate = msg.twist.twist.angular.z
    
    def scan_callback(self, msg: LaserScan):
        """Update obstacles from laser scan"""
        self.planner.update_obstacles(msg, self.robot_x, self.robot_y, self.robot_yaw)
    
    def goal_callback(self, msg: PoseStamped):
        """Set new goal from RViz or other source"""
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        
        self.planner.set_goal(goal_x, goal_y)
        self.goal_received = True
        self.planning_active = True
        
        self.get_logger().info(f'New goal received: ({goal_x:.2f}, {goal_y:.2f})')
        self.get_logger().info('Starting navigation...')
    
    def control_callback(self):
        """Main control loop"""
        if not self.goal_received or not self.planning_active:
            return
        
        # Check if goal is reached
        if self.planner.is_goal_reached(self.robot_x, self.robot_y):
            self.get_logger().info('Goal reached! Stopping robot.')
            self.stop_robot()
            self.planning_active = False
            return
        
        # Calculate optimal velocity command
        cmd_vel, trajectory = self.planner.calc_final_input(
            self.robot_x, self.robot_y, self.robot_yaw,
            self.robot_v, self.robot_yaw_rate
        )
        
        if cmd_vel is not None:
            # Publish velocity command
            self.cmd_vel_pub.publish(cmd_vel)
            
            # Visualize trajectory
            self.visualize_trajectory(trajectory)
            
            # Debug logging
            self.get_logger().debug(
                f'Command: v={cmd_vel.linear.x:.3f}, ω={cmd_vel.angular.z:.3f}'
            )
        else:
            self.get_logger().warn('No safe trajectory found! Stopping robot.')
            self.stop_robot()
    
    def stop_robot(self):
        """Stop the robot"""
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_cmd)
    
    def visualize_trajectory(self, trajectory):
        """Visualize predicted trajectory in RViz"""
        if not trajectory:
            return
            
        marker_array = MarkerArray()
        
        # Create line strip marker for trajectory
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "dwa_trajectory"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # Set marker properties
        marker.scale.x = 0.02  # Line width
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        # Add trajectory points
        for x, y, yaw in trajectory:
            point = Point()
            point.x = x
            point.y = y
            point.z = 0.01
            marker.points.append(point)
        
        marker_array.markers.append(marker)
        
        # Create marker for goal
        if self.planner.goal is not None:
            goal_marker = Marker()
            goal_marker.header.frame_id = "odom"
            goal_marker.header.stamp = self.get_clock().now().to_msg()
            goal_marker.ns = "dwa_goal"
            goal_marker.id = 1
            goal_marker.type = Marker.CYLINDER
            goal_marker.action = Marker.ADD
            
            goal_marker.pose.position.x = self.planner.goal[0]
            goal_marker.pose.position.y = self.planner.goal[1]
            goal_marker.pose.position.z = 0.1
            goal_marker.pose.orientation.w = 1.0
            
            goal_marker.scale.x = 0.3
            goal_marker.scale.y = 0.3
            goal_marker.scale.z = 0.2
            
            goal_marker.color.r = 1.0
            goal_marker.color.g = 0.0
            goal_marker.color.b = 0.0
            goal_marker.color.a = 0.8
            
            marker_array.markers.append(goal_marker)
        
        # Publish markers
        self.trajectory_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = DWAPlannerNode()
        
        # Use MultiThreadedExecutor for better performance
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            node.get_logger().info('Shutting down DWA Planner Node')
        finally:
            executor.shutdown()
            node.destroy_node()
            
    except Exception as e:
        print(f'Error in main: {e}')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()