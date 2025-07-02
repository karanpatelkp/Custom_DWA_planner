# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String

# class ListenerNode(Node):
#     def __init__(self):
#         super().__init__('listener')
#         self.subscription = self.create_subscription(
#             String,
#             'chatter',
#             self.listener_callback,
#             10)

#     def listener_callback(self, msg):
#         self.get_logger().info(f'I heard: "{msg.data}"')

# def main(args=None):
#     rclpy.init(args=args)
#     node = ListenerNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

import numpy as np
import math
from typing import List, Tuple, Optional
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan

class DWAConfig:
    """Configuration parameters for DWA algorithm"""
    def __init__(self):
        # Robot configuration
        self.max_speed = 0.26  # [m/s] - TurtleBot3 max linear speed
        self.min_speed = 0.0   # [m/s]
        self.max_yaw_rate = 1.82  # [rad/s] - TurtleBot3 max angular speed
        self.max_accel = 2.5   # [m/ss]
        self.max_delta_yaw_rate = 3.2  # [rad/ss]
        
        # Velocity resolution
        self.v_resolution = 0.05  # [m/s]
        self.yaw_rate_resolution = 0.1  # [rad/s]
        
        # Prediction parameters
        self.dt = 0.1  # [s] - time step
        self.predict_time = 2.0  # [s] - prediction horizon
        
        # Cost function weights
        self.to_goal_cost_gain = 0.15
        self.speed_cost_gain = 1.0  
        self.obstacle_cost_gain = 1.0
        self.path_smooth_gain = 0.1
        
        # Robot radius for collision checking
        self.robot_radius = 0.105  # [m] - TurtleBot3 radius
        
        # Goal tolerance
        self.goal_tolerance = 0.2  # [m]

class DWAPlanner:
    """Dynamic Window Approach local planner implementation"""
    
    def __init__(self, config: DWAConfig):
        self.config = config
        self.goal = None
        self.obstacles = []
        
    def set_goal(self, goal_x: float, goal_y: float):
        """Set the goal position"""
        self.goal = (goal_x, goal_y)
        
    def update_obstacles(self, scan: LaserScan, robot_x: float, robot_y: float, robot_yaw: float):
        """Update obstacle positions from laser scan"""
        self.obstacles = []
        angle = scan.angle_min
        
        for i, range_val in enumerate(scan.ranges):
            if scan.range_min < range_val < scan.range_max:
                # Convert laser point to global coordinates
                local_x = range_val * math.cos(angle)
                local_y = range_val * math.sin(angle)
                
                # Transform to global frame
                global_x = robot_x + local_x * math.cos(robot_yaw) - local_y * math.sin(robot_yaw)
                global_y = robot_y + local_x * math.sin(robot_yaw) + local_y * math.cos(robot_yaw)
                
                self.obstacles.append((global_x, global_y))
            
            angle += scan.angle_increment
    
    def calc_dynamic_window(self, v: float, yaw_rate: float) -> Tuple[float, float, float, float]:
        """Calculate dynamic window based on current velocity and constraints"""
        # Dynamic window from robot specification
        Vs = [self.config.min_speed, self.config.max_speed,
              -self.config.max_yaw_rate, self.config.max_yaw_rate]
        
        # Dynamic window from motion model
        Vd = [v - self.config.max_accel * self.config.dt,
              v + self.config.max_accel * self.config.dt,
              yaw_rate - self.config.max_delta_yaw_rate * self.config.dt,
              yaw_rate + self.config.max_delta_yaw_rate * self.config.dt]
        
        # Final dynamic window
        dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
              max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]
        
        return dw
    
    def predict_trajectory(self, x: float, y: float, yaw: float, v: float, yaw_rate: float) -> List[Tuple[float, float, float]]:
        """Predict robot trajectory for given velocity commands"""
        trajectory = [(x, y, yaw)]
        
        time = 0
        while time <= self.config.predict_time:
            x += v * math.cos(yaw) * self.config.dt
            y += v * math.sin(yaw) * self.config.dt
            yaw += yaw_rate * self.config.dt
            trajectory.append((x, y, yaw))
            time += self.config.dt
            
        return trajectory
    
    def calc_obstacle_cost(self, trajectory: List[Tuple[float, float, float]]) -> float:
        """Calculate cost based on distance to obstacles"""
        min_r = float("inf")
        
        for (x, y, yaw) in trajectory:
            for (ox, oy) in self.obstacles:
                dx = x - ox
                dy = y - oy
                r = math.sqrt(dx * dx + dy * dy)
                
                if r <= self.config.robot_radius:
                    return float("inf")  # Collision
                
                if r < min_r:
                    min_r = r
        
        return 1.0 / min_r if min_r < float("inf") else 0.0
    
    def calc_to_goal_cost(self, trajectory: List[Tuple[float, float, float]]) -> float:
        """Calculate cost based on distance to goal"""
        if self.goal is None:
            return 0.0
            
        dx = trajectory[-1][0] - self.goal[0]
        dy = trajectory[-1][1] - self.goal[1]
        error_angle = math.atan2(dy, dx)
        cost_angle = error_angle - trajectory[-1][2]
        cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))
        
        return cost
    
    def calc_speed_cost(self, v: float) -> float:
        """Calculate cost based on speed (prefer higher speeds)"""
        return self.config.max_speed - v
    
    def calc_path_smooth_cost(self, trajectory: List[Tuple[float, float, float]]) -> float:
        """Calculate path smoothness cost"""
        if len(trajectory) < 3:
            return 0.0
            
        cost = 0.0
        for i in range(1, len(trajectory) - 1):
            dx1 = trajectory[i][0] - trajectory[i-1][0]
            dy1 = trajectory[i][1] - trajectory[i-1][1]
            dx2 = trajectory[i+1][0] - trajectory[i][0]
            dy2 = trajectory[i+1][1] - trajectory[i][1]
            
            angle1 = math.atan2(dy1, dx1)
            angle2 = math.atan2(dy2, dx2)
            angle_diff = abs(angle2 - angle1)
            
            if angle_diff > math.pi:
                angle_diff = 2 * math.pi - angle_diff
                
            cost += angle_diff
            
        return cost
    
    def calc_final_input(self, x: float, y: float, yaw: float, v: float, yaw_rate: float) -> Tuple[Optional[Twist], List[Tuple[float, float, float]]]:
        """Calculate optimal velocity command using DWA"""
        dw = self.calc_dynamic_window(v, yaw_rate)
        
        best_cmd = None
        min_cost = float("inf")
        best_trajectory = []
        
        # Sample velocities within dynamic window
        v_samples = np.arange(dw[0], dw[1], self.config.v_resolution)
        yaw_rate_samples = np.arange(dw[2], dw[3], self.config.yaw_rate_resolution)
        
        for v_sample in v_samples:
            for yaw_rate_sample in yaw_rate_samples:
                # Predict trajectory
                trajectory = self.predict_trajectory(x, y, yaw, v_sample, yaw_rate_sample)
                
                # Calculate costs
                to_goal_cost = self.calc_to_goal_cost(trajectory)
                speed_cost = self.calc_speed_cost(v_sample)
                obstacle_cost = self.calc_obstacle_cost(trajectory)
                smooth_cost = self.calc_path_smooth_cost(trajectory)
                
                # Skip if collision
                if obstacle_cost == float("inf"):
                    continue
                
                # Weighted final cost
                final_cost = (self.config.to_goal_cost_gain * to_goal_cost +
                             self.config.speed_cost_gain * speed_cost +
                             self.config.obstacle_cost_gain * obstacle_cost +
                             self.config.path_smooth_gain * smooth_cost)
                
                # Update best command
                if final_cost < min_cost:
                    min_cost = final_cost
                    best_trajectory = trajectory
                    
                    best_cmd = Twist()
                    best_cmd.linear.x = v_sample
                    best_cmd.angular.z = yaw_rate_sample
        
        return best_cmd, best_trajectory
    
    def is_goal_reached(self, x: float, y: float) -> bool:
        """Check if goal is reached"""
        if self.goal is None:
            return False
            
        dx = x - self.goal[0]
        dy = y - self.goal[1]
        distance = math.sqrt(dx * dx + dy * dy)
        
        return distance < self.config.goal_tolerance