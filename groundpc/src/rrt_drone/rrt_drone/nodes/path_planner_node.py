#!/usr/bin/env python3
"""
Path Planner Node - Ground PC
Distributed RRT path planning with obstacle avoidance
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
import numpy as np
from scipy.spatial import cKDTree
import time
import struct
import binascii

# -------------------------------------------------------------------
# CLASS: MessageVerifier
# -------------------------------------------------------------------
# Source: message_verifier_class.py
# -------------------------------------------------------------------
class MessageVerifier:
    """
    Message integrity verification using CRC32 checksum
    Detects corrupted messages over WiFi
    """
    
    @staticmethod
    def pack_with_crc(data_list):
        """
        Pack float list with CRC32 checksum
        
        Args:
            data_list: List of floats to pack
        
        Returns:
            (packed_data, crc): Tuple of packed bytes and CRC value
        """
        # Pack all floats as doubles (8 bytes each)
        packed = struct.pack(f'{len(data_list)}d', *data_list)
        
        # Calculate CRC32
        crc = binascii.crc32(packed) & 0xFFFFFFFF
        
        return packed, crc
    
    @staticmethod
    def verify_crc(data_list, received_crc):
        """
        Verify CRC of received message
        
        Args:
            data_list: Received float list (without CRC)
            received_crc: Received CRC value
        
        Returns:
            bool: True if CRC matches, False otherwise
        """
        # Pack data
        packed = struct.pack(f'{len(data_list)}d', *data_list)
        
        # Calculate CRC
        calculated_crc = binascii.crc32(packed) & 0xFFFFFFFF
        
        # Compare
        return calculated_crc == int(received_crc)
    
    @staticmethod
    def calculate_crc(data_list):
        """
        Calculate CRC for a list of floats
        
        Args:
            data_list: List of floats
        
        Returns:
            int: CRC32 value
        """
        packed = struct.pack(f'{len(data_list)}d', *data_list)
        return binascii.crc32(packed) & 0xFFFFFFFF

# -------------------------------------------------------------------
# CLASS: RRTPlanner
# -------------------------------------------------------------------
# Source: path_planner_complete.py
# -------------------------------------------------------------------
class RRTPlanner:
    """RRT path planning algorithm with KD-tree optimization"""
    
    def __init__(self, max_iter=2000, step_size=0.6, goal_tolerance=0.3):
        self.max_iter = max_iter
        self.step_size = step_size
        self.goal_tolerance = goal_tolerance
        
    def plan(self, start, goal, obstacles):
        """
        Plan path from start to goal avoiding obstacles
        
        Args:
            start: [x, y] starting position
            goal: [x, y] goal position
            obstacles: numpy array of obstacle positions [[x1,y1], [x2,y2], ...]
        
        Returns:
            path: list of waypoints [[x1,y1], [x2,y2], ...] or None if no path found
        """
        # Build obstacle KD-tree for fast collision checking
        if len(obstacles) > 0:
            obstacle_tree = cKDTree(obstacles)
        else:
            obstacle_tree = None
        
        # Initialize tree with start node
        nodes = [start]
        parents = [-1]  # Parent indices
        
        for i in range(self.max_iter):
            # Sample random point (with goal bias)
            if np.random.rand() < 0.1:  # 10% goal bias
                rand_point = goal
            else:
                # Sample in workspace bounds [-10, 10] x [-10, 10]
                rand_point = np.random.uniform(-10, 10, 2)
            
            # Find nearest node in tree
            dists = np.linalg.norm(np.array(nodes) - rand_point, axis=1)
            nearest_idx = np.argmin(dists)
            nearest = nodes[nearest_idx]
            
            # Steer from nearest towards random point
            direction = rand_point - nearest
            distance = np.linalg.norm(direction)
            
            if distance > self.step_size:
                new_point = nearest + (direction / distance) * self.step_size
            else:
                new_point = rand_point
            
            # Collision check
            if obstacle_tree is not None:
                dist_to_obstacle, _ = obstacle_tree.query(new_point)
                if dist_to_obstacle < 0.5:  # 0.5m safety margin
                    continue
                
                # Check path from nearest to new_point for collisions
                path_points = self._interpolate_path(nearest, new_point, 0.1)
                if self._path_in_collision(path_points, obstacle_tree, 0.5):
                    continue
            
            # Add to tree
            nodes.append(new_point)
            parents.append(nearest_idx)
            
            # Check if goal reached
            if np.linalg.norm(new_point - goal) < self.goal_tolerance:
                # Extract path by backtracking through parents
                path = self._extract_path(nodes, parents, len(nodes) - 1)
                return path
        
        # No path found within iteration limit
        return None
    
    def _interpolate_path(self, start, end, resolution):
        """Generate points along line from start to end"""
        distance = np.linalg.norm(end - start)
        num_points = int(distance / resolution) + 1
        points = []
        for i in range(num_points):
            alpha = i / max(num_points - 1, 1)
            point = start + alpha * (end - start)
            points.append(point)
        return np.array(points)
    
    def _path_in_collision(self, path_points, obstacle_tree, margin):
        """Check if any point on path is too close to obstacles"""
        dists, _ = obstacle_tree.query(path_points)
        return np.any(dists < margin)
    
    def _extract_path(self, nodes, parents, goal_idx):
        """Extract path from tree by following parent links"""
        path = []
        idx = goal_idx
        while idx != -1:
            path.append(nodes[idx])
            idx = parents[idx]
        path.reverse()
        return path

# -------------------------------------------------------------------
# CLASS: OccupancyGridBuilder
# -------------------------------------------------------------------
# Source: path_planner_complete.py
# -------------------------------------------------------------------
class OccupancyGridBuilder:
    """Build 2D occupancy grid from LiDAR scans"""
    
    def __init__(self, width=200, height=200, resolution=0.1, origin_x=-10.0, origin_y=-10.0):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.grid = np.zeros((height, width), dtype=np.int8)
        
    def update_from_scan(self, scan, robot_pose):
        """
        Update grid from LaserScan message
        
        Args:
            scan: LaserScan message
            robot_pose: [x, y, yaw] robot position and orientation
        """
        # Decay existing occupancy (dynamic obstacles)
        self.grid = (self.grid * 0.95).astype(np.int8)
        
        # Add new scan data
        angle = scan.angle_min
        for r in scan.ranges:
            if scan.range_min < r < scan.range_max:
                # Convert polar to Cartesian in robot frame
                x_robot = r * np.cos(angle)
                y_robot = r * np.sin(angle)
                
                # Transform to world frame
                x_world = robot_pose[0] + x_robot * np.cos(robot_pose[2]) - y_robot * np.sin(robot_pose[2])
                y_world = robot_pose[1] + x_robot * np.sin(robot_pose[2]) + y_robot * np.cos(robot_pose[2])
                
                # Convert to grid coordinates
                grid_x = int((x_world - self.origin_x) / self.resolution)
                grid_y = int((y_world - self.origin_y) / self.resolution)
                
                # Mark as occupied
                if 0 <= grid_x < self.width and 0 <= grid_y < self.height:
                    self.grid[grid_y, grid_x] = 100  # Occupied
            
            angle += scan.angle_increment
    
    def get_obstacles(self):
        """Get list of obstacle positions from grid"""
        obstacles = []
        occupied_cells = np.argwhere(self.grid > 50)  # Threshold at 50%
        
        for cell in occupied_cells:
            # Convert grid coords to world coords
            x = cell[1] * self.resolution + self.origin_x
            y = cell[0] * self.resolution + self.origin_y
            obstacles.append([x, y])
        
        return np.array(obstacles) if obstacles else np.array([])
    
    def to_occupancy_grid_msg(self, frame_id='map'):
        """Convert to ROS OccupancyGrid message"""
        msg = OccupancyGrid()
        msg.header.frame_id = frame_id
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.position.z = 0.0
        msg.data = self.grid.flatten().tolist()
        return msg

# -------------------------------------------------------------------
# CLASS: PathPlannerNode (Main Node)
# -------------------------------------------------------------------
# Source: path_planner_complete.py
# MODIFIED: Removed grid publisher, added CRC
# -------------------------------------------------------------------
class PathPlannerNode(Node):
    """Main path planning node for Ground PC"""
    
    def __init__(self):
        super().__init__('path_planner')
        
        # Declare parameters
        self.declare_parameter('planning.goal_x', 5.0)
        self.declare_parameter('planning.goal_y', 0.0)
        self.declare_parameter('planning.goal_z', 2.0)
        self.declare_parameter('planning.rate', 1.5)
        self.declare_parameter('rrt.max_iterations', 2000)
        self.declare_parameter('rrt.step_size', 0.6)
        self.declare_parameter('rrt.goal_sample_rate', 0.15)
        self.declare_parameter('rrt.goal_tolerance', 0.3)
        self.declare_parameter('occupancy.grid_width', 200)
        self.declare_parameter('occupancy.grid_height', 200)
        self.declare_parameter('occupancy.resolution', 0.1)
        self.declare_parameter('occupancy.origin_x', -10.0)
        self.declare_parameter('occupancy.origin_y', -10.0)
        
        # Get parameters
        self.goal_x = self.get_parameter('planning.goal_x').value
        self.goal_y = self.get_parameter('planning.goal_y').value
        self.goal_z = self.get_parameter('planning.goal_z').value
        planning_rate = self.get_parameter('planning.rate').value
        
        max_iter = self.get_parameter('rrt.max_iterations').value
        step_size = self.get_parameter('rrt.step_size').value
        goal_tol = self.get_parameter('rrt.goal_tolerance').value
        
        grid_w = self.get_parameter('occupancy.grid_width').value
        grid_h = self.get_parameter('occupancy.grid_height').value
        resolution = self.get_parameter('occupancy.resolution').value
        origin_x = self.get_parameter('occupancy.origin_x').value
        origin_y = self.get_parameter('occupancy.origin_y').value
        
        # Initialize components
        self.rrt = RRTPlanner(max_iter, step_size, goal_tol)
        self.grid_builder = OccupancyGridBuilder(grid_w, grid_h, resolution, origin_x, origin_y)
        
        # State
        self.current_pose = None  # [x, y, yaw]
        self.last_scan = None
        self.path_sequence = 0
        
        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        
        # Publishers
        self.path_pub = self.create_publisher(Float32MultiArray, '/path_commands', 10)
        
        # --- Visualization Publisher REMOVED as requested ---
        # self.grid_pub = self.create_publisher(OccupancyGrid, '/occupancy_grid', 10)
        
        # Timer for periodic planning
        plan_period = 1.0 / planning_rate
        self.create_timer(plan_period, self.plan_callback)
        
        self.get_logger().info(f'Path Planner initialized')
        self.get_logger().info(f'  Goal: ({self.goal_x:.2f}, {self.goal_y:.2f}, {self.goal_z:.2f})')
        self.get_logger().info(f'  RRT: max_iter={max_iter}, step={step_size:.2f}m')
        self.get_logger().info(f'  Grid: {grid_w}x{grid_h} @ {resolution:.2f}m/cell')
        self.get_logger().info(f'  Planning rate: {planning_rate:.1f} Hz')
    
    def scan_callback(self, msg):
        """Store latest LiDAR scan"""
        self.last_scan = msg
    
    def pose_callback(self, msg):
        """Store current robot pose"""
        # Extract position
        x = msg.pose.position.x
        y = msg.pose.position.y
        
        # Extract yaw from quaternion
        qw = msg.pose.orientation.w
        qz = msg.pose.orientation.z
        yaw = 2.0 * np.arctan2(qz, qw)
        
        self.current_pose = [x, y, yaw]
    
    def goal_callback(self, msg):
        """Handle new goal"""
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.goal_z = msg.pose.position.z
        self.get_logger().info(f'New goal received: ({self.goal_x:.2f}, {self.goal_y:.2f}, {self.goal_z:.2f})')
    
    def plan_callback(self):
        """Main planning loop - called at planning rate"""
        # Check prerequisites
        if self.current_pose is None:
            self.get_logger().warn('Waiting for robot pose...', throttle_duration_sec=5.0)
            return
        
        if self.last_scan is None:
            self.get_logger().warn('Waiting for LiDAR scan...', throttle_duration_sec=5.0)
            return
        
        # Update occupancy grid
        self.grid_builder.update_from_scan(self.last_scan, self.current_pose)
        obstacles = self.grid_builder.get_obstacles()
        
        # --- Visualization Publisher REMOVED as requested ---
        # grid_msg = self.grid_builder.to_occupancy_grid_msg()
        # grid_msg.header.stamp = self.get_clock().now().to_msg()
        # self.grid_pub.publish(grid_msg)
        
        # Check if already at goal
        start_2d = np.array(self.current_pose[:2])
        goal_2d = np.array([self.goal_x, self.goal_y])
        dist_to_goal = np.linalg.norm(goal_2d - start_2d)
        
        if dist_to_goal < 0.3:
            self.get_logger().info('Already at goal', throttle_duration_sec=5.0)
            return
        
        # Plan path using RRT
        start_time = time.time()
        path = self.rrt.plan(start_2d, goal_2d, obstacles)
        planning_time = time.time() - start_time
        
        if path is None:
            self.get_logger().warn(f'No path found after {planning_time:.3f}s')
            return
        
        # Log success
        self.get_logger().info(f'Path found: {len(path)} waypoints in {planning_time:.3f}s')
        
        # Publish path
        self.publish_path(path)
    
    def publish_path(self, path):
        """Publish path as Float32MultiArray"""
        msg = Float32MultiArray()
        
        # Header: [sequence, timestamp, num_waypoints]
        # Data payload starts here for CRC calculation
        data_payload = [
            float(self.path_sequence),
            self.get_clock().now().nanoseconds / 1e9,
            float(len(path))
        ]
        
        # Waypoints: [x1, y1, z1, x2, y2, z2, ...]
        for waypoint in path:
            data_payload.extend([
                float(waypoint[0]),
                float(waypoint[1]),
                float(self.goal_z)
            ])
        
        # --- CRC LOGIC ADDED ---
        # Calculate CRC on the data payload
        crc = MessageVerifier.calculate_crc(data_payload)
        
        # Create final message: [data_payload, crc]
        final_data = data_payload
        final_data.append(float(crc))
        
        msg.data = final_data
        self.path_pub.publish(msg)
        
        self.path_sequence += 1

# -------------------------------------------------------------------
# FUNCTION: main
# -------------------------------------------------------------------
# Source: path_planner_complete.py
# -------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
