#!/usr/bin/env python3
"""
Path Planner Node - Ground PC
Integrated RRT planner (KD-tree), MessageVerifier (CRC32), OccupancyGridBuilder.
Place at: groundpc/src/rrt_drone/rrt_drone/nodes/path_planner_node.py
"""

from __future__ import annotations

import time
import struct
import binascii
from typing import List, Tuple, Optional

import numpy as np
from scipy.spatial import cKDTree

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid

# ---------------------------
# MessageVerifier (CRC32)
# ---------------------------
# (from message_verifier_class.py). :contentReference[oaicite:3]{index=3}
class MessageVerifier:
    """Message integrity verification using CRC32 checksum."""

    @staticmethod
    def pack_with_crc(data_list: List[float]) -> Tuple[bytes, int]:
        packed = struct.pack(f'{len(data_list)}d', *[float(x) for x in data_list])
        crc = binascii.crc32(packed) & 0xFFFFFFFF
        return packed, crc

    @staticmethod
    def verify_crc(data_list: List[float], received_crc: int) -> bool:
        packed = struct.pack(f'{len(data_list)}d', *[float(x) for x in data_list])
        calculated_crc = binascii.crc32(packed) & 0xFFFFFFFF
        return calculated_crc == int(received_crc)

    @staticmethod
    def calculate_crc(data_list: List[float]) -> int:
        packed = struct.pack(f'{len(data_list)}d', *[float(x) for x in data_list])
        return binascii.crc32(packed) & 0xFFFFFFFF


# ---------------------------
# RRTPlanner (KD-tree optimized)
# ---------------------------
# Adapted from rrt_planner_complete.py. :contentReference[oaicite:4]{index=4}
class RRTPlanner:
    """
    Rapidly-exploring Random Tree path planner
    Optimized with KD-tree for fast nearest-neighbor queries.
    """

    def __init__(self,
                 max_iterations: int = 3000,
                 step_size: float = 0.5,
                 goal_bias: float = 0.1,
                 goal_tolerance: float = 0.3,
                 safety_margin: float = 0.5):
        self.max_iterations = int(max_iterations)
        self.step_size = float(step_size)
        self.goal_bias = float(goal_bias)
        self.goal_tolerance = float(goal_tolerance)
        self.safety_margin = float(safety_margin)

    def plan(self,
             start: Tuple[float, float],
             goal: Tuple[float, float],
             occupancy_grid: np.ndarray,
             bounds: Tuple[float, float, float, float]) -> Optional[List[Tuple[float, float]]]:
        """
        Plans a 2D path (x,y) from start -> goal using RRT avoiding occupied cells.
        occupancy_grid: 2D numpy array (height, width) with occupancy values 0-255.
        bounds: (xmin, xmax, ymin, ymax)
        Returns list of 2D points or None.
        """
        start = np.array(start, dtype=float)
        goal = np.array(goal, dtype=float)

        # quick success check
        if np.linalg.norm(goal - start) < self.goal_tolerance:
            return [tuple(start), tuple(goal)]

        # extract obstacles (world coords) from occupancy grid
        obstacles = self._extract_obstacles(occupancy_grid, bounds)

        obstacle_tree = cKDTree(obstacles) if obstacles.size else None

        # check start/goal collisions
        if obstacle_tree is not None:
            d_s, _ = obstacle_tree.query(start)
            d_g, _ = obstacle_tree.query(goal)
            if d_s < self.safety_margin or d_g < self.safety_margin:
                return None

        nodes = [start]
        parents = [-1]

        # main loop
        for it in range(self.max_iterations):
            if np.random.rand() < self.goal_bias:
                sample = goal
            else:
                sample = self._sample_random_point(bounds)

            # nearest via KD-tree on nodes
            if len(nodes) < 50:
                dists = np.linalg.norm(np.stack(nodes) - sample, axis=1)
                nearest_idx = int(np.argmin(dists))
            else:
                tree_nodes = cKDTree(np.stack(nodes))
                _, nearest_idx = tree_nodes.query(sample)

            nearest = nodes[nearest_idx]
            vec = sample - nearest
            dist = np.linalg.norm(vec)
            if dist == 0:
                continue
            if dist > self.step_size:
                new_pt = nearest + (vec / dist) * self.step_size
            else:
                new_pt = sample

            # collision free?
            if not self._is_collision_free(nearest, new_pt, obstacle_tree):
                continue

            nodes.append(new_pt)
            parents.append(nearest_idx)

            # reached goal?
            if np.linalg.norm(new_pt - goal) < self.goal_tolerance:
                path_nodes = self._extract_path(nodes, parents, len(nodes) - 1)
                return [tuple(p) for p in path_nodes]

        return None

    def _extract_obstacles(self, grid: np.ndarray, bounds: Tuple[float, float, float, float]) -> np.ndarray:
        xmin, xmax, ymin, ymax = bounds
        height, width = grid.shape
        occupied = np.argwhere(grid > 127)  # threshold
        if occupied.size == 0:
            return np.empty((0, 2), dtype=float)

        # occupied: array of [row, col] => convert to x,y world
        obstacles = []
        for row, col in occupied:
            x = xmin + (col + 0.5) * ((xmax - xmin) / width)
            y = ymin + (row + 0.5) * ((ymax - ymin) / height)
            obstacles.append([x, y])
        return np.array(obstacles, dtype=float)

    def _sample_random_point(self, bounds: Tuple[float, float, float, float]) -> np.ndarray:
        xmin, xmax, ymin, ymax = bounds
        return np.array([np.random.uniform(xmin, xmax), np.random.uniform(ymin, ymax)], dtype=float)

    def _is_collision_free(self, p1: np.ndarray, p2: np.ndarray, obstacle_tree: Optional[cKDTree]) -> bool:
        if obstacle_tree is None:
            return True

        # quick endpoint check
        d, _ = obstacle_tree.query(p2)
        if d < self.safety_margin:
            return False

        # sample along segment
        seg_len = np.linalg.norm(p2 - p1)
        checks = max(int(seg_len / 0.1), 1)
        for i in range(1, checks + 1):
            alpha = i / checks
            pt = p1 + alpha * (p2 - p1)
            d, _ = obstacle_tree.query(pt)
            if d < self.safety_margin:
                return False
        return True

    def _extract_path(self, nodes: List[np.ndarray], parents: List[int], goal_idx: int) -> List[np.ndarray]:
        path = []
        idx = goal_idx
        while idx != -1:
            path.append(nodes[idx])
            idx = parents[idx]
        path.reverse()
        return path


# ---------------------------
# OccupancyGridBuilder
# ---------------------------
# Adapted from your path_planner_complete.py. :contentReference[oaicite:5]{index=5}
class OccupancyGridBuilder:
    def __init__(self, width: int = 200, height: int = 200, resolution: float = 0.1,
                 origin_x: float = -10.0, origin_y: float = -10.0, decay_rate: float = 0.95):
        self.width = int(width)
        self.height = int(height)
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.decay_rate = float(decay_rate)
        self.grid = np.zeros((self.height, self.width), dtype=np.uint8)

    def update_from_scan(self, scan: LaserScan, robot_pose: Tuple[float, float, float]):
        # decay
        self.grid = (self.grid.astype(np.float32) * self.decay_rate).astype(np.uint8)

        angle = scan.angle_min
        for r in scan.ranges:
            if np.isfinite(r) and scan.range_min < r < scan.range_max:
                x_robot = r * np.cos(angle)
                y_robot = r * np.sin(angle)
                # world
                x_world = robot_pose[0] + x_robot * np.cos(robot_pose[2]) - y_robot * np.sin(robot_pose[2])
                y_world = robot_pose[1] + x_robot * np.sin(robot_pose[2]) + y_robot * np.cos(robot_pose[2])

                grid_x = int((x_world - self.origin_x) / self.resolution)
                grid_y = int((y_world - self.origin_y) / self.resolution)

                if 0 <= grid_x < self.width and 0 <= grid_y < self.height:
                    self.grid[grid_y, grid_x] = min(255, int(self.grid[grid_y, grid_x]) + 50)
            angle += scan.angle_increment

    def get_obstacles(self) -> np.ndarray:
        occupied = np.argwhere(self.grid > 50)
        obs = []
        for row, col in occupied:
            x = col * self.resolution + self.origin_x
            y = row * self.resolution + self.origin_y
            obs.append([x, y])
        return np.array(obs, dtype=float) if obs else np.empty((0, 2), dtype=float)

    def to_occupancy_grid_msg(self, frame_id: str = 'map') -> OccupancyGrid:
        """
        Convert internal uint8 grid (0-255) into ROS nav_msgs/OccupancyGrid format.
        Mapping:
            0        -> 0    (free)
            1..50    -> -1   (unknown / low confidence)
            >50      -> 100  (occupied)
        NOTE: The node sets the header.stamp right before publishing.
        """
        msg = OccupancyGrid()
        # header.stamp is set by the caller (node) so we only set frame_id
        msg.header.frame_id = frame_id

        # Map metadata
        msg.info.resolution = float(self.resolution)
        msg.info.width = int(self.width)
        msg.info.height = int(self.height)

        # Set origin (position + identity orientation)
        msg.info.origin.position.x = float(self.origin_x)
        msg.info.origin.position.y = float(self.origin_y)
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.x = 0.0
        msg.info.origin.orientation.y = 0.0
        msg.info.origin.orientation.z = 0.0
        msg.info.origin.orientation.w = 1.0

        # Convert internal grid (uint8 0..255) -> ROS occupancy values (-1,0,100)
        # We iterate row-major (same order as numpy.flatten(order='C'))
        flat = []
        # Thresholds — tuned so faint detections become UNKNOWN, strong detections OCCUPIED
        OCC_THRESHOLD = 50     # cell value > OCC_THRESHOLD => occupied
        for row in range(self.height):
            # iterate columns for row-major ordering
            for col in range(self.width):
                v = int(self.grid[row, col])  # 0..255
                if v == 0:
                    flat.append(0)      # free
                elif v > OCC_THRESHOLD:
                    flat.append(100)    # occupied
                else:
                    flat.append(-1)     # unknown / low confidence

        # Ensure length matches width*height
        if len(flat) != msg.info.width * msg.info.height:
            # fallback: fill unknowns (shouldn't happen)
            flat = [-1] * (msg.info.width * msg.info.height)

        msg.data = flat
        return msg



# ---------------------------
# PathPlannerNode
# ---------------------------
class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner')

        # Parameters (match YAML keys)
        self.declare_parameter('rrt.max_iterations', 3000)
        self.declare_parameter('rrt.step_size', 0.5)
        self.declare_parameter('rrt.goal_sample_rate', 0.1)
        self.declare_parameter('rrt.goal_tolerance', 0.3)
        self.declare_parameter('planning.rate', 2.0)
        self.declare_parameter('planning.goal_x', 5.0)
        self.declare_parameter('planning.goal_y', 0.0)
        self.declare_parameter('planning.goal_z', 2.0)
        self.declare_parameter('occupancy.grid_width', 200)
        self.declare_parameter('occupancy.grid_height', 200)
        self.declare_parameter('occupancy.resolution', 0.1)
        self.declare_parameter('occupancy.origin_x', -10.0)
        self.declare_parameter('occupancy.origin_y', -10.0)
        self.declare_parameter('occupancy.decay_rate', 0.95)
        self.declare_parameter('health.check_rate', 10.0)

        # Read parameters
        max_iter = int(self.get_parameter('rrt.max_iterations').value)
        step_size = float(self.get_parameter('rrt.step_size').value)
        goal_tol = float(self.get_parameter('rrt.goal_tolerance').value)
        rate = float(self.get_parameter('planning.rate').value)

        grid_w = int(self.get_parameter('occupancy.grid_width').value)
        grid_h = int(self.get_parameter('occupancy.grid_height').value)
        resolution = float(self.get_parameter('occupancy.resolution').value)
        origin_x = float(self.get_parameter('occupancy.origin_x').value)
        origin_y = float(self.get_parameter('occupancy.origin_y').value)
        decay_rate = float(self.get_parameter('occupancy.decay_rate').value)

        # Components
        self.rrt = RRTPlanner(max_iterations=max_iter, step_size=step_size,
                              goal_bias=float(self.get_parameter('rrt.goal_sample_rate').value),
                              goal_tolerance=goal_tol)
        self.grid_builder = OccupancyGridBuilder(width=grid_w, height=grid_h,
                                                 resolution=resolution,
                                                 origin_x=origin_x, origin_y=origin_y,
                                                 decay_rate=decay_rate)

        # State
        self.current_pose: Optional[Tuple[float, float, float]] = None
        self.last_scan: Optional[LaserScan] = None
        self.path_sequence = 0

        # Subscribers
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self._pose_cb, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self._goal_cb, 10)

        # Publishers
        self.path_pub = self.create_publisher(Float32MultiArray, '/path_commands', 10)
        self.grid_pub = self.create_publisher(OccupancyGrid, '/occupancy_grid', 10)

        # Timer
        self.create_timer(1.0 / rate, self._plan_cycle)

        self.get_logger().info('Path Planner initialized.')
        self.get_logger().info(f'Grid {grid_w}x{grid_h} @ {resolution}m')

    # ---------------------------
    # Callbacks
    # ---------------------------
    def _scan_cb(self, msg: LaserScan):
        self.last_scan = msg

    def _pose_cb(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        # yaw extraction (safe)
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = float(np.arctan2(siny_cosp, cosy_cosp))
        self.current_pose = (float(x), float(y), yaw)

    def _goal_cb(self, msg: PoseStamped):
        self.get_logger().info('Received external goal')
        self.declare_parameter('planning.goal_x', msg.pose.position.x)
        self.declare_parameter('planning.goal_y', msg.pose.position.y)
        self.declare_parameter('planning.goal_z', msg.pose.position.z)

    # ---------------------------
    # Planning cycle
    # ---------------------------
    def _plan_cycle(self):
        # prerequisites
        if self.current_pose is None:
            self.get_logger().warn('Waiting for robot pose...', throttle_duration_sec=5.0)
            return
        if self.last_scan is None:
            self.get_logger().warn('Waiting for LiDAR...', throttle_duration_sec=5.0)
            return

        # update grid
        self.grid_builder.update_from_scan(self.last_scan, self.current_pose)
        obstacles = self.grid_builder.get_obstacles()

        # occupancy msg (optional viz)
        try:
            grid_msg = self.grid_builder.to_occupancy_grid_msg()
            grid_msg.header.stamp = self.get_clock().now().to_msg()
            self.grid_pub.publish(grid_msg)
        except Exception:
            # visualization optional — ignore if conversion fails
            pass

        # prepare planning inputs
        start = (self.current_pose[0], self.current_pose[1])
        goal_x = float(self.get_parameter('planning.goal_x').value)
        goal_y = float(self.get_parameter('planning.goal_y').value)
        goal = (goal_x, goal_y)

        # bounds derived from occupancy grid
        xmin = float(self.get_parameter('occupancy.origin_x').value)
        ymin = float(self.get_parameter('occupancy.origin_y').value)
        xmax = xmin + self.grid_builder.width * self.grid_builder.resolution
        ymax = ymin + self.grid_builder.height * self.grid_builder.resolution
        bounds = (xmin, xmax, ymin, ymax)

        # quick check
        dist = np.linalg.norm(np.array(goal) - np.array(start))
        if dist < float(self.get_parameter('rrt.goal_tolerance').value):
            self.get_logger().info('Already at goal', throttle_duration_sec=5.0)
            return

        t0 = time.time()
        path = self.rrt.plan(start, goal, self.grid_builder.grid, bounds)
        t_elap = time.time() - t0

        if path is None:
            self.get_logger().warn(f'No path found (t={t_elap:.3f}s)')
            return

        self.get_logger().info(f'Path found: {len(path)} waypoints in {t_elap:.3f}s')

        # publish path (append CRC)
        self._publish_path(path)

    # ---------------------------
    # Publish with CRC appended
    # ---------------------------
    def _publish_path(self, path: List[Tuple[float, float]]):
        # Format: [seq, timestamp, num_waypoints, x1,y1,z1, x2,y2,z2, ..., CRC]
        data: List[float] = []
        data.append(float(self.path_sequence))
        data.append(float(self.get_clock().now().nanoseconds) / 1e9)
        data.append(float(len(path)))

        # append waypoints; z uses planning.goal_z
        goal_z = float(self.get_parameter('planning.goal_z').value)
        for (x, y) in path:
            data.extend([float(x), float(y), float(goal_z)])

        # compute CRC (pack as doubles)
        _, crc = MessageVerifier.pack_with_crc(data)
        data.append(float(crc))

        msg = Float32MultiArray()
        msg.data = [float(x) for x in data]
        self.path_pub.publish(msg)
        self.path_sequence += 1

    # ---------------------------
    # Shutdown
    # ---------------------------
    def destroy_node(self):
        try:
            super().destroy_node()
        except Exception:
            pass


# ---------------------------
# Main
# ---------------------------
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
