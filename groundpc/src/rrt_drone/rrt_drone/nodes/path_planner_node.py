# ============================================================================
# CRITICAL FIX #2: CORRECTED OCCUPANCY GRID BUILDER
# ============================================================================

class OccupancyGridBuilder:
    """
    Build occupancy grid with CORRECT coordinate transforms
    Fixes coordinate mismatch bug
    """
    
    def __init__(self, grid_size=(200, 200), resolution=0.1, origin=(-10.0, -10.0)):
        self.grid_size = grid_size
        self.resolution = resolution
        self.origin = origin
        self.grid = np.zeros(grid_size, dtype=np.uint8)
        
    def update_from_scan(self, scan: LaserScan, robot_pose: Tuple[float, float, float]):
        """
        CRITICAL FIX #2: Correct coordinate transformation
        robot_pose: (x, y, theta) in world frame
        """
        ranges = np.array(scan.ranges)
        angle_min = scan.angle_min
        angle_increment = scan.angle_increment
        
        robot_x, robot_y, robot_theta = robot_pose
        
        # Decay old observations
        self.grid = (self.grid * 0.95).astype(np.uint8)
        
        for i, r in enumerate(ranges):
            if r < scan.range_min or r > scan.range_max or np.isnan(r) or np.isinf(r):
                continue
            
            # Calculate obstacle position in WORLD frame
            angle = angle_min + i * angle_increment + robot_theta
            obs_x_world = robot_x + r * np.cos(angle)
            obs_y_world = robot_y + r * np.sin(angle)
            
            # CRITICAL FIX #2: Convert world coordinates to grid coordinates
            grid_x = int((obs_x_world - self.origin[0]) / self.resolution)
            grid_y = int((obs_y_world - self.origin[1]) / self.resolution)
            
            # Clip to valid grid range
            grid_x = np.clip(grid_x, 0, self.grid_size[0] - 1)
            grid_y = np.clip(grid_y, 0, self.grid_size[1] - 1)
            
            # Update occupancy
            self.grid[grid_x, grid_y] = min(255, self.grid[grid_x, grid_y] + 50)
        
        return self.grid.copy()

# ============================================================================
# CRITICAL FIX #1, #3, #4: PATH PLANNER NODE (FULLY CORRECTED)
# ============================================================================

class PathPlannerNode(Node):
    """
    Path planning node on Ground PC
    ALL CRITICAL FIXES APPLIED
    """
    
    def __init__(self):
        super().__init__('path_planner')
        
        # Declare parameters
        self.declare_parameter('planning.goal_x', 5.0)
        self.declare_parameter('planning.goal_y', 0.0)
        self.declare_parameter('planning.goal_z', 2.0)
        self.declare_parameter('planning.rate', 2.0)
        self.declare_parameter('rrt.max_iterations', 3000)
        self.declare_parameter('rrt.step_size', 0.5)
        
        # Get parameters
        self.goal = (
            self.get_parameter('planning.goal_x').value,
            self.get_parameter('planning.goal_y').value,
            self.get_parameter('planning.goal_z').value
        )
        planning_rate = self.get_parameter('planning.rate').value
        
        # Initialize planner with KD-tree
        self.rrt_planner = RRTPlanner(
            max_iterations=self.get_parameter('rrt.max_iterations').value,
            step_size=self.get_parameter('rrt.step_size').value
        )
        
        self.occupancy_builder = OccupancyGridBuilder()
        
        # CRITICAL FIX #3: Initialize robot pose (will be updated)
        self.robot_pose = (0.0, 0.0, 0.0)
        self.pose_received = False
        
        self.current_scan = None
        self.command_sequence = 0
        
        # Retransmission tracking (FIX #7)
        self.pending_acks = {}
        self.ack_timeout = 0.5  # 500ms
        
        # QoS with liveliness
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            liveliness=QoSLivelinessPolicy.AUTOMATIC,
            liveliness_lease_duration=rclpy.duration.Duration(seconds=1.0)
        )
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.on_scan_received, qos
        )
        
        # CRITICAL FIX #3: Subscribe to robot pose
        self.pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self.on_pose_received, 10
        )
        
        # ACK subscriber (FIX #7)
        self.ack_sub = self.create_subscription(
            Float64MultiArray, '/path_command_ack',
            self.on_ack_received, 10
        )
        
        # Publishers
        self.path_pub = self.create_publisher(
            Float64MultiArray, '/path_commands', qos
        )
        
        self.occupancy_pub = self.create_publisher(
            OccupancyGrid, '/occupancy_grid', 10
        )
        
        # Timers
        self.planning_timer = self.create_timer(
            1.0 / planning_rate, self.planning_cycle
        )
        
        # Retransmission check timer (FIX #7)
        self.retransmit_timer = self.create_timer(0.1, self.check_retransmissions)
        
        self.get_logger().info(f'Path Planner initialized. Goal: {self.goal}')
    
    def on_scan_received(self, msg):
        """Store latest scan"""
        self.current_scan = msg
    
    def on_pose_received(self, msg: PoseStamped):
        """
        CRITICAL FIX #3: Update robot pose from MAVROS
        """
        # Extract position
        self.robot_pose = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        )
        
        # Extract yaw from quaternion
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        # Update pose with yaw
        self.robot_pose = (msg.pose.position.x, msg.pose.position.y, yaw)
        
        if not self.pose_received:
            self.get_logger().info(f'Robot pose received: {self.robot_pose}')
            self.pose_received = True
    
    def on_ack_received(self, msg: Float64MultiArray):
        """
        FIX #7: Handle acknowledgment from RPi
        """
        if len(msg.data) < 1:
            return
        
        ack_seq = int(msg.data[0])
        if ack_seq in self.pending_acks:
            del self.pending_acks[ack_seq]
            self.get_logger().debug(f'ACK received for command {ack_seq}')
    
    def planning_cycle(self):
        """Execute planning cycle"""
        if self.current_scan is None:
            self.get_logger().warn('Waiting for LiDAR data...')
            return
        
        if not self.pose_received:
            self.get_logger().warn('Waiting for robot pose...')
            return
        
        start_time = time.time()
        
        # Build occupancy grid with CORRECT coordinates (FIX #2)
        occupancy_grid = self.occupancy_builder.update_from_scan(
            self.current_scan, self.robot_pose
        )
        
        # CRITICAL FIX #4: Publish occupancy grid
        self.publish_occupancy_grid(occupancy_grid)
        
        # Plan path with optimized RRT (FIX #5)
        bounds = (-10, 10, -10, 10, 0.5, 5.0)
        path = self.rrt_planner.plan(
            self.robot_pose, self.goal, occupancy_grid, bounds
        )
        
        planning_time = time.time() - start_time
        
        if path is None:
            self.get_logger().warn(f'No path found (time: {planning_time:.3f}s)')
            return
        
        self.get_logger().info(
            f'Path found: {len(path)} waypoints in {planning_time:.3f}s'
        )
        
        # Publish with real retransmission (FIX #7)
        self.publish_path_with_retry(path)
    
    def publish_occupancy_grid(self, grid: np.ndarray):
        """
        CRITICAL FIX #4: Implement missing method
        """
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        msg.info.resolution = self.occupancy_builder.resolution
        msg.info.width = self.occupancy_builder.grid_size[0]
        msg.info.height = self.occupancy_builder.grid_size[1]
        msg.info.origin.position.x = self.occupancy_builder.origin[0]
        msg.info.origin.position.y = self.occupancy_builder.origin[1]
        msg.info.origin.position.z = 0.0
        
        # Convert to ROS occupancy format (0-100, -1 for unknown)
        grid_normalized = (grid / 255.0 * 100.0).astype(np.int8)
        msg.data = grid_normalized.flatten().tolist()
        
        self.occupancy_pub.publish(msg)
    
    def publish_path_with_retry(self, waypoints: List[Tuple[float, float, float]]):
        """
        FIX #7: Real retransmission with ACK timeout
        """
        # Create message
        msg_data = [
            float(self.command_sequence),
            time.time(),
            float(len(waypoints[:5]))
        ]
        
        for wp in waypoints[:5]:
            msg_data.extend([wp[0], wp[1], wp[2]])
        
        msg_data.extend([1.0, 0.0, 0.0])  # Velocity
        msg_data.append(0.0)  # Heading
        msg_data.append(500.0)  # Duration
        
        # FIX #9: Add CRC
        _, crc = MessageVerifier.pack_with_crc(msg_data)
        msg_data.append(float(crc))
        
        msg = Float64MultiArray()
        msg.data = msg_data
        
        # Track for retransmission
        self.pending_acks[self.command_sequence] = {
            'msg': msg,
            'timestamp': time.time(),
            'attempts': 0
        }
        
        self.path_pub.publish(msg)
        self.command_sequence += 1
    
    def check_retransmissions(self):
        """
        FIX #7: Check for timeout and retransmit
        """
        current_time = time.time()
        to_remove = []
        
        for seq, info in self.pending_acks.items():
            age = current_time - info['timestamp']
            
            if age > self.ack_timeout:
                if info['attempts'] < 3:
                    # Retransmit
                    self.path_pub.publish(info['msg'])
                    info['timestamp'] = current_time
                    info['attempts'] += 1
                    self.get_logger().warn(f'Retransmitting command {seq}')
                else:
                    # Give up after 3 attempts
                    self.get_logger().error(f'Command {seq} failed after 3 attempts')
                    to_remove.append(seq)
        
        for seq in to_remove:
            del self.pending_acks[seq]

