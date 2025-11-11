
#!/usr/bin/env python3
"""
Health Monitor Node - Raspberry Pi
Monitors system health and triggers emergency procedures
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from mavros_msgs.msg import State
from geometry_msgs.msg import TwistStamped

class HealthMonitorNode(Node):
    def __init__(self):
        super().__init__('health_monitor')
        
        # Parameters
        self.declare_parameter('health.check_rate', 10.0)
        self.declare_parameter('health.lidar_timeout', 0.5)
        self.declare_parameter('health.command_timeout', 2.0)
        
        check_rate = self.get_parameter('health.check_rate').value
        self.lidar_timeout = self.get_parameter('health.lidar_timeout').value
        self.command_timeout = self.get_parameter('health.command_timeout').value
        
        # State
        self.last_lidar_time = self.get_clock().now()
        self.last_command_time = self.get_clock().now()
        self.mavros_connected = False
        
        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.create_subscription(State, '/mavros/state', self.mavros_state_callback, 10)
        self.create_subscription(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', self.command_callback, 10)
        
        # Publishers
        self.health_pub = self.create_publisher(Bool, '/system_healthy', 10)
        self.emergency_pub = self.create_publisher(Bool, '/emergency_status', 10)
        
        # Timer
        self.create_timer(1.0 / check_rate, self.check_health)
        
        self.get_logger().info('Health Monitor initialized')
    
    def lidar_callback(self, msg):
        """Update LiDAR last seen time"""
        self.last_lidar_time = self.get_clock().now()
    
    def mavros_state_callback(self, msg):
        """Update MAVROS connection status"""
        self.mavros_connected = msg.connected
    
    def command_callback(self, msg):
        """Update command last seen time"""
        self.last_command_time = self.get_clock().now()
    
    def check_health(self):
        """Check system health"""
        now = self.get_clock().now()
        healthy = True
        emergency = False
        
        # Check LiDAR timeout
        lidar_dt = (now - self.last_lidar_time).nanoseconds / 1e9
        if lidar_dt > self.lidar_timeout:
            self.get_logger().warn(f'LiDAR timeout: {lidar_dt:.2f}s')
            healthy = False
        
        # Check command timeout
        cmd_dt = (now - self.last_command_time).nanoseconds / 1e9
        if cmd_dt > self.command_timeout:
            self.get_logger().warn(f'Command timeout: {cmd_dt:.2f}s')
            healthy = False
            emergency = True  # Trigger emergency landing
        
        # Check MAVROS connection
        if not self.mavros_connected:
            self.get_logger().warn('MAVROS not connected')
            healthy = False
        
        # Publish health status
        health_msg = Bool()
        health_msg.data = healthy
        self.health_pub.publish(health_msg)
        
        # Publish emergency status
        if emergency:
            emerg_msg = Bool()
            emerg_msg.data = True
            self.emergency_pub.publish(emerg_msg)
            self.get_logger().error('EMERGENCY: Triggering emergency landing')

def main(args=None):
    rclpy.init(args=args)
    node = HealthMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
