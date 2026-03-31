#!/usr/bin/env python3
"""
Command Executor Node - Raspberry Pi
Receives path commands and sends control setpoints to flight controller via MAVROS
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool
from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.msg import State
import numpy as np
import time
import struct
import zlib

try:
    from px4_msgs.msg import OffboardControlMode
except ImportError:
    OffboardControlMode = None
    
class CommandExecutorNode(Node):
    def __init__(self):
        super().__init__('command_executor')
        
        # Parameters
        self.declare_parameter('planning.rate', 2.0)
        
        # State
        self.current_waypoint_idx = 0
        self.waypoints = []
        self.mavros_state = None
        self.current_pose = None
        self.last_command_time = self.get_clock().now()
        
        # Subscribers
        self.create_subscription(Float64MultiArray, '/path_commands', self.path_callback, 10)
        self.create_subscription(State, '/mavros/state', self.state_callback, 10)
        self.create_subscription(Bool, '/emergency_status', self.emergency_callback, 10)
        
        # Publishers
        self.velocity_pub = self.create_publisher(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)

        if OffboardControlMode is not None:
            self.offboard_mode_pub = self.create_publisher(
                OffboardControlMode, '/mavros/offboard_control_mode', 10)
        else:
            self.offboard_mode_pub = None

        # Fix #7 - ACK publisher
        self.ack_pub = self.create_publisher(
            Float64MultiArray, '/path_command_ack', 10)

        # Fix #3 - pose subscription
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self.pose_callback, 10)
        
        # Timer for setpoint publishing (20 Hz for MAVROS offboard mode)
        self.create_timer(0.05, self.publish_setpoint)
        
        self.get_logger().info('Command Executor initialized')
    
    def path_callback(self, msg):
        """Receive new path from planner"""
        data = msg.data
        if len(data) < 3:
            return
        
        sequence = int(data[0])
        timestamp = data[1]
        num_waypoints = int(data[2])
        
        # Extract waypoints
        self.waypoints = []
        idx = 3
        for i in range(num_waypoints):
            if idx + 2 < len(data):
                wp = [data[idx], data[idx+1], data[idx+2]]
                self.waypoints.append(wp)
                idx += 3
        
        self.current_waypoint_idx = 0
        self.last_command_time = self.get_clock().now()
        
        self.get_logger().info(f'Received path: {num_waypoints} waypoints')
        
        # fix send ack back to planner
        ack = Float64MultiArray()
        ack.data = [float(sequence)]
        self.ack_pub.publish(ack)
    
    def state_callback(self, msg):
        """Receive MAVROS state"""
        self.mavros_state = msg
    
    def emergency_callback(self, msg):
        """Emergency stop"""
        if msg.data:
            self.get_logger().warn('Emergency stop activated!')
            self.waypoints = []
    def pose_callback(self, msg):
        """Fix #3 - track actual drone position"""
        self.current_pose = msg
        
    def publish_setpoint(self):
        """Publish velocity setpoints at 20 Hz"""

        # Fix #1 - OffboardControlMode heartbeat (PX4 only)
        if self.offboard_mode_pub is not None:
            om = OffboardControlMode()
            om.position     = False
            om.velocity     = True
            om.acceleration = False
            om.attitude     = False
            om.body_rate    = False
            om.timestamp    = int(self.get_clock().now().nanoseconds / 1000)
            self.offboard_mode_pub.publish(om)
            
        # Check if we have waypoints
        if not self.waypoints:
            # Publish zero velocity to maintain offboard mode
            twist = TwistStamped()
            twist.header.stamp = self.get_clock().now().to_msg()
            twist.header.frame_id = 'base_link'
            twist.twist.linear.x = 0.0
            twist.twist.linear.y = 0.0
            twist.twist.linear.z = 0.0
            twist.twist.angular.x = 0.0
            twist.twist.angular.y = 0.0
            twist.twist.angular.z = 0.0
            self.velocity_pub.publish(twist)
            return
        
        # Get current waypoint
        if self.current_waypoint_idx >= len(self.waypoints):
            self.waypoints = []
            return
        
        target = self.waypoints[self.current_waypoint_idx]
        
        # Simple proportional controller
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = 'base_link'
        
        # Velocity = K * (target - current)
        # For now, use waypoint directly as velocity direction
        twist.twist.linear.x = target[0] * 0.5  # Scale down
        twist.twist.linear.y = target[1] * 0.5
        twist.twist.linear.z = target[2] * 0.3
        twist.twist.angular.x = 0.0
        twist.twist.angular.y = 0.0
        twist.twist.angular.z = 0.0
        
        self.velocity_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = CommandExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
