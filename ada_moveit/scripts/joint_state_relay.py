#!/usr/bin/env python3
"""
Simple relay node that subscribes to joint_states with TRANSIENT_LOCAL QoS
and republishes with VOLATILE QoS to ensure compatibility with all subscribers.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState


class JointStateRelay(Node):
    def __init__(self):
        super().__init__('joint_state_relay')
        
        # Subscribe with TRANSIENT_LOCAL to receive from joint_state_broadcaster
        sub_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )
        
        # Publish with VOLATILE for standard ROS compatibility
        pub_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )
        
        # Track which joints we've seen to merge messages
        self.latest_joint_states = {}
        
        # Subscribe to joint_state_broadcaster output (TRANSIENT_LOCAL)
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            sub_qos
        )
        
        # Republish with VOLATILE QoS
        self.publisher = self.create_publisher(
            JointState,
            '/joint_states_relay',
            pub_qos
        )
        
        self.get_logger().info('Joint state relay started')
    
    def joint_state_callback(self, msg):
        # Simply relay the message
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
