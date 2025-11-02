#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class BaseLinkPosePublisher(Node):
    def __init__(self):
        super().__init__('base_link_pose_pub')
        self.publisher_ = self.create_publisher(PoseStamped, '/base_link_target', 10)
        self.timer = self.create_timer(1.0, self.publish_pose)

    def publish_pose(self):
        pose = PoseStamped()
        pose.header.frame_id = 'base_link'  # relative to robot
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = 1.0  # 1 meter in front of robot
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0  # facing same direction

        self.publisher_.publish(pose)
        self.get_logger().info('Published pose relative to base_link')

def main(args=None):
    rclpy.init(args=args)
    node = BaseLinkPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
