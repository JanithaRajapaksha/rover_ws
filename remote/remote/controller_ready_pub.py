#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class NodeReadyPublisher(Node):
    def __init__(self):
        super().__init__('node_ready_publisher')
        self.pub = self.create_publisher(String, '/nodes_ready', 10)
        self.timer = self.create_timer(1.0, self.check_nodes)  # check 1 Hz

        # Node names to check
        self.target_nodes = ['diff_cont', 'joint_broad']

    def check_nodes(self):
        try:
            all_nodes = self.get_node_names_and_namespaces()
            active_node_names = [name for name, ns in all_nodes]

            msg = String()
            if all(node in active_node_names for node in self.target_nodes):
                msg.data = 'ready'
            else:
                msg.data = 'not_ready'

            self.pub.publish(msg)
            # self.get_logger().info(f"Active nodes: {active_node_names}. Published: {msg.data}")

        except Exception as e:
            self.get_logger().error(f"Error checking nodes: {e}")
            msg = String()
            msg.data = 'not_ready'
            self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = NodeReadyPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
