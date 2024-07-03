#!/usr/bun/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class JoinSubscriberNode(Node):
    def __init__(self):
        super().__init__("test_greie")

        self.node_red_test_publisher_ = self.create_publisher(
            String, "/test_til_nodered", 10)
        
        self.timer_ = self.create_timer(4, self.print_dritt)

        self.node_red_test_subscriber_ = self.create_subscription(
            String, "/test_til_ros2", self.motta_dritt, 10)
        
    def motta_dritt(self, msg: String):

        self.get_logger().info("You got mail: " + str(msg.data))

    def print_dritt(self):
        melding = String()
        melding.data = "Melding fra ros2, kan du høre meg?"
        self.get_logger().info("Publishing message to '/test_til_nodered'")
        self.node_red_test_publisher_.publish(melding)
        


def main(args=None):
    rclpy.init(args=args)
    node = JoinSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()