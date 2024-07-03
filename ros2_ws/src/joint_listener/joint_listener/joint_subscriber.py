#!/usr/bun/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import numpy as np

class JoinSubscriberNode(Node):
    def __init__(self):
        super().__init__("joint_subscriber")
        self.node_red_publisher_ = self.create_publisher(String, "/cobot_to_unity", 10)
        self.joint_subscriber_ = self.create_subscription(JointState, "/joint_states", self.joint_callback, 10)
        self.cmd = String()
        self.previous_position_deg = np.zeros(6)  # Assuming 6 joints
        self.moving = False
        self.stable_iterations = 0
        self.required_stable_iterations = 5  # Number of iterations required to consider movement has stopped

    def joint_callback(self, msg: JointState):
        current_positions_deg = np.array([round(angle * (180 / math.pi), 2) for angle in msg.position[:6]])

        if not np.allclose(current_positions_deg, self.previous_position_deg, atol=0.01):
            if not self.moving:
                self.moving = True
                self.stable_iterations = 0  # Reset the counter as there is movement
                self.get_logger().info("Robot started moving")
            else:
                # If already moving, just reset the stable_iterations
                self.stable_iterations = 0
        else:
            if self.moving and self.stable_iterations < self.required_stable_iterations:
                # Increase the stable counter if positions are close and the robot was moving
                self.stable_iterations += 1
                # Check if the robot has been stable for enough iterations to consider it stopped
                if self.stable_iterations >= self.required_stable_iterations:
                    self.moving = False
                    self.get_logger().info("Robot stopped moving")
                    self.cmd.data = "STOPPED"
                    self.node_red_publisher_.publish(self.cmd)
            elif not self.moving:
                # If the robot was not considered moving, reset the stable_iterations
                self.stable_iterations = 0

        self.cmd.data = "#".join(str(deg) for deg in current_positions_deg)
        #self.get_logger().info("Current: " + str(current_positions_deg) + "\n" + "Previous: " + str(self.previous_position_deg) + "\n")
        self.node_red_publisher_.publish(self.cmd)
        self.previous_position_deg = current_positions_deg


def main(args=None):
    rclpy.init(args=args)
    node = JoinSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()