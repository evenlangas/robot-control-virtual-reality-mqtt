#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import String
from tm_msgs.srv import SetPositions

class unitySubscriberNode(Node):
    def __init__(self):
        super().__init__("unity_subscriber")
        self.get_logger().info("Node has been started")
        self.unity_subscriber_ = self.create_subscription(String, "/unity_to_cobot", self.unity_callback, 10)
        self.unity_client = self.create_client(SetPositions, "set_positions")

        while not self.unity_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
         
        self.req = SetPositions.Request()
        self.floats = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        #Choose motion type: 1.PTP_J 2.LINE_T
        self.motion = 2

        #if self.unity_client.wait_for_service(timeout_sec=1.0):
        #    self.send_initial_position()

    def unity_callback(self, msg: String):
        # Convert each string to float and then to radians
        position_list = msg.data.split('#')
        
        if self.motion == 1:
            #PTP
            self.get_logger().info("Motion type: PTP_J")

            for i in range(6):
                self.floats[i] = math.radians(float(position_list[i]))
            
        elif self.motion == 2:
            #LINE
            self.get_logger().info("Motion type: PLINE_T")

            for i in range(3):
                self.floats[i] = float(position_list[i])

            for i in range(3, 6):
                self.floats[i] = math.radians(float(position_list[i]))
   
        else:
            self.get_logger().warn("Incorrect motion type")


        self.get_logger().info(f'Sending positions: {self.floats}')

        if len(self.floats) != 6:
            self.get_logger().error(f'Incorrect number of positions: {len(self.floats)}. Expected: {6}')
        else:  
            if self.motion == 1:
                self.send_request_PTP_J()
            elif self.motion == 2:
                self.send_request_LINE_T()

    def send_request_LINE_T(self):
        self.req.motion_type = SetPositions.Request.LINE_T
        self.req.positions = self.floats
        self.req.velocity = 0.8
        self.req.acc_time = 0.5
        self.req.blend_percentage = 90
        self.req.fine_goal = False
        self.future = self.unity_client.call_async(self.req)
        self.future.add_done_callback(self.response_callback)

    def send_request_PTP_J(self):
        self.req.motion_type = SetPositions.Request.PTP_J
        self.req.positions = self.floats
        self.req.velocity = 0.8
        self.req.acc_time = 0.2
        self.req.blend_percentage = 80
        self.req.fine_goal = False
        self.future = self.unity_client.call_async(self.req)
        self.future.add_done_callback(self.response_callback)

    def send_initial_position(self):
        self.req.motion_type = SetPositions.Request.PTP_J
        self.req.positions = [-81.0, -13.0, 97.0, -2.0, 86.0, 1.0]
        self.req.velocity = 0.8
        self.req.acc_time = 0.2
        self.req.blend_percentage = 10
        self.req.fine_goal = False
        self.future = self.unity_client.call_async(self.req)
        self.future.add_done_callback(self.response_callback)

        while self.future.done() is False:
            rclpy.spin_once(self)

    
    def response_callback(self, future):
        try:
            response = future.result()
            if response.ok:
                self.get_logger().info('Movement complete and successful')
            else:
                self.get_logger().info('Movement complete but not successful')
        except Exception as e:
            self.get_logger().error('Service call failed %r' % (e,))

def main(args=None):
    rclpy.init(args=args)
    node = unitySubscriberNode()

    rclpy.spin(node)  # Changed to rclpy.spin to handle callbacks

    rclpy.shutdown()