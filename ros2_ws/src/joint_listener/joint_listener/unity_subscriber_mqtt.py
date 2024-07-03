#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import String
from tm_msgs.srv import SetPositions
from tm_msgs.srv import SetEvent
import numpy as np
import json 

class unitySubscriberMqttNode(Node):
    def __init__(self):
        super().__init__("unity_subscriber_mqtt")
        self.get_logger().info("Node has been started")
        self.unity_subscriber_ = self.create_subscription(String, "/unity_to_cobot_mqtt", self.unity_callback, 10)
        self.unity_client = self.create_client(SetPositions, "set_positions")
        # self.set_event_client = self.create_client(SetEvent, "set_event")
        #Test å sette stop commandoen

        while not self.unity_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("set position service not available, waiting again...")

        # while not self.set_event_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info("set io service not available, waiting again...")

        self.req = SetPositions.Request()
        # self.req_event = SetEvent.Request()

        # self.req_io.module = SetIO.Request.MODULE_CONTROLBOX
        # self.req_io.type = SetIO.Request.TYPE_INSTANT_DO
        # self.req_io.pin = 0
        # self.req_io.state = float(SetIO.Request.STATE_ON)
        # self.future_io = self.set_io_client.call_async(self.req_io)
        # self.future_io.add_done_callback(self.response_callback)

        self.floats = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        #Choose motion type: 1.PTP_J 2.LINE_T
        self.motion = 2

        #if self.unity_client.wait_for_service(timeout_sec=1.0):
        #    self.send_initial_position()

    def unity_callback(self, msg: String):
        # Convert each string to float and then to radians
        data = json.loads(msg.data)
        position_list = np.zeros(6)
        position_list[0] = str(data["x"])
        position_list[1] = str(data["y"])
        position_list[2] = str(data["z"])
        position_list[3] = str(data["roll"])
        position_list[4] = str(data["pitch"])
        position_list[5] = str(data["yaw"])
        
        print(', '.join(str(x) for x in position_list))

        self.get_logger().info(f'Sending posi# Make a call to the servicetions: {self.floats}')
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


        self.get_logger().info(f'Sending posi# Make a call to the servicetions: {self.floats}')
        self.get_logger().info(f'Sending posi# Make a call to the servicetions: {self.floats}')

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
        self.req.velocity = 3.14
        self.req.acc_time = 0.5
        self.req.blend_percentage = 99
        self.req.fine_goal = True
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
    node = unitySubscriberMqttNode()

    rclpy.spin(node)  # Changed to rclpy.spin to handle callbacks

    rclpy.shutdown()