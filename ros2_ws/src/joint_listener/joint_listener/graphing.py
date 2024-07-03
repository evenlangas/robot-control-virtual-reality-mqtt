#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import datetime
import csv

class GraphNode(Node):
    def __init__(self):
        super().__init__("graph_node")
        self.list_subscriber = self.create_subscription(String, "/input_delay_back", self.list_callback, 10)
        self.time_publisher = self.create_publisher(String, "/input_delay", 10)
        
        self.data = []
        self.timestamp = None  # Variable to store the timestamp
        self.timestamp2 = None
        
        self.csv_file = open('time_differences.csv', 'w', newline='')  # Open the CSV file in write mode
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Time Difference (milliseconds)'])  # Write the header row
        
        # Create a timer to publish the #t string at regular intervals
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    def timer_callback(self):
        # Record the current time in Norwegian time using datetime
        norwegian_time = datetime.timezone(datetime.timedelta(hours=2))  # Norwegian time zone (UTC+2)
        self.timestamp = datetime.datetime.now(norwegian_time)
        
        # Create and publish the string message
        msg_to_publish = String()
        msg_to_publish.data = "#t#" + self.timestamp.strftime("%H.%M.%S.%f")
        self.time_publisher.publish(msg_to_publish)
    
    def list_callback(self, msg: String):
        norwegian_time = datetime.timezone(datetime.timedelta(hours=2))  # Norwegian time zone (UTC+2)
        self.timestamp2 = datetime.datetime.now(norwegian_time)

        if self.timestamp is not None:
            # Extract the time from the message
            parts = msg.data.split('#')
            if len(parts) >= 3:
                time_str = parts[2]
                # Parse the extracted time string to a datetime object
                try:
                    received_time = datetime.datetime.strptime(time_str, "%H.%M.%S.%f").replace(tzinfo=norwegian_time)
                    
                    # Adjust the date part of received_time to match the date part of self.timestamp
                    received_time = received_time.replace(year=self.timestamp.year, month=self.timestamp.month, day=self.timestamp.day)
                    
                    # Calculate the difference between the recorded timestamp and the received timestamp
                    time_diff = (self.timestamp2 - received_time) / 2  # Correctly calculate the average delay
                    time_diff_ms = time_diff.total_seconds() * 1000  # Convert to milliseconds
                    
                    # Append the difference to the data list
                    self.data.append(time_diff_ms)
                    self.csv_writer.writerow([time_diff_ms])  # Write the time difference to the CSV file
                    self.get_logger().info(f"Time difference calculated: {time_diff_ms} milliseconds (self.timestamp: {self.timestamp}, received_time: {received_time})")
                    
                    # Exit if 10,000 data points are reached
                    if len(self.data) >= 10000:
                        self.get_logger().info("Reached 10,000 data points. Shutting down.")
                        rclpy.shutdown()
                except ValueError as e:
                    self.get_logger().error(f"Error parsing time: {e}")
    
    def destroy_node(self):
        self.csv_file.close()  # Close the CSV file when the node is destroyed
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GraphNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()