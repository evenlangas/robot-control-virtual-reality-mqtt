import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import cv2
import numpy as np

ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

class ArUcoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_test')
        self.publisher_ = self.create_publisher(Point, '/ArUco_Detect', 10)
        self.timer = self.create_timer(0.1, self.detect_and_publish)
        self.cap = cv2.VideoCapture(4)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT["DICT_4X4_100"])
        self.arucoParams = cv2.aruco.DetectorParameters_create()

    def detect_and_publish(self):
        ret, img = self.cap.read()
        if ret:
            corners, ids, rejected = cv2.aruco.detectMarkers(img, self.arucoDict, parameters=self.arucoParams)
            if len(corners) > 0:
                # Assuming you're interested in the first detected marker for simplicity
                markerCorner = corners[0]
                corners = markerCorner.reshape((4, 2))
                topLeft, topRight, bottomRight, bottomLeft = corners
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                point_msg = Point(x=float(cX), y=float(cY), z=0.0)
                self.publisher_.publish(point_msg)
                self.get_logger().info(f'Publishing: {point_msg}')

def main(args=None):
    rclpy.init(args=args)
    aruco_detector_node = ArUcoDetectorNode()
    try:
        rclpy.spin(aruco_detector_node)
    except KeyboardInterrupt:
        pass
    finally:
        aruco_detector_node.destroy_node()
        rclpy.shutdown()