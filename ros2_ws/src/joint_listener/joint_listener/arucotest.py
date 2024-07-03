import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import String
import cv2
from cv2 import aruco
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
        super().__init__('aruco_detector')
        self.publisher_ = self.create_publisher(String, '/ArUco', 10)
        self.timer = self.create_timer(0.1, self.detect_and_publish)
        self.cap = cv2.VideoCapture(4)  # Adjust the device number as necessary
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 848)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 60)

        self.arucoDict = aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
        self.arucoParams = aruco.DetectorParameters()
        self.arucoDetector = aruco.ArucoDetector(self.arucoDict, self.arucoParams)

        self.camera_matrix = np.array([[919.203735351562, 0, 647.1005859375],
                                       [0, 919.134643554688, 371.137908935547],
                                       [0, 0, 1]], dtype="double")

        self.dist_coeffs = np.array([0, 0, 0, 0, 0], dtype="double")

    def detect_and_publish(self):
        ret, frame = self.cap.read()
        if ret:
            corners, ids, rejectedImgPoints = self.arucoDetector.detectMarkers(frame)
            if ids is not None:
                max_x = None
                best_marker = None

                for i, corner in enumerate(corners):
                    center_x = int((corner[0][0][0] + corner[0][1][0] + corner[0][2][0] + corner[0][3][0]) / 4)
                    center_y = int((corner[0][0][1] + corner[0][1][1] + corner[0][2][1] + corner[0][3][1]) / 4)

                    vec_x = corner[0][1][0] - corner[0][0][0]  # x2 - x1
                    vec_y = corner[0][1][1] - corner[0][0][1]  # y2 - y1
                    angle_rad = math.atan2(vec_y, vec_x)
                    angle_deg = math.degrees(angle_rad)

                    if max_x is None or center_x > max_x:
                        max_x = center_x
                        best_marker = (center_x, center_y, angle_deg)

                if best_marker:
                    data = f"{best_marker[0]}#{best_marker[1]}#{best_marker[2]:.2f}#c"
                    self.publisher_.publish(String(data=data))
                    self.get_logger().info(f'Publishing Highest X Marker Center and Orientation: (x={best_marker[0]}, y={best_marker[1]}, angle={best_marker[2]:.2f} degrees)')

def main(args=None):
    rclpy.init(args=args)
    aruco_detector_node = ArUcoDetectorNode()
    rclpy.spin(aruco_detector_node)
    aruco_detector_node.destroy_node()
    rclpy.shutdown()