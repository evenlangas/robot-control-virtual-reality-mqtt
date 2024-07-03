import numpy as np
import cv2
import sys
import time
import matplotlib.pyplot as plt

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
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11,
}

# Function to display ArUco markers and store their positions
def aruco_display(corners, ids, rejected, image, marker_data):
    if len(corners) > 0:
        ids = ids.flatten()
        for (markerCorner, markerID) in zip(corners, ids):
            # Reshape the marker corner
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners

            # Convert coordinates to integers
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            # Display lines
            cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)

            # Store and display information
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)

            if markerID not in marker_data:
                marker_data[markerID] = {"x": [], "y": []}
            marker_data[markerID]["x"].append(cX)
            marker_data[markerID]["y"].append(cY)

            cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
            cv2.putText(image, str(markerID), (topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            print(f"[Inference] ArUco marker ID: {markerID} Position: ({cX}, {cY})")

    return image

# Function for pose estimation and updating marker data
def pose_estimation(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients, marker_data):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    arucoDict = cv2.aruco.Dictionary(aruco_dict_type)
    arucoParams = cv2.aruco.DetectorParameters_create()

    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, arucoDict, parameters=arucoParams,
        cameraMatrix=matrix_coefficients, distCoeff=distortion_coefficients)

    if len(corners) > 0:
        for i in range(len(ids)):
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, matrix_coefficients, distortion_coefficients)
            cv2.aruco.drawDetectedMarkers(frame, corners)
            cv2.aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)

    return frame

# Initializations
aruco_type = "DICT_4X4_100"
arucoDict = cv2.aruco.Dictionary(ARUCO_DICT[aruco_type])
arucoParams = cv2.aruco.DetectorParameters_create()

intrinsic_camera = np.array(((933.15867, 0, 657.59), (0, 933.1586, 400.36993), (0, 0, 1)))
distortion = np.array((-0.43948, 0.18514, 0, 0))

cap = cv2.VideoCapture(4)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 848)
cap.set(cv2.CAP_PROP_FPS, 60)

frame_count = 0
start_time = time.time()

# Marker data dictionary
marker_data = {}

while cap.isOpened():
    ret, img = cap.read()
    if not ret:
        break

    corners, ids, rejected = cv2.aruco.detectMarkers(img, arucoDict, parameters=arucoParams)
    detected_markers = aruco_display(corners, ids, rejected, img, marker_data)
    output = pose_estimation(img, ARUCO_DICT[aruco_type], intrinsic_camera, distortion, marker_data)

    cv2.imshow('Estimated Pose', output)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

# Plotting the dataa
plt.figure(figsize=(12, 6))

for markerID, data in marker_data.items():
    #plt.plot(range(len(data["x"])), data["x"], label=f'Marker {markerID}')
    #plt.scatter(range(len(data["x"])), data["x"], label=f'Marker {markerID}')
    plt.scatter(data["x"], data["y"], alpha=0.5, label=f'Marker {markerID}')

plt.title("X-Positions of ArUco Markers")
plt.xlabel("Frame")
plt.ylabel("X Position")
plt.legend()
plt.show()