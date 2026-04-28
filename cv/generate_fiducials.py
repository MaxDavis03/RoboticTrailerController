import cv2
import cv2.aruco as aruco

# Choose dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

# Create markers
for marker_id in [0, 1]:
    marker_img = aruco.generateImageMarker(aruco_dict, marker_id, 500)

    cv2.imwrite(f"marker_{marker_id}.png", marker_img)