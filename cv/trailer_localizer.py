# use the localizer camera to detect the aruco marker on the trailer

# use the localizer position to get the pose of the robot

# work out the world angle of the trailer and the world angle of the robot

# compute the angle of the trailer relative to the robot reference frame, and return as the hitch_angle


import pibot_client
import cv2, math


def create_bw_mask(image):
    # convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # apply a threshold to create a binary image
    _, bw_mask = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

    # apply a morphological operation to remove small noise
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    bw_mask = cv2.morphologyEx(bw_mask, cv2.MORPH_OPEN, kernel)
    return bw_mask


def detect_aruco_marker(image):
    # use opencv to detect the aruco marker in the image
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image, aruco_dict, parameters=parameters)
    return corners, ids


def compute_trailer_word_pose(aruco_corners, localizer_pose):
    # compute the world pose of the trailer based on the aruco marker corners and the localizer pose
    # this will involve some trigonometry to compute the angle of the trailer relative to the robot reference frame
    raise NotImplementedError("This function needs to be implemented")


def compute_hitch_angle(trailer_world_pose, robot_world_pose):
    # compute the angle of the trailer relative to the robot reference frame, and return as the hitch_angle
    raise NotImplementedError("This function needs to be implemented")



# optional
def compute_trailer_robot_pose(robot_world_pose, hitch_angle):
    # compute the pose of the trailer in the robot reference frame, based on the robot world pose and the hitch angle
    raise NotImplementedError("This function needs to be implemented")



# TESTING CODE

# plot the example image saved in this folder as LocaliserFiducialTest1.png
image = cv2.imread("LocaliserFiducialTest1.png")
cv2.imshow("Original Image", image)

# test the noise reduction
bw_mask = create_bw_mask(image)
cv2.imshow("Binary Mask", bw_mask)

# test the aruco marker detection
corners, ids = detect_aruco_marker(image)
print("Corners:", corners)
print("IDs:", ids)

cv2.imshow("Aruco Markers", image)
cv2.waitKey(0)
cv2.destroyAllWindows()

# test the trailer world pose computation
localizer_pose = (1, 1, 3, 0, math.pi, 0)  # example localizer pose (x, y, z, theta_x, theta_y, theta_z) in meters and radians (the localiser lives in 3D space, above the 2D plane of the robot, with the camera looking directly down at it)
trailer_world_pose = compute_trailer_word_pose(corners, localizer_pose)
print("Trailer World Pose:", trailer_world_pose)

# test the hitch angle computation
robot_world_pose = (1, 1, 0)  # example robot world pose (x, y, theta) in meters and radians (the robot lives in 2D space)
hitch_angle = compute_hitch_angle(trailer_world_pose, robot_world_pose)
print("Hitch Angle:", hitch_angle)
