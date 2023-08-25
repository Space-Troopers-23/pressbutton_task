#!/usr/bin/env python3
from __future__ import print_function
from variables import Aruco
import cv2
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import math
try:
    from math import pi, fabs, cos, sqrt
except:
    from math import pi, fabs, cos, sqrt

from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

def all_close(goal, actual, tolerance):
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False
    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)
    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        d = dist((x1, y1, z1), (x0, y0, z0))
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)
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
def main_position():
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    joint_goal = move_group.get_current_joint_values()
    
    joint_goal[0] = 0
    joint_goal[1] = -2.09
    joint_goal[2] = 1.745
    joint_goal[3] = 0.349
    joint_goal[4] = 1.5707
    joint_goal[5] = -1.5707 
    
    move_group.go(joint_goal, wait=True)
    move_group.stop()

def image_callback(msg):
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    h, w, _ = img.shape
    
    width = 680
    height = int(width * (h / w))
    img = cv2.resize(img, (width, height), interpolation=cv2.INTER_CUBIC)
    target_cx = width // 2
    
    for aruco_name in ARUCO_DICT.keys():
        arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[aruco_name])
        arucoParams = cv2.aruco.DetectorParameters_create()
        
        corners, ids, _ = cv2.aruco.detectMarkers(img, arucoDict, parameters=arucoParams)

        # Get ArUco IDs
        if ids is not None:
            tags_param = rospy.get_param("~tags", "").split()
            tags_param = [int(tag) for tag in tags_param if tag.isdigit()]

            for markerID in ids.flatten():
                if markerID in tags_param:
                    aruco_display(corners, markerID, img, target_cx)

    # Display the image
    # cv2.imshow("Image", img)
    # cv2.waitKey(1)

def aruco_display(corners, markerID, image, target_cx, tolerance=25): 
    if len(corners) > 0:
        ids = ids.flatten()

        for (markerCorner, markerID) in zip(corners, ids):
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners

            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)

            cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
            cv2.putText(image, str(markerID), (int(topLeft[0]), int(topLeft[1] - 10)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 0), 2)
            
            move_robot_to_target(cX, target_cx, tolerance)

def do_something_for_marker(markerID):
    if markerID == 1:
        # Movement definition and control only for ID 1 is done here
        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        joint_goal = move_group.get_current_joint_values()
        while abs(cx - 340) > 20:
            if cx < 340:
                joint_goal[0] += 0.012
            else:
                joint_goal[0] -= 0.012
        # Robot movement is performed here with the 'joint_goal' list
        move_group.go(joint_goal, wait=True)
        move_group.stop()


if __name__ == "__main__":
    # Connect to the rviz robot arm
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_publisher = rospy.Publisher(
        "/move_group/display_planned_path",
        moveit_msgs.msg.DisplayTrajectory,
        queue_size=20,
    )

    planning_frame = move_group.get_planning_frame()
    eef_link = move_group.get_end_effector_link()

    group_names = robot.get_group_names()

    print(robot.get_current_state())
    image_sub = rospy.Subscriber("camera_image/image_raw", Image, image_callback)

    rospy.spin()
    cv2.destroyAllWindows()

