#!/usr/bin/env python

import sys
import copy
import time
import math
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf2_ros
import tf2_geometry_msgs
import tf_conversions
import PyKDL
from svgpathtools import svg2paths
import pygame
from pygame.locals import*
from numpy import matmul

from std_msgs.msg import String

tf_buffer = tf2_ros.Buffer()

def get_canvas_to_base_transform():
    canvas_to_base_tf = None

    while canvas_to_base_tf == None:
        try:
            canvas_to_base_tf = tf_buffer.lookup_transform('base', 'canvas', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.sleep(0.1)
            continue

    return tf_conversions.posemath.toMatrix(tf2_geometry_msgs.transform_to_kdl(canvas_to_base_tf))

# Get pose oriented toward canvas with initial_pose as origin
def painting_pose(x, y, z):
    # Figure out where the canvas is
    canvas_to_base_4x4 = get_canvas_to_base_transform()

    # Set pose for point to paint

    paint_pose = geometry_msgs.msg.Pose()

    paint_pose.orientation.x = 0#-0.376903
    paint_pose.orientation.y = 0#0.786978
    paint_pose.orientation.z = 0#0.211245
    paint_pose.orientation.w = 1#0.440438

    paint_pose.position.x = x
    paint_pose.position.y = y
    paint_pose.position.z = z

    paint_pose_4x4 = tf_conversions.posemath.toMatrix(tf_conversions.posemath.fromMsg(paint_pose))
    paint_pose_kdl = tf_conversions.posemath.fromMatrix(matmul(paint_pose_4x4, canvas_to_base_4x4))
    paint_pose_kdl.M = PyKDL.Rotation.Quaternion(-0.376903, 0.786978, 0.211245, 0.440438)
    paint_base_pose = tf_conversions.posemath.toMsg(paint_pose_kdl)

    return copy.deepcopy(paint_base_pose)

if __name__ == '__main__':
    rospy.init_node('follow_vive', anonymous=True)

    listener = tf2_ros.TransformListener(tf_buffer)

    try:
        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        group = moveit_commander.MoveGroupCommander('arm')

        while True:
            group.set_pose_target(painting_pose(0, 0, 0))
            print("Going after it!")
            group.go(wait=True)
            rospy.sleep(3)

        moveit_commander.roscpp_shutdown()
    except rospy.ROSInterruptException:
        pass

