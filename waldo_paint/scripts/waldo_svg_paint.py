#!/usr/bin/env python

import sys
import copy
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

def scale_pt(pt):
    x, y = pt
    return (x / 1000.0, y / 1000.0)

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

def follow_waypoints(group, waypoints):
    # Do it!
    eef_step = 0.1 # Resolution of path in meters
    jump_threshold = 0 # Disable jumps

    found_trajectory = False
    for i in range(0, 3):
        painting_plan, fraction_complete = group.compute_cartesian_path(waypoints, eef_step, jump_threshold)

        if fraction_complete > 0.99:
            found_trajectory = True
            break

    if not found_trajectory:
        raise Exception('Trajectory not found')

    print('Able to follow {}% of trajectory'.format(fraction_complete * 100))

    group.execute(painting_plan, wait=True)

paint_offset_x = 0
paint_offset_y = 0
lift_for_paint = 0.1
def dip_brush(group):
    waypoints = []

    # Move away from canvas
    waypoints.append(painting_pose(0, 0, lift_for_paint))

    # Move over paint 
    waypoints.append(painting_pose(paint_offset_x, paint_offset_y, lift_for_paint))

    # Move into paint 
    waypoints.append(painting_pose(paint_offset_x, paint_offset_y, 0))

    # Move over paint 
    waypoints.append(painting_pose(paint_offset_x, paint_offset_y, lift_for_paint))

    # Move back to where we were
    waypoints.append(painting_pose(0, 0, lift_for_paint))

    follow_waypoints(group, waypoints)

def paint_path(group, path):
    offset = 0.01
    waypoints = []

    # Start at current pose
    current_pose = group.get_current_pose().pose
    #waypoints.append(current_pose)

    # Move away from canvas
    #waypoints.append(painting_pose(current_pose, 0, 0, offset))

    def move_to(x, y, z):
        waypoints.append(painting_pose(x, y, z))

    # Move to above start
    start_x, start_y = scale_pt(path[0])
    move_to(start_x, start_y, offset)

    # To canvas
    move_to(start_x, start_y, 0)

    for x, y in map(scale_pt, path):
        move_to(x, y, 0)

    # Away from canvas
    last_x, last_y = scale_pt(path[-1])
    move_to(last_x, last_y, offset)

    follow_waypoints(group, waypoints)

def paint_paths(paths):
    moveit_commander.roscpp_initialize(sys.argv)

    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander('arm')

    for path in paths:
        dip_brush(group)
        rospy.sleep(0.5)
        paint_path(group, path)
        rospy.sleep(0.5)

    dip_brush(initial_pose, group)
    rospy.sleep(0.5)

    #group.set_pose_target(initial_pose)
    #group.go(wait=True)

    moveit_commander.roscpp_shutdown()

def plot_paths(paths):
    moveit_commander.roscpp_initialize(sys.argv)

    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander('arm')

    for path in paths:
        paint_path(group, path)
        #rospy.sleep(0.1)

    #group.set_pose_target(initial_pose)
    #group.go(wait=True)

    moveit_commander.roscpp_shutdown()

"""
def render_paths(paths):
    screen = pygame.display.set_mode((2000, 2000))
    screen.fill((255, 255, 255))

    print('Drawing {} paths'.format(len(paths)))

    for path in paths:
        last_x = None
        last_y = None

        for x, y in path:
            if not last_x == None:
                pygame.draw.line(screen, (0, 0, 0), (last_x, last_y), (x, y))

            last_x = x
            last_y = y

    pygame.display.flip()

    while True:
        for events in pygame.event.get():
            if events.type == QUIT:
                sys.exit(0)
"""

def path_points_from_svg(svg_filename):
    svg_paths, attributes = svg2paths(svg_filename)

    SAMPLES_PER_PX = 0.05

    # List of lists of points
    paths = []
    for path, attr in zip(svg_paths, attributes):
        path_length = path.length()
        num_samples = int(path_length * SAMPLES_PER_PX)

        # New list to hold points for path
        current_path = []

        if num_samples <= 1:
            continue

        for i in range(num_samples):
            position_on_path = i / float(num_samples - 1)
            pt = path.point(position_on_path)

            if pt == None:
                break

            x = pt.real
            y = pt.imag
            current_path.append((x, y))

        # Only add paths that contain points
        if len(current_path) > 0:
            paths.append(current_path)
    
    return paths

def paint_circle():
    path = []

    r = 200.0
    offset_x = 200.0
    offset_y = 200.0 

    count = 360
    for i in range(0, count):
        angle = (float(i) / count) * math.pi * 2.0
        x = offset_x + r * math.cos(angle)
        y = offset_y + r * math.sin(angle)
        path.append((x, y))

    paint_paths([path])

def paint_svg(svg_filename):
  paths = path_points_from_svg(svg_filename)
  #paint_paths(paths)
  plot_paths(paths)

if __name__ == '__main__':
  rospy.init_node('waldo_svg_painter', anonymous=True)

  listener = tf2_ros.TransformListener(tf_buffer)

  try:
    paint_svg('/home/owen/Downloads/export1.svg')
    #paint_circle()
  except rospy.ROSInterruptException:
    pass

