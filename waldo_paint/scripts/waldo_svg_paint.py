#!/usr/bin/env python

import sys
import copy
import math
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf_conversions
from svgpathtools import svg2paths
import pygame
from pygame.locals import*

from std_msgs.msg import String

def scale_pt(pt):
    x, y = pt
    return (x / 1000.0, y / 1000.0)

# Get pose oriented toward canvas with initial_pose as origin
def painting_pose(initial_pose, x, y, z):
    wpose = geometry_msgs.msg.Pose()

    #ten_deg = 10.0 / 360.0 * 2.0 * math.pi
    #roll = 0
    #pitch = math.pi / 2
    #yaw = -math.pi / 2
    #wpose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(roll, pitch, yaw))

    wpose.orientation.x = initial_pose.orientation.x
    wpose.orientation.y = initial_pose.orientation.y
    wpose.orientation.z = initial_pose.orientation.z
    wpose.orientation.w = initial_pose.orientation.w

    wpose.position.x = initial_pose.position.x + x
    wpose.position.y = initial_pose.position.y + y
    wpose.position.z = initial_pose.position.z + z

    return copy.deepcopy(wpose)

def follow_waypoints(group, waypoints):
    # Do it!
    eef_step = 0.05 # Resolution of path in meters
    jump_threshold = 0 # Disable jumps
    painting_plan, fraction = group.compute_cartesian_path(waypoints, eef_step, jump_threshold)
    group.execute(painting_plan, wait=True)

def dip_brush(initial_pose, group):
    waypoints = []

    start_pose = group.get_current_pose().pose
    
    # Move away from canvas
    waypoints.append(painting_pose(start_pose, 0, 0, 0.2))

    # Move over paint 
    waypoints.append(painting_pose(initial_pose, -0.1, -0.1, 0.2))

    # Move into paint 
    waypoints.append(painting_pose(initial_pose, -0.1, -0.1, 0))

    # Move over paint 
    waypoints.append(painting_pose(initial_pose, -0.1, -0.1, 0.2))

    # Move back to where we were
    waypoints.append(painting_pose(start_pose, 0, 0, 0.2))

    follow_waypoints(group, waypoints)

def paint_path(initial_pose, group, path):
    offset = 0.01
    waypoints = []

    # Start at current pose
    current_pose = group.get_current_pose().pose
    waypoints.append(current_pose)

    def move_to(x, y, z):
        waypoints.append(painting_pose(initial_pose, x, y, z))

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

    initial_pose = group.get_current_pose().pose

    for path in paths:
        dip_brush(initial_pose, group)
        rospy.sleep(0.5)
        paint_path(initial_pose, group, path)
        rospy.sleep(0.5)

    dip_brush(initial_pose, group)
    rospy.sleep(0.5)

    group.set_pose_target(initial_pose)
    group.go(wait=True)

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

def paint_svg(svg_filename):
  rospy.init_node('waldo_svg_painter', anonymous=True)
  paths = path_points_from_svg(svg_filename)
  paint_paths(paths)

if __name__ == '__main__':
  try:
    paint_svg('/path/to/some.svg')
  except rospy.ROSInterruptException:
    pass

