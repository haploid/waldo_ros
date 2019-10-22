#!/usr/bin/env python

import rospy
import copy

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point, Pose
from tf.broadcaster import TransformBroadcaster
from tf.transformations import superimposition_matrix

import tf2_ros
import tf2_msgs.msg
import tf2_geometry_msgs
import tf_conversions.posemath as pm
import PyKDL
import geometry_msgs.msg
import openvr
import math
import numpy

def transform_to_pose(transform):
    pose = Pose()
    pose.position.x = transform.p.x()
    pose.position.y = transform.p.y()
    pose.position.z = transform.p.z()
    rx, ry, rz, rw = transform.M.GetQuaternion()
    pose.orientation.x = rx
    pose.orientation.y = ry
    pose.orientation.z = rz
    pose.orientation.w = rw
    return pose

# Calculated via calibration interface
global origin_pose
origin_pose = Pose()
origin_pose.position.x = 0
origin_pose.position.y = 0
origin_pose.position.z = 0
origin_pose.orientation.w = 1
origin_pose.orientation.x = 0
origin_pose.orientation.y = 0
origin_pose.orientation.z = 0

calibration_mat = numpy.matrix([
    [-0.81549539, -0.4580189 ,  0.35381628,  0.94091618],
    [-0.48820883,  0.21605975, -0.84555918,  0.90221903],
    [ 0.31083662, -0.86228584, -0.3998046 ,  0.75132073],
    [ 0.        ,  0.        ,  0.        ,  1.        ]
])

origin_pose = transform_to_pose(pm.fromMatrix(calibration_mat))
"""
origin_pose.position.x = 2.94754496472
origin_pose.position.y = 3.00120429773
origin_pose.position.z = 2.71170948649
origin_pose.orientation.w = -0.555416794601
origin_pose.orientation.x = 0.76727975113
origin_pose.orientation.y = -0.235432775172
origin_pose.orientation.z = 0.217635879753
"""

global tf_publisher
global tf_listener
tf_buffer = tf2_ros.Buffer()

server = None
menu_handler = MenuHandler()

calibration_poses = []

# VIVE

def get_tracker_index():
    for i in range(0, openvr.k_unMaxTrackedDeviceCount):
        try:
            if openvr.IVRSystem().getTrackedDeviceClass(i) == openvr.TrackedDeviceClass_GenericTracker:
                return i
        except openvr.OpenVRError:
            pass

    return None

def get_controller_index(controllerIndex):
    controllerIndices = []

    for i in range(0, openvr.k_unMaxTrackedDeviceCount):
        if openvr.IVRSystem().getTrackedDeviceClass(i) == openvr.TrackedDeviceClass_Controller:
            controllerIndices += [i]

    if controllerIndex >= len(controllerIndices):
        return None
    else:
        return controllerIndices[controllerIndex]

def pose_4x4(in_pose):
    out_pose = numpy.identity(4)
    out_pose[0:3, :] = in_pose[:]
    return out_pose

def lookup_transform(destination, source):
    source_to_dest_tf = None

    while source_to_dest_tf == None:
        try:
            source_to_dest_tf = tf_buffer.lookup_transform(destination, source, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.sleep(0.1)
            continue

    return tf2_geometry_msgs.transform_to_kdl(source_to_dest_tf)

# INTERACTIVE MARKERS

def publish_tf(pose, parent, child):
    t = geometry_msgs.msg.TransformStamped()
    t.header.frame_id = parent
    t.header.stamp = rospy.Time.now()
    t.child_frame_id = child

    if pose == None:
        t.transform.translation.x = 0
        t.transform.translation.y = 0
        t.transform.translation.z = 0

        t.transform.rotation.x = 0
        t.transform.rotation.y = 0
        t.transform.rotation.z = 0
        t.transform.rotation.w = 1
    else:
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z

        t.transform.rotation.x = pose.orientation.x
        t.transform.rotation.y = pose.orientation.y
        t.transform.rotation.z = pose.orientation.z
        t.transform.rotation.w = pose.orientation.w

    tfm = tf2_msgs.msg.TFMessage([t])
    tf_publisher.publish(tfm)

def publish_tracking_origin(origin_pose=None):
    publish_tf(origin_pose, 'base', 'vive_origin')

def add_calibration_frame(tool_pose, tracker_pose):
    global calibration_poses
    calibration_poses.append((tool_pose, tracker_pose))
    print('Added calibration frame:')
    print(tool_pose, tracker_pose)

def publish_calibration_poses():
    for i, poses in enumerate(calibration_poses):
        tool_pose, tracker_pose = poses
        publish_tf(tool_pose, 'base_link', 'calibration_pt_tool_' + str(i))
        publish_tf(tracker_pose, 'vive_origin', 'calibration_pt_tracker_' + str(i))

def matrix_to_pose(mat):
    return transform_to_pose(pm.fromMatrix(mat))

def get_tool_pose():
    return transform_to_pose(lookup_transform('base_link', 'tool0'))

def get_tracker_pose():
    return transform_to_pose(lookup_transform('vive_origin', 'vive_tracker'))

def tf_translation(tf_name, parent):                                            
    t = lookup_transform(parent, tf_name)                                       
    t_mat = pm.toMatrix(t)                                                      
    return tf.transformations.translation_from_matrix(t_mat)

def calibrate_vive_origin():
    pose_pos = lambda p: [p.position.x, p.position.y, p.position.z]

    num_pts = len(calibration_poses)
    tool_vectors = 
    #tool_vectors = [pose_pos(tool_pose) for tool_pose, tracker_pose in calibration_poses]
    #tracker_vectors = [pose_pos(tracker_pose) for tool_pose, tracker_pose in calibration_poses]

    print(tool_vectors, tracker_vectors)
    calibration_matrix = superimposition_matrix(tracker_vectors, tool_vectors, usesvd=False)
    vive_origin_mat = pm.toMatrix(pm.fromMsg(origin_pose))
    new_vive_origin_mat = numpy.matmul(vive_origin_mat, calibration_matrix)
    global origin_pose
    origin_pose = pm.toMsg(pm.fromMatrix(new_vive_origin_mat))
    print('Origin pose calibrated')

def processFeedback(feedback):
    s = "Feedback from marker '" + feedback.marker_name
    s += "' / control '" + feedback.control_name + "'"

    mp = ""
    if feedback.mouse_point_valid:
        mp = " at " + str(feedback.mouse_point.x)
        mp += ", " + str(feedback.mouse_point.y)
        mp += ", " + str(feedback.mouse_point.z)
        mp += " in frame " + feedback.header.frame_id

    if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
        rospy.loginfo( s + ": button click" + mp + "." )
    elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
        rospy.loginfo( s + ": menu item " + str(feedback.menu_entry_id) + " clicked" + mp + "." )

	if feedback.menu_entry_id == 1:
	    add_calibration_frame(get_tool_pose(), get_tracker_pose())
        elif feedback.menu_entry_id == 2:
            calibrate_vive_origin()
    elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        rospy.loginfo( s + ": pose changed")

        global origin_pose
        origin_pose = copy.deepcopy(feedback.pose)
        publish_tracking_origin(origin_pose)
    elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
        rospy.loginfo( s + ": mouse down" + mp + "." )
    elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
        rospy.loginfo( s + ": mouse up" + mp + "." )
    server.applyChanges()

def makeBox( msg, r=0.5, g=0.5, b=0.5 ):
    marker = Marker()

    marker.type = Marker.CUBE
    marker.scale.x = msg.scale * 0.45
    marker.scale.y = msg.scale * 0.45
    marker.scale.z = msg.scale * 0.45
    marker.color.r = r
    marker.color.g = g
    marker.color.b = b
    marker.color.a = 1.0

    return marker

def makeBoxControl( msg ):
    control =  InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append( makeBox(msg) )
    msg.controls.append( control )
    return control

def saveMarker( int_marker ):
  server.insert(int_marker, processFeedback)


#####################################################################
# Marker Creation

def normalizeQuaternion( quaternion_msg ):
    norm = quaternion_msg.x**2 + quaternion_msg.y**2 + quaternion_msg.z**2 + quaternion_msg.w**2
    s = norm**(-0.5)
    quaternion_msg.x *= s
    quaternion_msg.y *= s
    quaternion_msg.z *= s
    quaternion_msg.w *= s

def makeViveOriginMarker():
    fixed = True
    interaction_mode = InteractiveMarkerControl.NONE
    position = Point(0, 0, 0)
    show_6dof = True

    int_marker = InteractiveMarker()
    int_marker.header.frame_id = 'base_link'
    int_marker.pose.position = position
    int_marker.scale = 1

    int_marker.name = 'vive_origin_marker'
    int_marker.description = "Vive tracking origin"

    # Insert a box
    makeBoxControl(int_marker)
    int_marker.controls[0].interaction_mode = interaction_mode

    if interaction_mode != InteractiveMarkerControl.NONE:
        control_modes_dict = { 
                          InteractiveMarkerControl.MOVE_3D : "MOVE_3D",
                          InteractiveMarkerControl.ROTATE_3D : "ROTATE_3D",
                          InteractiveMarkerControl.MOVE_ROTATE_3D : "MOVE_ROTATE_3D" }
        int_marker.name += "_" + control_modes_dict[interaction_mode]
        int_marker.description = "3D Control"
        if show_6dof: 
          int_marker.description += " + 6-DOF controls"
        int_marker.description += "\n" + control_modes_dict[interaction_mode]
    
    if show_6dof: 
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        normalizeQuaternion(control.orientation)
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        normalizeQuaternion(control.orientation)
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        normalizeQuaternion(control.orientation)
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        normalizeQuaternion(control.orientation)
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        normalizeQuaternion(control.orientation)
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        normalizeQuaternion(control.orientation)
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

    server.insert(int_marker, processFeedback)
    menu_handler.apply(server, int_marker.name)

def makeCalibrationMenuMarker():
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = 'base_link'
    int_marker.pose.position = Point(5, 0, 0)
    int_marker.scale = 1

    int_marker.name = 'calibration_menu'
    int_marker.description = 'Calibration Menu'

    control = InteractiveMarkerControl()
    control.interaction_mode = InteractiveMarkerControl.MENU
    control.description = 'Options'
    control.name = 'menu_only_control'
    int_marker.controls.append(copy.deepcopy(control))

    marker = makeBox(int_marker)
    control.markers.append(marker)
    control.always_visible = True
    int_marker.controls.append(control)

    server.insert(int_marker, processFeedback)
    menu_handler.apply(server, int_marker.name)

def broadcast_vive():
    # Vive init

    openvr.init(openvr.VRApplication_Scene)

    print('Vive initialized')

    poses_t = openvr.TrackedDevicePose_t * openvr.k_unMaxTrackedDeviceCount
    poses = poses_t()

    tracker_index = get_tracker_index()

    if tracker_index == None:
        raise Exception('Tracker not found')
    else:
        print('Found Vive Tracker')

    global tf_publisher
    tf_publisher = rospy.Publisher('/tf', tf2_msgs.msg.TFMessage, queue_size=1)

    print('Broadcasting Vive tracker pose...')

    while not rospy.is_shutdown():
        rospy.sleep(0.01)

        publish_tracking_origin(origin_pose)
        publish_calibration_poses()

        openvr.VRCompositor().waitGetPoses(poses, len(poses), None, 0)
        tracker_pose = pose_4x4(poses[tracker_index].mDeviceToAbsoluteTracking)

        tracker_frame = pm.fromMatrix(tracker_pose)
        rx, ry, rz, rw = tracker_frame.M.GetQuaternion()
        #quat = numpy.array([rx, ry, rz, rw])
        #rx, ry, rz, rw = quat / numpy.sqrt(numpy.dot(quat, quat))

        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = 'vive_origin'
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = 'vive_tracker'

        t.transform.translation.x = tracker_frame.p.x() #0.652726
        t.transform.translation.y = tracker_frame.p.y() #0.325163
        t.transform.translation.z = tracker_frame.p.z() #0.789832

        t.transform.rotation.x = rx #-0.24886
        t.transform.rotation.y = ry #0.636402
        t.transform.rotation.z = rz #0.278563
        t.transform.rotation.w = rw #0.67488

        tfm = tf2_msgs.msg.TFMessage([t])
        tf_publisher.publish(tfm)

if __name__=="__main__":
    rospy.init_node('vive_tracking')

    server = InteractiveMarkerServer('basic_controls')
    makeViveOriginMarker()

    menu_handler.insert('Set calibration point', callback=processFeedback)
    menu_handler.insert('Calibrate', callback=processFeedback)
    makeCalibrationMenuMarker()

    server.applyChanges()

    global tf_listener
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    broadcast_vive()
    #tfb = StaticTFBroadcaster(0.778561, 0.274599, 1.1055, 0, 0, 0, 1)
    
    rospy.spin()
    openvr.shutdown()

