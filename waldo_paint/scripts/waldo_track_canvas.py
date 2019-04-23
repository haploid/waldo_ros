#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_msgs.msg
import tf2_geometry_msgs
import tf_conversions.posemath as pm
import PyKDL
import geometry_msgs.msg
import openvr
import math
import numpy

tf_listener = None
tf_buffer = tf2_ros.Buffer()

vive_to_robot = numpy.matrix([
    [ 0.05912276,  0.24772471,  0.07612668,  0.14178994],
    [ 0.52984906, -0.00857709,  0.22512834,  1.2935338 ],
    [ 0.28870523,  0.17252041,  0.17329422,  1.24402038],
    [ 0.        ,  0.        ,  0.        ,  1.        ]])

#calibration_tf = PyKDL.Frame(PyKDL.Rotation.Quaternion(-0.583131055195, -0.413052317293, 0.426094875175, 0.554787413626), PyKDL.Vector(-0.591979438504, 1.96640354624, 1.50396268791))

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

def pose_to_xyz(pose):
    x = pose[0][3]
    y = pose[1][3]
    z = pose[2][3]

    return (x, y, z)

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

def get_canvas_to_tool_twist():
    canvas_tf = lookup_transform('/world', '/canvas')
    tool_tf = lookup_transform('tool0', '/canvas')

    return PyKDL.diff(canvas_tf, tool_tf, 1)

def print_calibration():
    """
    canvas_tf = lookup_transform('world', 'canvas')
    tool_tf = lookup_transform('world', 'tool0')

    canvas_rx, canvas_ry, canvas_rz, canvas_rw = canvas_tf.M.GetQuaternion()
    tool_rx, tool_ry, tool_rz, tool_rw = tool_tf.M.GetQuaternion()

    print('calib_canvas_tf = PyKDL.Frame(PyKDL.Rotation.Quaternion({}, {}, {}, {}), PyKDL.Vector({}, {}, {}))'.format(canvas_rx, canvas_ry, canvas_rz, canvas_rw, canvas_tf.p.x(), canvas_tf.p.y(), canvas_tf.p.z()))
    print('calib_tool_tf = PyKDL.Frame(PyKDL.Rotation.Quaternion({}, {}, {}, {}), PyKDL.Vector({}, {}, {}))'.format(tool_rx, tool_ry, tool_rz, tool_rw, tool_tf.p.x(), tool_tf.p.y(), tool_tf.p.z()))
    """
    canvas_to_tool = lookup_transform('tool0', 'canvas').Inverse()
    calib_rx, calib_ry, calib_rz, calib_rw = canvas_to_tool.M.GetQuaternion()
    print('calibration_tf = PyKDL.Frame(PyKDL.Rotation.Quaternion({}, {}, {}, {}), PyKDL.Vector({}, {}, {}))'.format(calib_rx, calib_ry, calib_rz, calib_rw, canvas_to_tool.p.x(), canvas_to_tool.p.y(), canvas_to_tool.p.z()))

class ViveTFBroadcaster:
    def __init__(self):
	# Vive init

	openvr.init(openvr.VRApplication_Scene)

	poses_t = openvr.TrackedDevicePose_t * openvr.k_unMaxTrackedDeviceCount
	poses = poses_t()

	tracker_index = get_tracker_index()

	if tracker_index == None:
	    raise Exception('Tracker not found')

        self.pub_tf = rospy.Publisher('/tf', tf2_msgs.msg.TFMessage, queue_size=1)

        print('Broadcasting Vive tracker pose...')

        while not rospy.is_shutdown():
            rospy.sleep(0.01)

	    openvr.VRCompositor().waitGetPoses(poses, len(poses), None, 0)
	    tracker_pose = pose_4x4(poses[tracker_index].mDeviceToAbsoluteTracking) * vive_to_robot

            tracker_frame = pm.fromMatrix(tracker_pose)
            rx, ry, rz, rw = tracker_frame.M.GetQuaternion()
            quat = numpy.array([rx, ry, rz, rw])
            rx, ry, rz, rw = quat / numpy.sqrt(numpy.dot(quat, quat))

            t = geometry_msgs.msg.TransformStamped()
            t.header.frame_id = 'world'
            t.header.stamp = rospy.Time.now()
            t.child_frame_id = 'canvas'

            """
            t.transform.translation.x = -tracker_frame.p.x() #0.652726
            t.transform.translation.y = tracker_frame.p.z() #0.325163
            t.transform.translation.z = tracker_frame.p.y() #0.789832
            """

            t.transform.translation.x = tracker_frame.p.x() #0.652726
            t.transform.translation.y = tracker_frame.p.y() #0.325163
            t.transform.translation.z = tracker_frame.p.z() #0.789832

            t.transform.rotation.x = rx #-0.24886
            t.transform.rotation.y = ry #0.636402
            t.transform.rotation.z = rz #0.278563
            t.transform.rotation.w = rw #0.67488

            tfm = tf2_msgs.msg.TFMessage([t])
            self.pub_tf.publish(tfm)

            #print_calibration()

class StaticTFBroadcaster:
    def __init__(self, x, y, z, rx, ry, rz, rw):
        self.pub_tf = rospy.Publisher('/tf', tf2_msgs.msg.TFMessage, queue_size=1)

        print('Broadcasting static pose...')

        while not rospy.is_shutdown():
            # Loop at about 10 Hz
            rospy.sleep(0.1)

            t = geometry_msgs.msg.TransformStamped()
            t.header.frame_id = 'world'
            t.header.stamp = rospy.Time.now()
            t.child_frame_id = 'canvas'

            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = z

            t.transform.rotation.x = rx
            t.transform.rotation.y = ry
            t.transform.rotation.z = rz
            t.transform.rotation.w = rw

            tfm = tf2_msgs.msg.TFMessage([t])
            self.pub_tf.publish(tfm)

if __name__ == '__main__':
    rospy.init_node('waldo_track_canvas')

    global tf_listener
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    tfb = ViveTFBroadcaster()
    #tfb = StaticTFBroadcaster(0.778561, 0.274599, 1.1055, 0, 0, 0, 1)
    
    rospy.spin()
    openvr.shutdown()

