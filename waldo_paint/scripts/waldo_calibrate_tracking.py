#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_msgs.msg
import tf2_geometry_msgs
import tf_conversions.posemath as pm
import PyKDL
import geometry_msgs.msg
import math
import numpy

tf_listener = None
tf_buffer = tf2_ros.Buffer()

def lookup_transform(destination, source):
    source_to_dest_tf = None

    while source_to_dest_tf == None:
        try:
            source_to_dest_tf = tf_buffer.lookup_transform(destination, source, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.sleep(0.1)
            continue

    return tf2_geometry_msgs.transform_to_kdl(source_to_dest_tf)

class TrackingCalibrator:
    def __init__(self):
        while not rospy.is_shutdown():
            rospy.sleep(5)

            tracker_pose = pm.toMatrix(lookup_transform('base', 'canvas'))
            tool_pose = pm.toMatrix(lookup_transform('base', 'tool0'))
            values = numpy.ndarray.flatten(tracker_pose).tolist() + numpy.ndarray.flatten(tool_pose).tolist()
            print(','.join([str(v) for v in values]))

if __name__ == '__main__':
    rospy.init_node('waldo_calibrate_tracking')

    global tf_listener
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    TrackingCalibrator()
    
    rospy.spin()
    openvr.shutdown()

