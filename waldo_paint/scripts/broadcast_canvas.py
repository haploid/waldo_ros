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

class StaticTFBroadcaster:
    def __init__(self, x, y, z, rx, ry, rz, rw):
        self.pub_tf = rospy.Publisher('/tf', tf2_msgs.msg.TFMessage, queue_size=1)

        while not rospy.is_shutdown():
            # Loop at about 10 Hz
            rospy.sleep(0.1)

            t = geometry_msgs.msg.TransformStamped()
            t.header.frame_id = 'vive_tracker'
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
    rospy.init_node('broadcast_canvas')

    global tf_listener
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    tfb = StaticTFBroadcaster(0, 0, 0.2, 0, 0, 0, 1)
    
    rospy.spin()
    openvr.shutdown()

