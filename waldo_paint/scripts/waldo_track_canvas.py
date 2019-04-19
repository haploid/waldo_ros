#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg

class FixedTFBroadcaster:
    def __init__(self):
        self.pub_tf = rospy.Publisher('/tf', tf2_msgs.msg.TFMessage, queue_size=1)

        while not rospy.is_shutdown():
            # Loop at about 10 Hz
            rospy.sleep(0.1)

            t = geometry_msgs.msg.TransformStamped()
            t.header.frame_id = 'world'
            t.header.stamp = rospy.Time.now()
            t.child_frame_id = 'canvas'

            t.transform.translation.x = 0.652726
            t.transform.translation.y = 0.325163
            t.transform.translation.z = 0.789832

            t.transform.rotation.x = -0.24886
            t.transform.rotation.y = 0.636402
            t.transform.rotation.z = 0.278563
            t.transform.rotation.w = 0.67488

            tfm = tf2_msgs.msg.TFMessage([t])
            self.pub_tf.publish(tfm)

if __name__ == '__main__':
    rospy.init_node('fixed_tf2_broadcaster')
    tfb = FixedTFBroadcaster()
    
    rospy.spin()

