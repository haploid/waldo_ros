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
from geometry_msgs.msg import Pose
import tf

# Find the calibration marks
# Fail if less than three are set
# Do: vive_origin_mat = tf.transformations.superimposition_matrix(vive_mats, robot_mats)
# Print Vive origin matrix
# Set Vive origin transformation

global tf_listener
global tf_publisher
tf_listener = None
tf_publisher = None
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

def matrix_to_pose(mat):
    return transform_to_pose(pm.fromMatrix(mat))

def tf_translation(tf_name, parent):
    t = lookup_transform(parent, tf_name)
    t_mat = pm.toMatrix(t)
    return tf.transformations.translation_from_matrix(t_mat)

if __name__ == '__main__':
    rospy.init_node('vive_auto_calibration')

    tf_listener = tf2_ros.TransformListener(tf_buffer)
    tf_publisher = rospy.Publisher('/tf', tf2_msgs.msg.TFMessage, queue_size=1)

    num_pts = 4

    robot_vectors = [tf_translation('calibration_pt_tool_{}'.format(i), 'base_link') for i in range(0, num_pts)]
    vive_vectors = [tf_translation('calibration_pt_tracker_{}'.format(i), 'vive_origin') for i in range(0, num_pts)]

    print('Vive vectors')
    for pt in vive_vectors:
        print(pt)

    print('Robot vectors')
    for pt in robot_vectors:
        print(pt)

    vive_origin_mat = tf.transformations.superimposition_matrix(vive_vectors, robot_vectors)

    print(repr(vive_origin_mat))

    #vive_origin_pose = matrix_to_pose(vive_origin_mat)
    #publish_tf(vive_origin_pose, 'base', 'vive_origin')

    #print('Published vive_origin:', )
    x, y, z = tf.transformations.translation_from_matrix(vive_origin_mat)
    rw, rx, ry, rz = tf.transformations.quaternion_from_matrix(vive_origin_mat)
    print('origin_pose.position.x = {}'.format(x))
    print('origin_pose.position.y = {}'.format(y))
    print('origin_pose.position.z = {}'.format(z))
    print('origin_pose.orientation.w = {}'.format(rw))
    print('origin_pose.orientation.x = {}'.format(rx))
    print('origin_pose.orientation.y = {}'.format(ry))
    print('origin_pose.orientation.z = {}'.format(rz))

