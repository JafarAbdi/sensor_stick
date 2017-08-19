#!/usr/bin/env python

import rospy
import tf
import tf2_ros
from std_msgs.msg import Float64
from geometry_msgs.msg import TransformStamped, Transform
import numpy as np


def message_from_transform(T):
    msg = Transform()
    q = tf.transformations.quaternion_from_matrix(T)
    p = tf.transformations.translation_from_matrix(T)
    msg.translation.x = p[0]
    msg.translation.y = p[1]
    msg.translation.z = p[2]
    msg.rotation.x = q[0]
    msg.rotation.y = q[1]
    msg.rotation.z = q[2]
    msg.rotation.w = q[3]
    return msg

rospy.init_node('cloud_transformer')
tf_broadcaster = tf2_ros.TransformBroadcaster()


global angle
init_angle = -30.0

angle = init_angle

def angle_cb(msg):
    global angle
    angle = -(msg.data)*np.pi/180.0
    #rospy.loginfo(10)

#rospy.Subscriber('/cur_tilt_angle', Float64, angle_cb)
rospy.Subscriber('/tilt_angle', Float64, angle_cb)

angle_pub = rospy.Publisher('/tilt_angle', Float64, queue_size=0)

reset_angle = Float64(init_angle)
rospy.sleep(1.0)
angle_pub.publish(reset_angle)


rate = rospy.Rate(100)
while not rospy.is_shutdown():
    transform = TransformStamped()
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = "world"
    transform.child_frame_id = "camera_link"

    trans = tf.transformations.translation_matrix([0, 0, 0.78])
    rot = tf.transformations.euler_matrix(0, angle, 0,'rxyz')
    transform.transform = message_from_transform(np.dot(trans, rot))
    tf_broadcaster.sendTransform(transform)

    rate.sleep()
