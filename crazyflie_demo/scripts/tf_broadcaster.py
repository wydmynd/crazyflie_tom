#!/usr/bin/env python

import rospy
from crazyflie_driver.msg import GenericLogData

import geometry_msgs.msg
import tf_conversions
import tf2_ros

from math import radians

global x, y, z, roll, pitch, yaw
x=0
y=0
z=0
roll=0
pitch=0
yaw=0


def get_pose(msg):
    global x,y,z
    x=msg.values[0]
    y = msg.values[1]
    z = msg.values[2]

def get_rpy(msg):
    global roll,pitch,yaw
    roll=msg.values[0]
    pitch = msg.values[1]
    yaw = msg.values[2]

def handle_pose():
    global x,y,z,roll,pitch,yaw
    rospy.init_node('tf_broadcaster')
    rospy.Subscriber('/cf1/log_pos' , GenericLogData, get_pose)
    rospy.Subscriber('/cf1/log_rpy' , GenericLogData, get_rpy)
    r=rospy.Rate(40)
    while not rospy.is_shutdown():
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.child_frame_id = "cf1"
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        q = tf_conversions.transformations.quaternion_from_euler(radians(roll),radians(pitch),radians(yaw))
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        br.sendTransform(t)
        #print "x=",x
        r.sleep()


if __name__ == '__main__':
    handle_pose()




