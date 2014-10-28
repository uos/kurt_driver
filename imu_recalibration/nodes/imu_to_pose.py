#!/usr/bin/env python
'''
Created on 18.10.2012

@author: Martin Guenther <mguenthe@uos.de>

This ROS node publishes a pose for IMU messages (for visualization in
RViz). Note that ROS Fuerte+ has a RViz plugin that can directly visualize IMU
messages.
'''

import roslib
roslib.load_manifest('imu_recalibration')

import rospy

from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, Quaternion
import tf

pub = None

def callback(msg_in):
    msg_out = PoseStamped()

    msg_out.header = msg_in.header
    msg_out.pose.position.x = 0
    msg_out.pose.position.y = 0
    msg_out.pose.position.z = 0
    
    (r, p, y) = tf.transformations.euler_from_quaternion(
        [msg_in.orientation.x,
         msg_in.orientation.y,
         msg_in.orientation.z,
         msg_in.orientation.w])

    # mirror orientations
    q = tf.transformations.quaternion_from_euler(-r, -p, -y)    
    msg_out.pose.orientation.x = q[0]
    msg_out.pose.orientation.y = q[1]
    msg_out.pose.orientation.z = q[2]
    msg_out.pose.orientation.w = q[3]

    pub.publish(msg_out)

def main():
    global pub
    rospy.init_node('imu_to_pose', anonymous=True)
    pub = rospy.Publisher('imu_pose', PoseStamped, queue_size=100)
    rospy.Subscriber('imu', Imu, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
