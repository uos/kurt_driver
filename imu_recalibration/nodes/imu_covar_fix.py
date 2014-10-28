#!/usr/bin/env python
'''
Created on 18.10.2012

@author: Martin Guenther <mguenthe@uos.de>

This ROS node changes the covariance inside an IMU message. This is necessary
for IMU nodes that report a wrong covariance (like android_sensors_driver).
'''

import roslib
roslib.load_manifest('imu_recalibration')

import rospy

from sensor_msgs.msg import Imu

pub = None

def callback(msg_in):
    msg_out = msg_in
    msg_out.orientation_covariance = [2.0e-7, 0.0, 0.0, 0.0, 2.0e-7, 0.0, 0.0, 0.0, 2.0e-7]
    pub.publish(msg_out)

def main():
    global pub
    rospy.init_node('imu_covar_fix', anonymous=True)
    pub = rospy.Publisher('imu_covar_fixed', Imu, queue_size=100)
    rospy.Subscriber('imu', Imu, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
