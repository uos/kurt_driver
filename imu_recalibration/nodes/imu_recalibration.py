#!/usr/bin/env python
'''
Created on 16.10.2012

@author: Jochen Sprickerhof <jochen@sprickerhof.de> and Martin Guenther <mguenthe@uos.de>
'''

import roslib; roslib.load_manifest('imu_recalibration')
import rospy
import tf
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from math import pi

class ImuRecalibration:
    """
    This ROS node recalibrates the IMU (i.e., calculates the IMU drift and
    subtracts it) whenever the robot is standing.
    """

    # number of IMU messages to average over
    NUM_SAMPLES = 100

    # drifts larger than this value are ignored (rad/s)
    # (the maximum drift actually measured is 2pi in 120 seconds)
    MAX_DELTA = 2.0 * pi / 120.0 / NUM_SAMPLES

    # the robot is considered to be moving if the angular velocity is larger than this (rad/s)
    MAX_ANGULAR_VEL = 2.0 * pi / 3600.0

    def __init__(self):
        rospy.init_node('imu_recalibration')
        rospy.Subscriber("imu", Imu, self.imu_callback)
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.pub = rospy.Publisher('imu_recalibrated', Imu, queue_size=100)
        self.error = 0.0
        self.delta = 0.0
        self.delta_new = 0.0
        self.yaw_old = 0.0
        self.calibration_counter = -1
        #self.debug_file = open('/tmp/imu_recalibration.txt', 'w')
        rospy.loginfo('imu_recalibration node initialized.')

    @classmethod
    def normalize(cls, angle):
        while angle < pi:
            angle += 2.0 * pi
        while angle > pi:
            angle -= 2.0 * pi
        return angle

    def imu_callback(self, msg_in):
        (r, p, yaw) = tf.transformations.euler_from_quaternion([msg_in.orientation.x,
            msg_in.orientation.y, msg_in.orientation.z, msg_in.orientation.w])

        self.calibration_counter += 1
        if self.calibration_counter == 0:
            # start of calibration interval --> reset
            self.delta_new = 0.0
        else:
            # inside calibration interval --> accumulate
            self.delta_new += self.normalize(yaw - self.yaw_old)

        if self.calibration_counter == ImuRecalibration.NUM_SAMPLES:
            self.delta_new /= ImuRecalibration.NUM_SAMPLES
            # end of calibration interval
            if abs(self.delta_new) < ImuRecalibration.MAX_DELTA:
                self.delta = self.delta_new
                rospy.loginfo("New IMU delta: %f" % self.delta)
            else:
                # this can happen if the base was switched off and on again
                rospy.logwarn("IMU delta too large, ignoring: %f" % self.delta_new)

            self.calibration_counter = -1

        self.error += self.delta;
        self.error = self.normalize(self.error)
        self.yaw_old = yaw
        yaw_new = self.normalize(yaw - self.error)

        q = tf.transformations.quaternion_from_euler(r, p, yaw_new)
        msg_out = msg_in
        msg_out.orientation.x = q[0]
        msg_out.orientation.y = q[1]
        msg_out.orientation.z = q[2]
        msg_out.orientation.w = q[3]

        #self.debug_file.write("%s %s %s %s %s %s\n" % (yaw, yaw_new, self.error, self.delta, self.delta_new, self.calibration_counter))
        self.pub.publish(msg_out)

    def odom_callback(self, msg):
        if abs(msg.twist.twist.angular.z) > ImuRecalibration.MAX_ANGULAR_VEL:
            rospy.logdebug("Resetting imu_recalibration (robot is moving)")
            self.calibration_counter = -1

if __name__ == '__main__':
    ImuRecalibration()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
