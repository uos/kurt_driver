#!/usr/bin/env python
import roslib; roslib.load_manifest('kurt_base')
import rospy
from sensor_msgs.msg import JointState
def fake_wheel_publisher():
    pub = rospy.Publisher('/joint_states', JointState, queue_size=100)
    rospy.init_node('fake_wheel_publisher')
    while not rospy.is_shutdown():
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name = ['left_front_wheel_joint', 'left_middle_wheel_joint', 'left_rear_wheel_joint', 'right_front_wheel_joint', 'right_middle_wheel_joint', 'right_rear_wheel_joint']
        js.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        pub.publish(js)
        rospy.sleep(0.02)
if __name__ == '__main__':
    try:
        fake_wheel_publisher()
    except rospy.ROSInterruptException: pass
