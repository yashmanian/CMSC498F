#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool

rospy.init_node('adventure_starter', anonymous=True)
navigation_pub = rospy.Publisher('navigation_arguments', Pose2D)
sift_pub = rospy.Publisher('sift_arguments', Bool)
gripper_pub = rospy.Publisher('gripper_arguments', Bool)
object_positions = [(4, -1, 0), (4, -2, 0), (4, 2, 0)]
counter = 0


def generate_navigation_message():
    return Pose2D(x=object_positions[counter][0], y=object_positions[counter][1], theta=object_positions[counter][2])


def navigation_callback(message):
    if message.data:
        print 'Successful Navigation'
        sift_pub.publish(Bool(True))
    else:
        print 'Error in navigation'


def sift_callback(message):
    if message.data:
        print 'Successful Sift'
        gripper_pub.publish(Bool(True))
    else:
        print 'Error in sift'


def gripper_callback(message):
    print 'Gripper called'
    if message.data:
        print 'Successful Gripper'
        global counter
        counter = counter + 1
        if counter == len(object_positions):
            print 'Finish all tasks'
            return
        navigation_pub.publish(generate_navigation_message())
    else:
        print 'Error in gripper'


def main():
    rospy.Subscriber('navigation_flag', Bool, navigation_callback)
    rospy.Subscriber('sift_flag', Bool, sift_callback)
    rospy.Subscriber('gripper_flag', Bool, gripper_callback)
    rospy.sleep(1)
    navigation_pub.publish(generate_navigation_message())
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass