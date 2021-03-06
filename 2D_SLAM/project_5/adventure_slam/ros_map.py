#!/usr/bin/python

from project_4.map_maker import *
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from project_4.args_parser import parse_args
import rospy
import tf


rospy.init_node('map_maker')
size, origin, resolution, transform = parse_args()
transformer = tf.TransformListener()
m = MapMaker(origin, origin, resolution, size, size, transformer)
pub = rospy.Publisher('/map', OccupancyGrid)

odometry = rospy.get_param('odometry')

#r = rospy.Rate(50)
tf_check = False
while tf_check == False:
	try:
		msg = transformer.lookupTransform(odometry, "/camera_depth_frame", rospy.Time(0))
		tf_check = True
	except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		continue

def mapping(scan):
	msg = transformer.lookupTransform(odometry, "/camera_depth_frame", rospy.Time(0))
	m.process_odom(msg)
	m.process_scan(scan)
	pub.publish(m.grid)

def main():
	r = rospy.Rate(50)
	while not rospy.is_shutdown():
		rospy.Subscriber("/scan", LaserScan, mapping)
		rospy.spin()
		r.sleep()
		if rospy.is_shutdown():
			break


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
