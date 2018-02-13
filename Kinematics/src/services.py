#!/usr/bin/env python 

import rospy
from wheeled_robot_kinematics.srv import *
from wheeled_robot_kinematics.msg import *
from uid_114965165_project_2.kinematics import *

def forward_func(req):


	rd = ( rospy.get_param("/axle_length") , rospy.get_param("/wheel_radius") , rospy.get_param("/max_speed") )
	p = ( req.pose.x , req.pose.y , req.pose.theta )
	a = ( req.action.left_velocity , req.action.right_velocity , req.action.time )

	(x,y,theta) = forward( p , a , rd )

	resp = DiffDriveFKResponse()
	resp.end_pose.x = x
	resp.end_pose.y = y
	resp.end_pose.theta = theta

	return resp

def inverse_func(req):

	rd = ( rospy.get_param("/axle_length") , rospy.get_param("/wheel_radius") , rospy.get_param("/max_speed") )
	p0 = ( req.pose.x , req.pose.y , req.pose.theta )
	p1 = ( req.end_pose.x , req.end_pose.y , req.end_pose.theta )

	resp = DiffDriveIKResponse()
	resp = inverse(p0,p1,rd)

	return resp

def forward_inverse_server():
	rospy.init_node('kinematics')
	f = rospy.Service('kinematics/forward', DiffDriveFK, forward_func)
	i = rospy.Service('kinematics/inverse', DiffDriveIK, inverse_func)
	rospy.spin()

if __name__ == '__main__':
	forward_inverse_server()

