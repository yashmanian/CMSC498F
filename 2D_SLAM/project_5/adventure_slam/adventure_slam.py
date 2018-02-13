#!/usr/bin/env python
import rospy, pcl_ros, tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import OccupancyGrid, Odometry
import cv2, math, pcl
import numpy as np
import itertools

pub = rospy.Publisher('/slam_debug', MarkerArray)
odom_broadcaster = tf.TransformBroadcaster()
odom_pub = rospy.Publisher("vo", Odometry)

#Parameters
RANSAC_thresh = rospy.get_param('RANSAC_thresh')
min_outliers = rospy.get_param('min_outliers')
dist_thresh = rospy.get_param('dist_thresh')
angle_thresh = rospy.get_param('angle_thresh')
x_offset = rospy.get_param('x_offset')
y_offset = rospy.get_param('y_offset')
max_dist = rospy.get_param('max_dist')
max_shift = rospy.get_param('max_shift')


def get_line(p1, v1, id_, color=(0,0,1)):
    marker = Marker()
    marker.header.frame_id = "camera_depth_frame"
    marker.header.stamp = rospy.Time()
    marker.lifetime = rospy.Duration(1)
    marker.ns = "slam_debug_ns"
    marker.id = id_
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 0.0
    marker.scale.x = 0.01
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.a = 1.0
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.points.append(Point(p1[0] + 100 * v1[0], p1[1] + 100 * v1[1], 0))
    marker.points.append(Point(p1[0] - 100 * v1[0], p1[1] - 100 * v1[1], 0))
    return marker


#Function to transform points: rotation and translation
def homogeneous(x, y, theta, x_point, y_point):
	robot_points = []
	c, s = np.cos(theta), np.sin(theta)
	R = np.matrix('{} {} {}; {} {} {}; {} {} {}'.format(c, -s, x, s, c, y, 0, 0, 1))
	robot_array = np.matrix('{}; {}; {}'.format(x_point, y_point, 1))
	robot_new = R * robot_array
	return robot_new[0,0], robot_new[1,0]


def laser_callback(scan):
    marker_array = MarkerArray()
    print "..."

    global count
    global current_lines
    global virtual_line
    global total_angle
    global total_dist_x, total_dist_y

    print count
    count = count + 1

    # Convert the laserscan to coordinates
    angle = scan.angle_min
    points = []    
    for r in scan.ranges:
        theta = angle
        angle += scan.angle_increment
        if (r > scan.range_max) or (r < scan.range_min):
            continue
        if (math.isnan(r)):
            continue
        points.append([r * math.sin(theta), r * math.cos(theta)]) 

    # Store the previous fitted lines for matching them later
    if count == 0:
    	previous_lines = []
    	virtual_line_prev = []
    else:
    	previous_lines = current_lines
    	virtual_line_prev = virtual_line
    	current_lines = []
    	virtual_line =[]


    #Initialize variables
    inliers = []
    outliers = []
    match = []
    matched = []
    angles = []
    x_dist = 0
    y_dist = 0
    virtual_line_m = []
    x_inter_prev, y_inter_prev = 0, 0
    x_inter_current, y_inter_current = 0, 0
    intersect_x_curr = []
    intersect_y_curr = []
    intersect_x_prev = []
    intersect_y_prev = []
    dx = []
    dy = []
    Dx, Dy = 0, 0
    dx2 = []
    dy2 = []
    Dx2, Dy2 = 0, 0
    x_curr, y_curr = 0, 0
    angle2 = 0
    flag_pub = 0
    flag_intersect = 0

    # -- Line Fitting -- #
    if len(points) > 0:

		## convert points to pcl type
		points = np.array(points, dtype=np.float32)
		pcl_points = np.concatenate((points, np.zeros((len(points), 1))), axis=1)    
		p = pcl.PointCloud(np.array(pcl_points, dtype=np.float32))

		## create a segmenter object and apply RANSAC
		## to the largest component of the pointcloud
		seg = p.make_segmenter()
		seg.set_model_type(pcl.SACMODEL_LINE)
		seg.set_method_type(pcl.SAC_RANSAC)
		seg.set_distance_threshold (RANSAC_thresh)
		indices, model = seg.segment()

		#Publish the output in RViz
		marker_array.markers.append(get_line((model[1], model[0]), (model[4], model[3]), 0))
		pub.publish(marker_array)

		p1 = (model[1] + 100 * model[4], model[0] + 100 * model[3])
		p2 = (model[1] - 100 * model[4], model[0] - 100 * model[3])
		m = (p2[1] - p1[1]) / (p2[0] - p1[0]) 
		b = p1[1] - m * p1[0]
		distp = abs(p2[0]*p1[1] - p2[1]*p1[0]) / math.sqrt(pow(p2[1]-p1[1],2) + pow(p2[0]-p1[0],2))
		current_lines.append((m , b, p1, p2, distp, (model[4], model[3])))

		# Remove the inliers from the previous line from the pointcloud
		# and use the remaining points (outliers) to fit a new line.
		# This is done to fit all of the lines.
		outliers = pcl_points
		n = 1
		while True:

			if len(outliers) == 0:
				break

			for i in indices:
				inliers.append(pcl_points[i])
			outliers = np.array(list(itertools.compress(pcl_points, [i not in indices for i in range(len(pcl_points))])))

			if len(outliers) < min_outliers:
				break

			out = pcl.PointCloud(np.array(outliers, dtype=np.float32))
			seg = out.make_segmenter()
			seg.set_model_type(pcl.SACMODEL_LINE)
			seg.set_method_type(pcl.SAC_RANSAC)
			seg.set_distance_threshold (RANSAC_thresh)
			indices, model = seg.segment()
			marker_array.markers.append(get_line((model[1], model[0]), (model[4], model[3]), n))
			pub.publish(marker_array)

			pcl_points = outliers
			n = n + 1

			p1 = (model[1] + 100 * model[4], model[0] + 100 * model[3])
			p2 = (model[1] - 100 * model[4], model[0] - 100 * model[3])
			m = (p2[1] - p1[1]) / (p2[0] - p1[0]) 
			b = p1[1] - m * p1[0]
			distp = abs(p2[0]*p1[1] - p2[1]*p1[0]) / math.sqrt(pow(p2[1]-p1[1],2) + pow(p2[0]-p1[0],2))
			current_lines.append((m, b, p1, p2, distp, (model[4], model[3])))


		# -- Matching algorithm -- #
		#For distance, create a virtual line that is orthogonal to the fitted line 
		#The intersection of this line with previous and current line gives approximate distance
		#For angle, follow the formula for angle between intersecting lines
		for i in range(len(current_lines)):

			b1 = 1
			a1 = (-current_lines[i][5][1] * b1) / current_lines[i][5][0]

			p11 = (0 + 100 * a1, 0 + 100 * b1)
			p21 = (0 - 100 * a1, 0 - 100 * b1)
			m1 = (p21[1] - p11[1]) / (p21[0] - p11[0]) 
			b3 = p11[1] - m1 * p11[0]
			virtual_line_m = (m1, b3, p11, p21)

			#marker_array.markers.append(get_line((0, 0), (a1, b1), i+10, color=(0,1,0)))
			#pub.publish(marker_array)

			x_inter_current1 = (current_lines[i][1] - virtual_line_m[1]) / (virtual_line_m[0] - current_lines[i][0])
			y_inter_current1 = current_lines[i][0] * x_inter_current1 + current_lines[i][1]
			x_inter_current1 = x_inter_current1 + x_offset

			dist1_curr = math.sqrt(pow(x_inter_current1,2) + pow(y_inter_current1,2))

			for j in range(len(previous_lines)):

				angle2 = math.atan2((previous_lines[j][0] - current_lines[i][0]), (1 + previous_lines[j][0] * current_lines[i][0]))

				x_inter_prev1 = (previous_lines[j][1] - virtual_line_m[1]) / (virtual_line_m[0] - previous_lines[j][0])
				y_inter_prev1 = previous_lines[j][0] * x_inter_prev1 + previous_lines[j][1]
				x_inter_prev1 = x_inter_prev1 + x_offset

				dist1_prev = math.sqrt(pow(x_inter_prev1,2) + pow(y_inter_prev1,2))

				diff = abs(dist1_curr) - abs(dist1_prev)

				# Matching condition
				#If angle and distance between previous and current are small, then match
				#And append angle and distance to use them to estimate rotation and shift
				if (abs(angle2) < angle_thresh) and abs(diff) < dist_thresh:

					if (x_inter_current1 - dist_thresh <= x_inter_prev1 <= x_inter_current1 + dist_thresh) and (y_inter_current1 - dist_thresh <= y_inter_prev1 <= y_inter_current1 + dist_thresh):

						print "Current line", i, "matches with prev. line", j
						angles.append(angle2)

						match = current_lines[i], previous_lines[j]
						matched.append(match)
						flag_pub = 1

		# -- Localization -- #
		if flag_pub == 1:

			#Rotation of the robot is the mean of all angles of line matches
			robot_angle = np.mean(angles)
			robot_angle = np.round(robot_angle,3)

			# The angle in each frame is added to the global rotation estimation
			total_angle += robot_angle
			total_angle = np.round(total_angle,3)

			#Shift estimation
			#Look for intersections between line matches (previous and current)
			#This is done using the line equation: y = mx + b 
			#Once the intersections for previous and current are determined, append them 
			for ii in range(len(matched)):
				for jj in range(len(matched)):
					if ((matched[jj][1][0] - matched[ii][1][0]) == 0):
						continue

					#Previous intersections 
					x_inter_prev = (matched[ii][1][1] - matched[jj][1][1]) / (matched[jj][1][0] - matched[ii][1][0])
					y_inter_prev = matched[ii][1][0] * x_inter_prev + matched[ii][1][1]
					x_inter_prev = x_inter_prev + x_offset
					y_inter_prev = y_inter_prev + y_offset

					if ((matched[jj][0][0] - matched[ii][0][0]) == 0):
						continue

					#Current intersections
					x_inter_current = (matched[ii][0][1] - matched[jj][0][1]) / (matched[jj][0][0] - matched[ii][0][0])
					y_inter_current = matched[ii][0][0] * x_inter_current + matched[ii][0][1]
					x_inter_current = x_inter_current + x_offset
					y_inter_current = y_inter_current + y_offset

					if abs(x_inter_current) < max_dist and abs(y_inter_current) < max_dist:
						intersect_x_curr.append(x_inter_current)
						intersect_y_curr.append(y_inter_current)
						intersect_x_prev.append(x_inter_prev)
						intersect_y_prev.append(y_inter_prev)
					else:
						continue


			if len(intersect_x_curr) >= 1:
				flag_intersect = 1
			else:
				flag_intersect = 0

			#If there are intersections, get the difference between them
			#First the current intersections have to be transformed to the previous frames
			#Then get the differences and transform them to the global frame and append them  
			if flag_intersect == 1:
			    for i in range(len(intersect_x_curr)):
			    	x_curr, y_curr = homogeneous(0,0, robot_angle, intersect_x_curr[i], intersect_y_curr[i])

			    	x_dist = x_curr - intersect_x_prev[i]
			    	x_dist = np.round(x_dist, 3)

			    	y_dist = y_curr - intersect_y_prev[i]
			    	y_dist = np.round(y_dist, 3)

			    	x_dist, y_dist = homogeneous(0,0, total_angle, x_dist, y_dist)
			    	if abs(x_dist) < max_shift and abs(y_dist) < max_shift:
			    		dx.append(x_dist)
			    		dy.append(y_dist)
			    		print x_dist, y_dist

			    # The displacement in each frame is the mean of the distances between line intersections
			    Dx = np.mean(dx)
			    Dy = np.mean(dy)
			    
			    #The local shift is added to the total distance of the robots with respect to the origin
			    total_dist_x += Dx
			    total_dist_y += Dy		    		

			    #Lastly, publish a transform message with the rotation and shift
			    current_time = rospy.Time.now()
			    odom_quat = tf.transformations.quaternion_from_euler(0, 0, -total_angle)
			    odom_trans = (total_dist_x, total_dist_y, 0)

			    odom_broadcaster.sendTransform(
			        (0,0,0),
			        odom_quat,
			        current_time,
			        "odom_visual2",
			        "base_footprint"
			    )

			    odom_broadcaster.sendTransform(
			        odom_trans,
			        (0,0,0,1),
			        current_time,
			        "odom_visual",
			        "odom_visual2"
			    )

			    #Also publish an odometry message with shift and rotatiion
			    odom = Odometry()
			    odom.header.stamp = current_time
			    odom.header.frame_id = "vo"
			    odom.pose.pose = Pose(Point(total_dist_x, total_dist_y, 0.), Quaternion(*odom_quat))
			    odom.child_frame_id = "base_footprint"
			    odom_pub.publish(odom)

			#If there are no intersections, localization won't work completely fine
			#Create a virtual orthogonal line to the fitted lines
			#Get the intersections as in the previous case and the shift in the same way
			#Translation will drift along the direction of the line
			if flag_intersect == 0:

				print "No intersections"
				#Case with no intersections

				b = 1
				a = (-current_lines[0][5][1] * b) / current_lines[0][5][0]

				#marker_array.markers.append(get_line((0, 0), (a, b), 1, color=(0,1,0)))
				#pub.publish(marker_array)

				p1 = (0 + 100 * a, 0 + 100 * b)
				p2 = (0 - 100 * a, 0 - 100 * b)
				m = (p2[1] - p1[1]) / (p2[0] - p1[0]) 
				b2 = p1[1] - m * p1[0]
				virtual_line = (m, b2, p1, p2)

				#Done like in the previous part
				if len(virtual_line_prev) != 0:
					for k in range(len(matched)):

						#Previous virtual intersection
						x_inter_prev = (matched[k][1][1] - virtual_line_prev[1]) / (virtual_line_prev[0] - matched[k][1][0])
						y_inter_prev = matched[k][1][0] * x_inter_prev + matched[k][1][1]
						x_inter_prev = x_inter_prev + x_offset
						y_inter_prev = y_inter_prev# + 0.0475

						#Current virtual intersection
						x_inter_current = (matched[k][0][1] - virtual_line[1]) / (virtual_line[0] - matched[k][0][0])
						y_inter_current = matched[k][0][0] * x_inter_current + matched[k][0][1]
						x_inter_current = x_inter_current + x_offset
						y_inter_current = y_inter_current# + 0.0475

						#Again, rotate the current intersections to the revious frame
						x_curr, y_curr = homogeneous(0,0, robot_angle, x_inter_current, y_inter_current)

						#Get distance between intersections
						x_dist = x_curr - x_inter_prev
						x_dist = np.round(x_dist, 3)

						y_dist = y_curr - y_inter_prev
						y_dist = np.round(y_dist, 3)

						#And transform this distances to the global frame
						x_dist, y_dist = homogeneous(0,0, total_angle, x_dist, y_dist)

						dx2.append(x_dist)
						dy2.append(y_dist)

					#Shift is mean of all the virtual intersectiions
					Dx2 = np.mean(dx2)
					Dy2 = np.mean(dy2)

					total_dist_x += Dx2
					total_dist_y += Dy2

					#Again, send a transform message with the rotation and translation
					current_time = rospy.Time.now()
					odom_quat = tf.transformations.quaternion_from_euler(0, 0, -total_angle)
					odom_trans = (total_dist_x, total_dist_y, 0)


					odom_broadcaster.sendTransform(
					    (0,0,0),
					    odom_quat,
					    current_time,
					    "odom_visual2",
					    "base_footprint"
					)

					odom_broadcaster.sendTransform(
					    odom_trans,
					    (0,0,0,1),
					    current_time,
					    "odom_visual",
					    "odom_visual2"
					)

					#Odometry message
					odom = Odometry()
					odom.header.stamp = current_time
					odom.header.frame_id = "vo"
					odom.pose.pose = Pose(Point(total_dist_x, total_dist_y, 0.), Quaternion(*odom_quat))
					odom.child_frame_id = "base_footprint"
					odom_pub.publish(odom)
				

	#When there are no laserscan at all, no way we can locate. 
	#Just get the robots current position as origin until there are laserscan lectures
    else:
		print "Error: No laserscan lectures"

		current_time = rospy.Time.now()
		odom_quat = tf.transformations.quaternion_from_euler(0, 0, 0)
		odom_trans = (0, 0, 0)


		odom_broadcaster.sendTransform(
		    (0,0,0),
		    odom_quat,
		    current_time,
		    "odom_visual2",
		    "base_footprint"
		)


		odom_broadcaster.sendTransform(
		    odom_trans,
		    (0,0,0,1),
		    current_time,
		    "odom_visual",
		    "odom_visual2"
		)

		odom = Odometry()
		odom.header.stamp = current_time
		odom.header.frame_id = "vo"
		odom.pose.pose = Pose(Point(0, 0, 0.), Quaternion(*odom_quat))
		odom.child_frame_id = "base_footprint"
		odom_pub.publish(odom)

    print "..."



      

def main():
    rospy.init_node('adventure_slam', anonymous=True)

    rospy.Subscriber("/scan", LaserScan, laser_callback)
    rospy.spin()


if __name__ == '__main__':

	#Global Variables Initialization
    count = 0
    current_lines = []
    virtual_line = []
    total_angle = 0
    total_dist_x = 0
    total_dist_y = 0
    
    try:
        main()
    except rospy.ROSInterruptException:
        pass
