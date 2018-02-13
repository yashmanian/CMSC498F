#!/usr/bin/env python

import rospy
import argparse
from geometry_msgs.msg import Twist
import math
import tf

def polygon_params():

  #-----------------------------------------------------------------------------
  # Initialize node
  #-----------------------------------------------------------------------------

  rospy.init_node('prop_polygon', anonymous=False)

  # This code is required to make sure this node gets simulation time correctly
  simulation = False
  if rospy.has_param('/use_sim_time'):
    if rospy.get_param("/use_sim_time") == True:
      simulation = True

  if simulation:
    rospy.loginfo("Using simulated time.")
    rospy.loginfo("Waiting for the first valid time measurement...")
    t = rospy.Time.now()
    while t == rospy.Time.from_sec(0):
      t = rospy.Time.now()
    rospy.loginfo("Done!")

  else:
    rospy.loginfo("Using real time.")

  #-----------------------------------------------------------------------------
  # Parse command line
  #-----------------------------------------------------------------------------

  parser = argparse.ArgumentParser(description='Polygon Drive Openloop Control')
  parser.add_argument('-d',      default=0.2, type=float)
  parser.add_argument('-n',      default=6, type=int)

  args = parser.parse_args()
  sideLength = args.d
  numSides = args.n

  rospy.loginfo("Polygon parameters:")
  rospy.loginfo("  number of sides: " + str (numSides))
  rospy.loginfo("  side length: " + str(sideLength))

  #-----------------------------------------------------------------------------
  # Drive (your code should go here)
  #-----------------------------------------------------------------------------

  v = 0.2 #max linear speed
  w = 0.5 #max angular speed
  tolerance_dist = 0.3
  tolerance_ang = 0.3
  
  #Proportional gains
  k_rho = 2.1
  k_alpha = 1.75
  k_beta = -0.05

  #Create the publisher to publish Twist messages to the navi topic
  velocity_publisher = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
  vel_msg = Twist()

  # Initialize transform listener
  tfListener = tf.TransformListener()
  # Get current transform between robot base and robot start point
  tf_check = False
  while tf_check == False:
    try:
      (position, orientation) = tfListener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
      tf_check = True
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):  
      continue

  #Function to publish messages for a set amount of time
  def pub_msg(number_of_seconds, vel_msg):
    r = rospy.Rate(10)
    t = rospy.Time.now()
    while rospy.Time.now() - t < rospy.Duration(number_of_seconds):
      velocity_publisher.publish( vel_msg )
      r.sleep()

  #Create a function to get position and orientation from TF
  def location_feedback():
    (position, orientation) = tfListener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
    orientation = tf.transformations.euler_from_quaternion(orientation)

    (x, y, theta) = position
    (roll, pitch, yaw) = orientation

    return x, y, theta, roll, pitch, yaw 


  def polygon_corners(x, y, yaw, numSides, sideLength):
    ini_pose = [x, y, yaw]
    polygon_angle = (2*math.pi)/numSides
    xp_1 = x
    yp_1 = y
    th = yaw
    corners = []
    for i in range(numSides):
      xp = xp_1 + sideLength*math.cos(th)
      yp = yp_1 + sideLength*math.sin(th)


      if th > (math.pi):
        th = th - (2*math.pi)

      corners.append([xp, yp, th])
      xp_1 = xp
      yp_1 = yp
      th = th + polygon_angle 

    return corners


  (x, y, theta, roll, pitch, yaw) = location_feedback()
  corners1 = polygon_corners(x, y, yaw, numSides, sideLength)


  for i in range(0, numSides):

    (x_ref, y_ref, theta_ref) = corners1[i]

    print "Corner: ", i, x_ref, y_ref, theta_ref

    rho = math.sqrt(math.pow((x_ref - x),2) + math.pow((y_ref - y),2))
    alpha = - yaw + math.atan2((y_ref - y), (x_ref - x))
    beta = - math.atan2((y_ref - y), (x_ref - x)) 

    while True: 

      vel_msg.linear.x = k_rho * rho
      if vel_msg.linear.x > v:
        vel_msg.linear.x = v

      alpha = ((alpha + math.pi) % (2*math.pi)) - math.pi
      beta = ((beta + math.pi) % (2 * math.pi)) - math.pi
      vel_msg.angular.z = (k_alpha*alpha) + (k_beta*beta)

      if vel_msg.angular.z > w:
        vel_msg.angular.z = w
      if vel_msg.angular.z < -w:
        vel_msg.angular.z = -w


      if (abs(rho) < tolerance_dist) and (abs(((theta_ref - yaw + math.pi) % (2 * math.pi)) - math.pi) < tolerance_ang):
        break

      pub_msg(0.001, vel_msg)

      (x, y, theta, roll, pitch, yaw) = location_feedback()


      rho = math.sqrt(math.pow((x_ref - x),2) + math.pow((y_ref - y),2))
      alpha = math.atan2((y_ref - y), (x_ref - x)) - yaw
      beta = - math.atan2((y_ref - y), (x_ref - x)) 



#-------------------------------------------------------------------------------
# Main
#-------------------------------------------------------------------------------

if __name__ == '__main__':
  try:
    polygon_params()
  except rospy.ROSInterruptException:
    pass



