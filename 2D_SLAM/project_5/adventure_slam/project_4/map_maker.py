from project_4.geometry import *
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PointStamped
from math import sin, cos, degrees
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

import rospy

x_offset = rospy.get_param('x_offset')
y_offset = rospy.get_param('y_offset')
odometry = rospy.get_param('odometry')

# ------------------------------------------------------------------------------
# MapMaker class
# ------------------------------------------------------------------------------
class MapMaker:
  def __init__(self, origin_x, origin_y, resolution, size_x, size_y, transformer):
    self.origin_x = origin_x
    self.origin_y = origin_y
    self.resolution = resolution
    self.size_x = size_x
    self.size_y = size_y
    self.transformer = transformer
    
    self.grid = OccupancyGrid()
    self.grid.header.frame_id = odometry
    self.grid.info.resolution = resolution
    self.grid.info.width = size_x
    self.grid.info.height = size_y
    self.grid.info.origin.position.x = origin_x
    self.grid.info.origin.position.y = origin_y
    self.grid.info.origin.orientation.w = 1.0
    self.grid.data = [-1] * (size_x * size_y)
    self.numScansReceived = 0

    # Insert additional code here if needed
    self.odom = 0

  # ----------------------------------------------------------------------------
  # Convert from world coordinates to grid coordinates. This is convenience 
  # wrapper around the to_grid function from the first part of the assignment.
  # Usage:
  #   (x_grid, y_grid) = self.to_grid(x_world, y_world)
  def to_grid(self, x, y):
    return to_grid(x, y, self.origin_x, self.origin_y, self.size_x, self.size_y, self.resolution)    

  # ----------------------------------------------------------------------------
  # Convert from grid coordinates to world coordinates. This is convenience 
  # wrapper around the to_world function from the first part of the assignment.
  # Usage:
  #   (x_world, y_world) = self.to_world(x_grid, y_grid)
  def to_world(self, gx, gy):
    return to_world(gx, gy, self.origin_x, self.origin_y, self.size_x, self.size_y, self.resolution)    

  # ----------------------------------------------------------------------------
  # Process odometry message. You code should go here.
  def process_odom(self, msg):
    
    position, orientation = msg
    #orientation = msg.pose.pose.orientation
    #orientation = (orientation.x, orientation.y, orientation.z, orientation.w)

    self.x, self.y, self.z = position
    self.theta = euler_from_quaternion(orientation)[2]
    self.odom = 1

    None

  # ----------------------------------------------------------------------------
  # Process laserscan message. You code should go here.
  def process_scan(self, msg):

    self.numScansReceived+=1

    ray_num = 0
    #Start the loop only when there are odometry readings
    if self.odom == 1:
      angle = msg.angle_min
      for i in msg.ranges:
        theta = angle + self.theta
        angle += msg.angle_increment
        if (i < msg.range_min) or (i > msg.range_max):
          continue
        if (np.isnan(i)):
          continue
        x_w = i * cos(theta) + self.x# + x_offset
        y_w = i * sin(theta) + self.y# + y_offset

        #Get the coordinates of the laser points in the world frame
        # angle_robot = msg.angle_min + ray_num*msg.angle_increment
        # ray_num = ray_num + 1

        # x_robot = i * cos(angle_robot + self.theta)
        # y_robot = i * sin(angle_robot + self.theta)

        # x_w = x_robot + self.x
        # y_w = y_robot + self.y

        #Once we have the world coordinates, convert them to grid cells
        #Also convert the coordinates given by the odometry
        (grid_x1, grid_y1) = self.to_grid(x_w, y_w)
        (grid_x0, grid_y0) = self.to_grid(self.x, self.y)
        #Get the grid points for the rays using Bresenham
        laser_gridpoints = bresenham(grid_x0, grid_y0, grid_x1, grid_y1)

        #Loop that gets the values of the cells
        for j in range(len(laser_gridpoints)):

          #Since the map is built in OccupancyGrid, get the index of the grid coordinates 
          #to be able to change the right cell value
          grid_index = to_index(laser_gridpoints[j][0], laser_gridpoints[j][1], self.size_x)

          if j < (len(laser_gridpoints) - 1):
            self.grid.data[grid_index] = 0
          elif j == (len(laser_gridpoints) - 1):
            if i < msg.range_max:
              for k in range(0, 2):
                self.grid.data[grid_index + k] = 100
            else:
              self.grid.data[grid_index] = 0
          else:
            self.grid.data[grid_index] = -1

    None        

