#!/usr/bin/env python

import numpy as np               # Linear algebra
import matplotlib.pyplot as plt  # Plotting
import math


#-------------------------------------------------------------------------------
def GoToGoal(x_g, y_g):

  #-----------------------------------------------------------------------------
  # Initialization (you don't have to modify anything here)
  #-----------------------------------------------------------------------------

  # Define a map
  nX = 100
  nY = 100
  X, Y = np.meshgrid(np.linspace(0,nX,100), np.linspace(0,nY,100))

  # Define start position
  x_0 = 20
  y_0 = 20

  # Define Obstacles
  obstacles = []
  obstacles.append([50, 20])
  obstacles.append([80, 35])

  #-----------------------------------------------------------------------------
  # Calculate potential field for each cell of the map
  #-----------------------------------------------------------------------------

  k_att = 1
  k_rep = 10000
  rho_0 = 10

  U = np.empty([len(X), len(Y)])
  V = np.empty([len(X), len(Y)])

  for x in range(100):
    for y in range(100):
      u_force, v_force = Forces_calc([x, y], [x_g, y_g], obstacles, k_att, k_rep, rho_0)

      U[x, y] = u_force
      V[x, y] = v_force

  #-----------------------------------------------------------------------------
  # Finding the robot path using the potential field
  #-----------------------------------------------------------------------------

  path = []
  path = computePath([x_0, y_0], [x_g, y_g], U, V)
  print "- - > COMPUTED PATH OF SIZE: ", len(path)
  #-----------------------------------------------------------------------------
  # Plot results (you don't have to modify anything here)
  #-----------------------------------------------------------------------------
  nth = 1
  fig = plt.figure()
  Q = plt.quiver(Y[::nth, ::nth], X[::nth, ::nth], U[::nth, ::nth], V[::nth, ::nth],
        pivot='mid', units='width')


  plt.axis([-5, 105, -5, 105])
  plt.title('Robot path')
  plt.xlabel('X')
  plt.ylabel('Y')

  # Plot Path
  path_x = []
  path_y = []
  for i in range(len(path)-1):
    path_x.append(path[i][0])
    path_y.append(path[i][1])

  plt.plot(path_x, path_y, 'r', linewidth=4)

  # Plot Start and goal positions
  plt.plot([x_0], [y_0], 'bo', markersize=10)
  plt.plot([x_g], [y_g], 'go', markersize=10)

  obst1 = plt.Circle((50, 20), 5, color='#6699cc', alpha=0.4)
  obst2 = plt.Circle((80, 35), 5, color='#6699cc', alpha=0.4)
  ax = fig.add_subplot(111)
  ax.add_artist(obst1)
  ax.add_artist(obst2)
  # Show plot
  plt.show()

def computePath(initial_location, final_location, U, V):
  current_location = initial_location;
  currDist2Goal = math.sqrt((final_location[1] - initial_location[1])**2 + (final_location[0] - initial_location[0])**2)
  finalPath = []
  finalPath.append(current_location) 
  currSteps = 0
  maxSteps = 1000

  while currDist2Goal > 1:
    if currSteps > maxSteps:
      return finalPath

    current_force_u = U[current_location[0], current_location[1]]
    current_force_v = V[current_location[0], current_location[1]]

    neighbor_location = []
    action = -1;


    if abs(current_force_u) > abs(current_force_v):
      if current_force_u > 0:
        neighbor_location = [current_location[0]+1, current_location[1]]
        action = 1
      else:
        neighbor_location = [current_location[0]-1, current_location[1]]
        action = 2
    else:
      if current_force_v > 0:
        neighbor_location = [current_location[0], current_location[1]+1]
        action = 3
      else:
        neighbor_location = [current_location[0], current_location[1]-1]
        action = 4

    # Checking if the state is in local minima    
    if len(finalPath) > 1:
      dist2PrevPoint = math.sqrt((finalPath[-2][1] - neighbor_location[1])**2 + (finalPath[-2][0] - neighbor_location[0])**2)
      if dist2PrevPoint < 0.1:


        if action < 3:

          if (current_location[1] - final_location[1]) < 0:
            neighbor_location = [current_location[0], current_location[1]+1] 
            if neighbor_location[1] >= 100:
              neighbor_location = [current_location[0], current_location[1]-1] 
          else:
            neighbor_location = [current_location[0], current_location[1]-1] 
            if neighbor_location[1] < 0:
              neighbor_location = [current_location[0], current_location[1]+1] 
        else:


          if (current_location[0] - final_location[0]) < 0:
            neighbor_location = [current_location[0]+1, current_location[1]] 
            if neighbor_location[0] >= 100:
              neighbor_location = [current_location[0]-1, current_location[1]] 
          else:
            neighbor_location = [current_location[0]-1, current_location[1]] 
            if neighbor_location[0] >= 100:
              neighbor_location = [current_location[0]+1, current_location[1]] 
          
    if neighbor_location[0] > 100:
      if (current_location[0] - final_location[0]) < 0:
        neighbor_location = [current_location[0], current_location[1]+1] 
      else:
        neighbor_location = [current_location[0], current_location[1]-1] 

    if neighbor_location[0] <= 0:
      if (current_location[0] - final_location[0]) < 0:
        neighbor_location = [current_location[0], current_location[1]+1] 
      else:
        neighbor_location = [current_location[0], current_location[1]-1]

    if neighbor_location[1] > 100:
      if (current_location[1] - final_location[1]) < 0:
        neighbor_location = [current_location[0]+1, current_location[1]] 
      else:
        neighbor_location = [current_location[0]-1, current_location[1]]  

    if neighbor_location[1] <= 0:
      if (current_location[1] - final_location[1]) < 0:
        neighbor_location = [current_location[0]+1, current_location[1]] 
      else:
        neighbor_location = [current_location[0]-1, current_location[1]]                         

    if len(neighbor_location) > 0:    
      current_location = neighbor_location
    else:
      print "No Neighbor to Visit..!!!"

    currDist2Goal = math.sqrt((final_location[1] - current_location[1])**2 + (final_location[0] - current_location[0])**2)
    finalPath.append(current_location)
    currSteps = currSteps + 1

  finalPath.append(final_location) 
  return finalPath

def Forces_calc(current_location, final_location, obstacles, k_att, k_rep, rho_0):
  currAngle2Goal = math.atan2(final_location[1] - current_location[1], final_location[0] - current_location[0])
  currDist2Goal = math.sqrt((final_location[1] - current_location[1])**2 + (final_location[0] - current_location[0])**2)
  f_attrat_mag = k_att*currDist2Goal
  foa_u = f_attrat_mag*math.cos(currAngle2Goal)
  foa_v = f_attrat_mag*math.sin(currAngle2Goal)
  f_attraction = [foa_u, foa_v]                                                   

  for_u = 0
  for_v = 0

  for i in range(len(obstacles)):
    currAngle2Obst = math.atan2(obstacles[i][1] - current_location[1], obstacles[i][0] - current_location[0])
    currDist2Obst = math.sqrt((obstacles[i][1] - current_location[1])**2 + (obstacles[i][0] - current_location[0])**2)

    if (currDist2Obst == 0):
      f_rep1_x = 0;
      f_rep1_y = 0;
    elif (currDist2Obst < rho_0): 
      grad_x = (current_location[0] - obstacles[i][0])/currDist2Obst
      grad_y = (current_location[1] - obstacles[i][1])/currDist2Obst
      rho_ob1 = currDist2Obst
      f_rep1_x = k_rep * ( (1/rho_ob1) - ( 1/rho_0 ) ) * ( 1/ (rho_ob1**2) ) * grad_x;
      f_rep1_y = k_rep * ( (1/rho_ob1) - ( 1/rho_0 ) ) * ( 1/ (rho_ob1**2) ) * grad_y;
    else:
      f_rep1_x = 0;
      f_rep1_y = 0;

    for_u = for_u + f_rep1_x
    for_v = for_v + f_rep1_y

  f_total_u = foa_u + for_u
  f_total_v = foa_v + for_v

 

  return f_total_u, f_total_v  
#-------------------------------------------------------------------------------
if __name__ == '__main__':
  x_g = input("Enter goal X coordinate: ")
  y_g = input("Enter goal Y coordinate: ")
  GoToGoal(x_g, y_g)
