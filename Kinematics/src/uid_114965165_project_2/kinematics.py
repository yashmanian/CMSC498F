import math
from wheeled_robot_kinematics.srv import *
from wheeled_robot_kinematics.msg import *


def forward(p, a, rd):

  (x,y,theta) = p
  (vl,vr,t) = a
  (axle_length, wheel_radius, max_speed) = rd

  dt = 0.1
  
  # Drive straight
  if vl == vr:
  	x += (vl * math.cos(theta) * t) * wheel_radius
  	y += (vl * math.sin(theta) * t) * wheel_radius
  
  # Rotate
  elif vl == (-vr):
  	theta += (2 * vr * t) / axle_length
  
  # Follow curve
  else:
  	R = (axle_length/2) * ( (vl+vr) / (vr-vl) )
  	w = ((vr-vl)/axle_length) * wheel_radius

  	for i in range(int(t/dt)):
  		icc_x = x - (R* math.sin(theta) )
  		icc_y = y + (R* math.cos(theta) )

  		new_x = ( ( math.cos(w*dt) ) * ( x - icc_x ) ) - ( ( math.sin(w*dt) ) * ( y - icc_y ) ) + icc_x
  		new_y = ( ( math.sin(w*dt) ) * ( x - icc_x ) ) + ( ( math.cos(w*dt) ) * ( y - icc_y ) ) + icc_y
  		new_theta = theta + ( w*dt )

  		x = new_x
  		y = new_y 
  		theta = new_theta

  return x, y, theta

# Drive straight
def move_straight_line(p0,p1,rd):
	(x0,y0,theta0) = p0
	(x1,y1,theta1) = p1
	(axle_length, wheel_radius, max_speed) = rd

	vl = max_speed
	vr = max_speed
	distance = ( math.sqrt( math.pow((y1-y0),2) + math.pow((x1-x0),2) ) ) / wheel_radius
	t = distance / vl

	return vl,vr,t

# Rotate
def rotate(theta0,theta1,rd):
	(axle_length, wheel_radius, max_speed) = rd

	vl = - (max_speed)
  	vr = max_speed
  	distance = (axle_length/2) * (theta1 - theta0)
  	t = distance / vr

  	return vl,vr,t

# Inverse curve
def inverse_solver(p0,p1,rd):

	(x0,y0,theta0) = p0
	(x1,y1,theta1) = p1
	(axle_length, wheel_radius, max_speed) = rd

	t = 0.1
  	solution_found = 0

  	R = (x1-x0)/ ( math.sin(theta1) - math.sin(theta0) )

  	while solution_found == 0:
  		
  		w = (theta1-theta0) / t 

  		vr = w * ( R + (axle_length/2) )
  		vl = w * ( R - (axle_length/2) )


  		(x_new,y_new,theta_new) = forward(p0, (vl,vr,t), rd)
  	
  		if (abs(x_new-x1)) <= 0.05 and (abs(y_new-y1)) <= 0.05:
  			solution_found = 1
  			if (abs(vl)) > max_speed or (abs(vr)) > max_speed:
  				solution_found = 0
  				t += 0.1
  		elif t > 20:
  			solution_found = 1
  		else:
  			t += 0.1

  	return vl,vr,t


# Inverse function
def inverse(p0, p1, rd):
  (x0,y0,theta0) = p0
  (x1,y1,theta1) = p1
  (axle_length, wheel_radius, max_speed) = rd

  vl = 0
  vr = 0
  t = 0

  resp = DiffDriveIKResponse()

  if p0 == p1:
  	print "p0 = p1"

  elif theta0 == theta1:
  	vl,vr,t = move_straight_line(p0,p1,rd)	

	if p1 == forward(p0,(vl,vr,t),rd):
		resp.actions.append( DiffDriveAction(vl,vr,t) )
	else:
		actual_theta1 = theta1
		vl,vr,t = inverse_solver(p0,(x1,y1,(theta1+1.57)),rd)
		resp.actions.append( DiffDriveAction(vl,vr,t) )
		vl,vr,t = rotate((theta1+1.57),theta1,rd)
		resp.actions.append( DiffDriveAction(vl,vr,t) )

  elif (x0,y0) == (x1,y1):
  	vl, vr, t = rotate(theta0,theta1,rd)
  	resp.actions.append( DiffDriveAction(vl,vr,t) )

  else:
  	vl,vr,t = inverse_solver(p0,p1,rd)
  	resp.actions.append( DiffDriveAction(vl,vr,t) )

  return resp



