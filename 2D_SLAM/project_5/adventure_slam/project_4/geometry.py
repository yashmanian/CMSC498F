import math

# ------------------------------------------------------------------------------
# Convert world coordinate to grid coordinate
# ------------------------------------------------------------------------------
def to_grid(x, y, origin_x, origin_y, size_x, size_y, resolution):

    #If the coordinate is inside the grid
    if x >= origin_x and y >= origin_y:

        #Distance wrt origin
        A_x = x - origin_x
        A_y = y - origin_y

        #Grid values for this distances
        grid_x = A_x / resolution
        grid_y = A_y / resolution

        grid_x = int(grid_x)
        grid_y = int(grid_y)

        #Only return this grid coordinate if the values are inside of the grid itself
        if grid_x < size_x and grid_y < size_y:
            return (grid_x, grid_y)
        else:
            return None
    else:

        return None
    
# ------------------------------------------------------------------------------
# Convert grid coordinate to world coordinate
# ------------------------------------------------------------------------------
def to_world(gx, gy, origin_x, origin_y, size_x, size_y, resolution):

    #Convert only idf the coordinates are inside the boundaries
    if gx >= origin_x and gy >= origin_y:
        #Return the coordinates of the center point of the cell 
        x = origin_x + (gx * resolution) + (resolution / 2)
        y = origin_y + (gy * resolution) + (resolution / 2)

        if x <= size_x and y <= size_y:
            return (x, y)
        else:

            return None
    else:
        return None
  
# ------------------------------------------------------------------------------
# Convert grid coordinate to map index
# ------------------------------------------------------------------------------
def to_index(gx, gy, size_x):
  return gy * size_x + gx

# ------------------------------------------------------------------------------
# Given two integer coordinates return a list of coordinates of a line between 
# the two points.
# ------------------------------------------------------------------------------
def bresenham(x0, y0, x1, y1):

    # Setup initial conditions
    dx = x1 - x0
    dy = y1 - y0
 
    # Determine how steep the line is
    is_steep = abs(dy) > abs(dx)
 
    # Rotate line
    if is_steep:
        x0, y0 = y0, x0
        x1, y1 = y1, x1
 
    # Swap start and end points if necessary and store swap state
    swapped = False
    if x0 > x1:
        x0, x1 = x1, x0
        y0, y1 = y1, y0
        swapped = True
 
    # Recalculate differentials
    dx = x1 - x0
    dy = y1 - y0
 
    # Calculate error
    error = int(dx / 2.0)
    ystep = 1 if y0 < y1 else -1
 
    # Iterate over bounding box generating points between start and end
    y = y0
    points = []
    for x in range(x0, x1 + 1):
        coord = (y, x) if is_steep else (x, y)
        points.append(coord)
        error -= abs(dy)
        if error < 0:
            y += ystep
            error += dx
 
    # Reverse the list if the coordinates were swapped
    if swapped:
        points.reverse()
    return points