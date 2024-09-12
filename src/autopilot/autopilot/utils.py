import numpy as np
from enum import Enum

import geopy
import geopy.distance
import pyproj
import math

from .position import Position



class AutopilotMode(Enum):
    Disabled = 0
    Full_RC = 1
    Hold_Best_Sail = 2
    Hold_Heading = 3
    Hold_Heading_And_Best_Sail = 4
    Waypoint_Mission = 5
    
class States(Enum):
    NORMAL = 0
    CW_TACKING = 1
    CCW_TACKING = 2
    STALL = 3
    # JIBE = 4

class Maneuvers(Enum):
    AUTOPILOT_DISABLED = 0
    STANDARD = 1
    TACK = 2
    JIBE = 3
  
  


  
    
def check_float_equivalence(float1, float2):
    return abs(float1 - float2) <= 0.001


def cartesian_vector_to_polar(x, y):
    """
        Converts a cartesian vector (x and y coordinates) to polar form (magnitude and direction).
        Outputs a tuple of magnitude and direction of the inputted vector
    """
    # arctan2 doesn't like when we pass 2 zeros into it so we should cover that case
    if x == 0. and y == 0.:
        return 0., 0.
    magnitude = np.sqrt(x**2 + y**2)
    direction = np.arctan2(y, x) # radians
    direction = direction * (180/np.pi)  # angle from -180 to 180 degrees
    direction = direction % 360  # angle from 0 to 360 degrees
    return magnitude, direction
    
def get_distance_between_angles(angle1, angle2):
    # https://stackoverflow.com/questions/1878907/how-can-i-find-the-smallest-difference-between-two-angles-around-a-point
    return -1 * ((float(angle1) - float(angle2) + 180) % 360 - 180)
    

def get_bearing(current_pos: Position, destination_pos: Position):
    '''
    utility function to get the bearing towards a specific destination point, from our current location.
    This returns the bearing as an angle between 0 to 360, counter clockwise, measured from east
    '''
    # X = float(destination_pos.longitude) - float(current_pos.longitude) #delta X
    # Y = float(destination_pos.latitude) - float(current_pos.latitude) #delta Y
    # ang_east = (math.atan2(Y, X)*(180/math.pi) + 360)%360
    # return ang_east

    cur_lat, cur_lon = current_pos.get_lat_lon()
    des_lat, des_lon = destination_pos.get_lat_lon()
    azimuth_heading, _, _ = pyproj.Geod(ellps='WGS84').inv(cur_lon, cur_lat, des_lon, des_lat)
    return (-azimuth_heading+90) % 360
    # return (-azimuth_heading + 90) % 360   # azimuth is cw from true north while we want ccw from true east

def get_distance_between_positions(pos1: Position, pos2: Position):
    return geopy.distance.geodesic(pos1.get_lat_lon(), pos2.get_lat_lon()).m



def does_line_intersect_obstacle(start_point: list, end_point: list, obstacle_position: list, obstacle_sizes):
    """
    Adapted from top answer of this stack overflow post: https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm.
    It is assumed that all obstacles are spheres with a size of the obstacle_size constant.
    Sorry if the variables don't have very creative names and if its not well documented, if you would like to understand how this algorithm works, I encourage you to read the page linked above
    """
    # initializing
    e = np.array(start_point)
    l = np.array(end_point)
    c = np.array(obstacle_position)
    r = obstacle_sizes
    
    d = l - e
    f = e - c
    
    # using the algorithm from the stack overflow post (basically solving a quadratic equation)
    a = np.dot(d, d)
    b = 2*np.dot(f, d)
    c = np.dot(f, f) - r*r

    discriminant = b*b-4*a*c
    
    return discriminant >= 0

    
def does_line_intersect_any_obstacles(start_point: list, end_point: list, obstacle_positions: list):
    """
    Just calls the method _does_line_intersect_obstacle on all of the obstacles. If you would like to learn how the algorithm works, 
    look at the stack overflow post linked at that method
    """
    for obstacle_position in obstacle_positions:
        result = does_line_intersect_obstacle(start_point, end_point, obstacle_position)
        if result == True: 
            return True
    
    return False

        
def does_line_violate_no_sail_zone(start_point: list, end_point: list, true_wind_angle: float, no_sail_zone_size: float):
    """wind direction is ccw from true north"""
    displacement = np.array(end_point) - np.array(start_point)
    displacement_magnitude = np.sqrt(displacement[0]**2 + displacement[1]**2 + displacement[2]**2)
    normalized_displacement = displacement/ displacement_magnitude
    
    true_wind_angle = (true_wind_angle + 90) % 360    # transform to ccw from true east
    up_wind_angle = (true_wind_angle + 180) % 360
    
    up_wind_vector = np.array(np.cos(up_wind_angle), np.sin(up_wind_angle))
    
    # find the angle between these two vectors
    angle_between = np.arccos(np.dot(up_wind_vector, normalized_displacement))
    
    if -no_sail_zone_size < angle_between < no_sail_zone_size:
        return True
    
    return False




def angle_between_vectors(v1: np.ndarray, v2: np.ndarray):
    v1_normalized = v1/ np.linalg.norm(v1)
    v2_normalized = v2/ np.linalg.norm(v2)
    
    return np.rad2deg(np.arccos(np.clip(np.dot(v1_normalized, v2_normalized), -1, 1)))
    
    
def is_angle_between_boundaries(angle, boundary1, boundary2):
    # TODO make the names of these a little bit less cringeworthy
    angle = np.deg2rad(angle)
    boundary1 = np.deg2rad(boundary1)
    boundary2 = np.deg2rad(boundary2)
    
    angle_vector = np.array([np.cos(angle), np.sin(angle)])
    boundary1_vector = np.array([np.cos(boundary1), np.sin(boundary1)])
    boundary2_vector = np.array([np.cos(boundary2), np.sin(boundary2)])
  
    return check_float_equivalence(
        angle_between_vectors(boundary1_vector, angle_vector) + angle_between_vectors(angle_vector, boundary2_vector), 
        angle_between_vectors(boundary1_vector, boundary2_vector)
    )
    
    
def get_maneuver_from_desired_heading(heading, desired_heading, true_wind_angle):
    
    # wind ccw from true east (in other words the polar angle not relative to where the boat is facing)
    polar_downwind_angle = (true_wind_angle + heading) % 360
    
    #Calculate median into the wind angle (nominal angle at which course is heading straight into the wind)
    #True wind                    0     45    90   135  179 | 180  225  270  315  360
    #median into wind angle       180   225   270  315  359 | 0    45   90   135  180
    polar_upwind_angle = (polar_downwind_angle + 180) % 360

    if is_angle_between_boundaries(polar_downwind_angle, heading, desired_heading): 
        return Maneuvers.JIBE
    
    elif is_angle_between_boundaries(polar_upwind_angle, heading, desired_heading): 
        return Maneuvers.TACK
        
    else: 
        return Maneuvers.STANDARD



# def get_maneuver_at_position(heading, current_position:Position, desired_position:Position, true_wind_angle):
#     """
#     takes in the following:
#         - heading angle measured ccw from true east
#         - the index of the current waypoint that we are headed towards
#         - the true wind angle measured ccw from the centerline of the boat. This is measured from on the boat.
        
#     returns either "tack", "jibe", or "standard" denoting the type of maneuver that the boat should perform once it reaches the waypoint
#     TODO: doesn't work for the first waypoint so fix that
#     """
    
#     # This is the (0, 0) for our coordinate scheme. We have to pick something and it might as well be this (this is mostly arbitrary)
#     local_coordinate_reference = current_position.get_lon_lat()
    
#     cur_waypoint_position = current_position.get_local_coordinates(local_coordinate_reference)
#     next_waypoint_position = desired_position.get_local_coordinates(local_coordinate_reference)

#     heading_vector = np.array([np.cos(np.deg2rad(heading)), np.sin(np.deg2rad(heading))])
#     next_bearing_vector = next_waypoint_position - cur_waypoint_position

#     _, prev_course_angle = cartesian_vector_to_polar(heading_vector[0], heading_vector[1])
#     _, new_course_angle = cartesian_vector_to_polar(next_bearing_vector[0], next_bearing_vector[1])
    
#     # wind ccw from true east (in other words the polar angle not relative to where the boat is facing)
#     polar_downwind_angle = (true_wind_angle + heading) % 360
    
#     #Calculate median into the wind angle (nominal angle at which course is heading straight into the wind)
#     #True wind                    0     45    90   135  179 | 180  225  270  315  360
#     #median into wind angle       180   225   270  315  359 | 0    45   90   135  180
#     polar_upwind_angle = (polar_downwind_angle + 180) % 360

#     print()
#     print(f"previous course angle: {prev_course_angle}")
#     print(f"new course angle: {new_course_angle}")
#     print(f"downwind angle: {polar_downwind_angle}")
#     print(f"upwind_angle: {polar_upwind_angle}")
#     print()
    
#     if is_angle_between_boundaries(polar_downwind_angle, prev_course_angle, new_course_angle): 
#         return Maneuvers.JIBE
    
#     elif is_angle_between_boundaries(polar_upwind_angle, prev_course_angle, new_course_angle): 
#         return Maneuvers.TACK
        
#     else: 
#         return Maneuvers.STANDARD

# class Route:
    
#     def __init__(self, waypoints: list[Position]):
#         # TODO perhaps add a variable for the original waypoints here so that we know which waypoints are important and which ones are intermediate waypoints
#         self.waypoints: list[Position] = waypoints
    
    
#     def get_maneuver_at_waypoint(self, heading: float, cur_waypoint_index: int, true_wind_angle: float):
#         return get_maneuver_at_position(heading, self.waypoints[cur_waypoint_index], self.waypoints[cur_waypoint_index+1], true_wind_angle)
        
    
#     def _is_route_valid(self, cur_position, waypoints, cur_waypoint_index):
#         pass

    
#     def recalculate_route(self, cur_position: Position, true_wind_angle: float, cur_waypoint_index: int):
#         """
#         TODO make this so that it recalculates mainly based on changing winds. If the winds change then so do our intermediate waypoints
#         Make sure that the route doesn't end up hitting an obstacle or sailing in the no sail zone.
#         Wind angle is measured in degrees ccw from north.
#         Returns the updated waypoints.
#         """
#         pass
    
