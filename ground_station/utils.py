import numpy as np
from typing import TypedDict
import navpy


def latlon_to_local(lat, lon, ref_lat, ref_lon):
    north, east, down = navpy.lla2ned(lat, lon, 0, ref_lat, ref_lon, 0)
    return np.array([north, east])


def rgba(color, alpha, background=np.array([255, 255, 255])):
    return tuple((1 - alpha) * background + alpha * np.array(color))

def angle_to_vec(angle):
    return np.array([np.cos(angle), np.sin(angle)])

def cartesian_vector_to_polar(x, y):
    """
        Converts a cartesian vector (x and y coordinates) to polar form (magnitude and direction).
        Outputs a tuple of magnitude and direction of the inputted vector
    """
    magnitude = np.sqrt(x**2 + y**2)
    direction = np.arctan2(y, x) # radians
    direction = direction * (180/np.pi)  # angle from -180 to 180 degrees
    direction = direction % 360  # angle from 0 to 360 degrees
    return magnitude, direction

def rotate_vector(vector: np.ndarray, angle: float):
    assert vector.shape == (2,)
    rot = np.array([
        [np.cos(angle), -np.sin(angle)],
        [np.sin(angle), np.cos(angle)]
    ])
    return rot @ vector
    
    
class State(TypedDict):
    p_boat: np.ndarray[3]
    dt_p_boat: np.ndarray[3]
    theta_boat: np.ndarray[3]
    dt_theta_boat: np.ndarray[3]
    theta_rudder: np.ndarray[1]
    dt_theta_rudder: np.ndarray[1]
    theta_sail: np.ndarray[1]
    dt_theta_sail: np.ndarray[1]
    apparent_wind: np.ndarray[2]
    true_wind: np.ndarray[2]
    water: np.ndarray[2]
    waypoints: np.ndarray
    cur_waypoint_index: int
    buoys: np.ndarray
    
    no_go_zone_size: float
    decision_zone_size: float
    
    
    