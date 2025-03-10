import requests
import os, sys, time, datetime
import cv2
import geopy
import geopy.distance
import matplotlib.pyplot as plt

from renderer import CV2DRenderer
from utils import *

RUN_WITH_SAILOR_STANDARDS = True
DEGREE_SIGN = u'\N{DEGREE SIGN}'
TELEMETRY_SERVER_URL = 'http://18.191.164.84:8080/'

pid_data_file = None
telemetry_file = None
telemetry_start_time = time.time()

MAP_BOUNDS = [[-200, -200], [200, 200]]
# MAP_BOUNDS = [[-25, -50], [100, 75]]
# MAP_BOUNDS = [[-300, -300], [300, 300]]
# MAP_BOUNDS = [[-30, 0], [75, 30]]
#BUOYS =  [[42.845474, -70.977055], [42.846278, -70.976928], [42.8446333, -70.9771833]]


BUOYS = []
# [
#     # [42.8449667, -70.9772667],
#     # [42.8447667, -70.9771167],
#     # [42.8449167, -70.9761667],
#     # [42.8455167, -70.9765333]
# ]


def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')

def move_terminal_cursor(y, x):
    """
        Please look here as to how to interpret this abomination:
        https://stackoverflow.com/questions/54630766/how-can-move-terminal-cursor-in-python
    """
    print("\033[%d;%dH" % (y, x))


def hide_terminal_cursor():
    # if os.name == 'nt':
    #     ci = _CursorInfo()
    #     handle = ctypes.windll.kernel32.GetStdHandle(-11)
    #     ctypes.windll.kernel32.GetConsoleCursorInfo(handle, ctypes.byref(ci))
    #     ci.visible = False
    #     ctypes.windll.kernel32.SetConsoleCursorInfo(handle, ctypes.byref(ci))
    if os.name == 'posix':
        sys.stdout.write("\033[?25l")
        sys.stdout.flush()

def show_terminal_cursor():
    # if os.name == 'nt':
    #     ci = _CursorInfo()
    #     handle = ctypes.windll.kernel32.GetStdHandle(-11)
    #     ctypes.windll.kernel32.GetConsoleCursorInfo(handle, ctypes.byref(ci))
    #     ci.visible = True
    #     ctypes.windll.kernel32.SetConsoleCursorInfo(handle, ctypes.byref(ci))
    if os.name == 'posix':
        sys.stdout.write("\033[?25h")
        sys.stdout.flush()



def get_decision_zone_size(tack_distance, no_sail_zone_size, distance_to_waypoint):
    inner = (tack_distance/distance_to_waypoint) * np.sin(np.deg2rad(no_sail_zone_size/2))
    inner = np.clip(inner, -1, 1)
    return np.clip(np.rad2deg(np.arcsin(inner)), 0, no_sail_zone_size)

def get_distance_to_waypoint(cur_position, next_waypoint):
    if next_waypoint:
        return geopy.distance.geodesic(next_waypoint, cur_position).m
    else:
        return geopy.distance.Distance(0.).m
    



def request_boat_status() -> dict:
    """
    Should return a dictionary with the following keys:
        position as a latitude, longitude tuple
        state as a string
        speed as a float in m/s
        bearing in degrees
        heading in degrees
        true_wind_speed in m/s
        true_wind_angle in degrees
        apparent_wind_speed in m/s
        apparent_wind_angle in degrees
        sail_angle in degrees
        rudder_angle in degrees
        current_waypoint as a latitude, longitude tuple
        current_route as a list of latitude, longitude tuples
        
        TODO: document vesc data in here
    """
    boat_status = requests.get(TELEMETRY_SERVER_URL + "boat_status/get").json()
    return boat_status


def update_telemetry_text(boat_status: dict):
    global telemetry_file, telemetry_start_time
        
    # Convert to the units that the sailors are happy with
    heading_cw_north = (90 - boat_status["heading"]) % 360        # ccw from true east -> cw from true north
    bearing_cw_north = (90 - boat_status["bearing"]) % 360        # ccw from true east -> cw from true north
    apparent_wind_angle_cw_centerline_from = (180 - boat_status["apparent_wind_angle"]) % 360         # ccw centerline measuring the direction the wind is blowing towards -> cw centerline measuring the direction the wind is blowing from
    apparent_wind_speed_knots = 1.94384 * boat_status["apparent_wind_speed"]                          # m/s -> knots
    true_wind_angle_cw_centerline_from =  (180 - boat_status["true_wind_angle"]) % 360                # ccw centerline measuring the direction the wind is blowing towards -> cw centerline measuring the direction the wind is blowing from
    true_wind_speed_knots = 1.94384 * boat_status["true_wind_speed"]                                  # m/s -> knots
    boat_speed_knots = 1.94384 * boat_status["speed"]                               # m/s -> knots
    
    current_waypoint_index = boat_status["current_waypoint_index"]
    
    
    if boat_status["current_route"]:
        distance_to_next_waypoint = get_distance_to_waypoint(boat_status["position"], boat_status["current_route"][current_waypoint_index])
    else:
        distance_to_next_waypoint = get_distance_to_waypoint(boat_status["position"], None)

    
    if RUN_WITH_SAILOR_STANDARDS:
        speed_unit = "kts"
        heading = heading_cw_north
        bearing = bearing_cw_north
        apparent_wind_angle = apparent_wind_angle_cw_centerline_from
        apparent_wind_speed = apparent_wind_speed_knots
        true_wind_angle = true_wind_angle_cw_centerline_from
        true_wind_speed = true_wind_speed_knots
        boat_speed = boat_speed_knots
        
    else:
        speed_unit = "m/s"
        heading = boat_status["heading"]
        bearing = boat_status["bearing"]
        apparent_wind_angle = boat_status["apparent_wind_angle"]
        apparent_wind_speed = boat_status["apparent_wind_speed"]
        true_wind_angle  = boat_status["true_wind_angle"]
        true_wind_speed = boat_status["true_wind_speed"]
        boat_speed = boat_status["speed"]
        
    string_to_show += f"rpm: {boat_status['vesc_data_rpm']}"
    
    # Get Formatted Time
    time_since_startup = (time.time() - telemetry_start_time)
    time_since_startup = datetime.time(
        hour=int((time_since_startup // 3600) % 24), 
        minute=int(time_since_startup // 60) % 60, 
        second=int(time_since_startup) % 60, 
        microsecond=int(time_since_startup * 10**6) % 10**6
    )
    time_since_startup_str = time_since_startup.strftime('%H:%M:%S.{:02.0f}').format(time_since_startup.microsecond/10000.0)
    
    real_life_date_time = datetime.datetime.now()
    real_life_date_time_str = real_life_date_time.strftime('%m-%d-%Y %H:%M:%S.{:02.0f}').format(real_life_date_time.microsecond/10000.0)
    
    # Construct String to Display to Command Line
    string_to_show = ""
    string_to_show += f"Time Today: {real_life_date_time_str}                                                                                                  \n"
    string_to_show += f"Time Since Start Up: {time_since_startup_str}                                                                                          \n"
    string_to_show += f"GPS Latitude: {boat_status['position'][0]:.8f}, GPS Longitude: {boat_status['position'][1]:.8f}                                            \n"
    string_to_show += f"Autopilot Mode: {boat_status['state']}                                                                                                   \n"
    string_to_show += f"Fully Autonomous Maneuver: {boat_status['full_autonomy_maneuver']}                                                                           \n"
    string_to_show += f"Speed Over Ground: {boat_speed:.2f} {speed_unit}                                                                                       \n"
    string_to_show += f"Target Heading: {bearing:.2f}{DEGREE_SIGN}                                                                                             \n"
    string_to_show += f"Heading: {heading:.2f}{DEGREE_SIGN}                                                                                                    \n"
    string_to_show += f"True Wind Speed: {true_wind_speed:.2f} {speed_unit}, True Wind Angle {true_wind_angle:.2f}{DEGREE_SIGN}                                \n"
    string_to_show += f"Apparent Wind Speed: {apparent_wind_speed:.2f} {speed_unit}, Apparent Wind Angle: {apparent_wind_angle:.2f}{DEGREE_SIGN}               \n"
    string_to_show += f"Target Sail Angle: {boat_status['sail_angle']:.2f}{DEGREE_SIGN}                                                                          \n"
    string_to_show += f"Target Rudder Angle: {boat_status['rudder_angle']:.2f}{DEGREE_SIGN}                                                                      \n"
    string_to_show += f"Current Waypoint Index: {current_waypoint_index}                                                                                       \n"
    string_to_show += f"Distance to next waypoint: {distance_to_next_waypoint:.2f} meters                                                                        \n"
    string_to_show += "                                                                                                                                        \n"
    
    string_to_show += f"Parameters:                                                                                                                            \n"
    string_to_show += f"------------------------------------                                                                                                   \n"
    for param_name, param_value in boat_status["parameters"].items():
        string_to_show += f"{param_name}: {param_value}                                                 \n"
    
    string_to_show += "                                                                                                                                        \n"
    string_to_show += f"Current Route:                                                                                                                         \n"
    string_to_show += f"------------------------------------                                                                                                   \n"
    for index, waypoint in enumerate(boat_status["current_route"]):
        string_to_show += f"Waypoint {index} Latitude: {waypoint[0]:.8f}, Waypoint {index} Longitude: {waypoint[1]:.8f}                                        \n"
    
    
    trailing_white_space = ""
    # for i in range(2 - len(boat_status["current_route"])):
    #     trailing_white_space += "                                                                                                                                      \n"
    
    # Display String and Write to Telemetry File
    move_terminal_cursor(0, 0)
    print(string_to_show + trailing_white_space)
    
    telemetry_file.write(string_to_show)

    

def display_image(img):
    cv2.imshow("Ground Station GUI", img)
    cv2.waitKey(1)
        
def update_telemetry_gui(renderer: CV2DRenderer, telemetry: dict):
    local_y, local_x = 0, 0
    sail_dir_fix = -1 if 0 < telemetry["true_wind_angle"] < 180 else 1
    
    absolute_true_wind_angle = telemetry["true_wind_angle"] + telemetry["heading"]
    absolute_apparent_wind_angle = telemetry["apparent_wind_angle"] + telemetry["heading"]
    
        
    tack_distance = telemetry["parameters"]["tack_distance"]
    no_sail_zone_size = telemetry["parameters"]["no_sail_zone_size"]
    cur_waypoint = telemetry["current_route"][telemetry["current_waypoint_index"]]
    distance_to_next_waypoint = get_distance_to_waypoint(telemetry["position"], cur_waypoint)
    decision_zone_size = get_decision_zone_size(tack_distance, no_sail_zone_size, distance_to_next_waypoint)
    
    # TWS = telemetry["true_wind_speed"]
    # TWA = telemetry["true_wind_angle"]
    # AWS = telemetry["apparent_wind_speed"]
    # AWA = telemetry["apparent_wind_angle"]
    
    # true_wind_vector = np.array([TWS * np.cos(np.deg2rad(TWA-90)), TWS * np.sin(np.deg2rad(TWA-90))])
    # apparent_wind_vector = np.array([AWS * np.cos(np.deg2rad(AWA-90)), AWS * np.sin(np.deg2rad(AWA-90))])
    # # velocity_vector = (true_wind_vector - apparent_wind_vector)
    velocity_vector = telemetry["velocity_vector"]
    
    gui_state = State()
    gui_state["p_boat"] = np.array([local_x, local_y, 0])
    gui_state["dt_p_boat"] = np.array(velocity_vector)
    gui_state["theta_boat"] = np.array([0, 0, np.deg2rad(telemetry["heading"])])
    gui_state["dt_theta_boat"] = np.array([0, 0, 0])
    gui_state["theta_rudder"] = np.array([0, 0, 0])
    gui_state["dt_theta_rudder"] = np.array([0, 0, 0])
    gui_state["theta_sail"] = np.array([sail_dir_fix * np.deg2rad(telemetry["sail_angle"]), 0, 0])
    gui_state["dt_theta_sail"] = np.array([0, 0, 0])
    gui_state["apparent_wind"] = np.array([telemetry["apparent_wind_speed"] * np.cos(np.deg2rad(absolute_apparent_wind_angle)), telemetry["apparent_wind_speed"] * np.sin(np.deg2rad(absolute_apparent_wind_angle))])
    gui_state["true_wind"] = np.array([telemetry["true_wind_speed"] * np.cos(np.deg2rad(absolute_true_wind_angle)), telemetry["true_wind_speed"] * np.sin(np.deg2rad(absolute_true_wind_angle))])
    gui_state["water"] = np.array([0, 0])
    gui_state["buoys"] = np.array(BUOYS)
    gui_state["cur_waypoint_index"] = telemetry["current_waypoint_index"]
    gui_state["no_go_zone_size"] = 100
    gui_state["decision_zone_size"] = decision_zone_size
    
    waypoints = []
    for waypoint in telemetry["current_route"]:
        local_y, local_x, _ = navpy.lla2ned(waypoint[0], waypoint[1], 0, telemetry["position"][0], telemetry["position"][1], 0)
        local_x = np.clip(local_x, MAP_BOUNDS[0][0], MAP_BOUNDS[1][0])
        local_y = np.clip(local_y, MAP_BOUNDS[0][1], MAP_BOUNDS[1][1])
        
        waypoints.append((local_x, local_y))
    gui_state["waypoints"] = np.array(waypoints)
    
    
    buoys = []
    for buoy in BUOYS:
        local_y, local_x, _ = navpy.lla2ned(buoy[0], buoy[1], 0, telemetry["position"][0], telemetry["position"][1], 0)
        local_x = np.clip(local_x, MAP_BOUNDS[0][0], MAP_BOUNDS[1][0])
        local_y = np.clip(local_y, MAP_BOUNDS[0][1], MAP_BOUNDS[1][1])
        
        buoys.append((local_x, local_y))
    gui_state["buoys"] = np.array(buoys)

    display_image(renderer.render(gui_state))


def update_heading_pid_graph(telemetry):
    global pid_data_file

    heading = telemetry["heading"]
    desired_heading = telemetry["bearing"]
    current_time = time.time() - telemetry_start_time
    
    pid_data_file.write(f"{current_time},{heading},{desired_heading}\n")




def main():
    global telemetry_file, pid_data_file, renderer
    # if os.name == 'nt':
    #     import msvcrt
    #     import ctypes

    #     class _CursorInfo(ctypes.Structure):
    #         _fields_ = [("size", ctypes.c_int),
    #                     ("visible", ctypes.c_byte)]
    
    clear_screen()
    hide_terminal_cursor()
    boat_status = request_boat_status()
    
    map_bounds = np.array(MAP_BOUNDS)
    renderer = CV2DRenderer()
    renderer.setup(map_bounds)
    
    telemetry_file = open("./telemetry_log.txt", "a")
    pid_data_file = open("./pid_data.csv", "w")
    
    pid_data_file.write("time,heading,desired_heading\n")
    
    plt.ion()
    
    
    while True:
        boat_status = request_boat_status()
        update_telemetry_text(boat_status)
        update_telemetry_gui(renderer, boat_status)
        update_heading_pid_graph(boat_status)
         
        # time.sleep(0.05)
    
    
if __name__ == "__main__": 
    try:
        main()
    finally:
        clear_screen()
        show_terminal_cursor()
        telemetry_file.close()
        
