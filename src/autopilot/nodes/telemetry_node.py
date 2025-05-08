#!usr/bin/python3

from autopilot.utils import *

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Float32, Bool, String, Int32
from geometry_msgs.msg import Vector3, Twist
from sensor_msgs.msg import NavSatFix, Image
from sailbot_msgs.msg import WaypointList, VESCData
from cv_bridge import CvBridge
import base64

import numpy as np
import array, time, json, requests
import cv2

TELEMETRY_SERVER_URL = 'http://18.191.164.84:8080/'

# TODO: MOST IMPORTANT MAKE THE IP AN ARGUMENT INTO THE ROS NODE AND DOCKER CONTAINER

# TODO: Update the website info at a fixed rate
# TODO: add current state of the board and the FSM

class TelemetryNode(Node):

    def __init__(self):
        super().__init__("telemetry")

        # NOTE: All units are in standard SI units and angle is measured in degrees
        self.create_timer(0.01, self.update_everything)

        sensor_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.cv_bridge  = CvBridge()
        
        # self.autopilot_parameter_listener = self.create_subscription(String, '/autopilot_parameters', callback=self.autopilot_parameters_callback, qos_profile=10)
        self.autopilot_parameters_publisher = self.create_publisher(msg_type=String, topic='/autopilot_parameters', qos_profile=10)
        self.vesc_data_listener = self.create_subscription(VESCData, '/vesc_data', self.vesc_data_callback, sensor_qos_profile)

        self.desired_heading_listener = self.create_subscription(Float32, '/desired_heading', self.desired_heading_callback, 10)
        self.waypoints_list_listener = self.create_subscription(WaypointList, '/waypoints_list', self.waypoints_list_callback, 10)
        self.waypoints_list_publisher = self.create_publisher(WaypointList, '/waypoints_list', qos_profile=10)
        self.cur_waypoint_index_listener = self.create_subscription(Int32, '/cur_waypoint_index', self.cur_waypoint_index_callback, 10)
        
        self.full_autonomy_maneuver_listener = self.create_subscription(msg_type=String, topic="/full_autonomy_maneuver", callback=self.full_autonomy_maneuver_callback, qos_profile=sensor_qos_profile)
        self.autopilot_mode_listener = self.create_subscription(msg_type=String, topic="/autopilot_mode", callback=self.autopilot_mode_callback, qos_profile=sensor_qos_profile)
        
        self.position_listener = self.create_subscription(msg_type=NavSatFix, topic="/position", callback=self.position_callback, qos_profile=sensor_qos_profile)
        self.velocity_listener = self.create_subscription(msg_type=Twist, topic="/velocity", callback=self.velocity_callback, qos_profile=sensor_qos_profile)
        self.heading_listener = self.create_subscription(msg_type=Float32, topic="/heading", callback=self.heading_callback, qos_profile=sensor_qos_profile)
        self.apparent_wind_vector_listener = self.create_subscription(msg_type=Vector3, topic="/apparent_wind_vector", callback=self.apparent_wind_vector_callback, qos_profile=sensor_qos_profile)
        self.camera_rgb_image_listener = self.create_subscription(msg_type=Image, topic="/camera/camera/color/image_raw", callback=self.camera_rgb_image_callback, qos_profile=sensor_qos_profile)
        
        self.sail_angle_listener = self.create_subscription(msg_type=Float32, topic="/actions/sail_angle", callback=self.sail_angle_callback, qos_profile=sensor_qos_profile)
        self.rudder_angle_listener = self.create_subscription(msg_type=Float32, topic="/actions/rudder_angle", callback=self.rudder_angle_callback, qos_profile=sensor_qos_profile)

        # Default values in case these are never sent through ROS
        # If these values aren't changing then the ros node thats supposed to be sending these values may not be working correctly
        self.waypoints_list: list[NavSatFix] = []
        self.cur_waypoint_index = 0
        self.position = NavSatFix(latitude=0., longitude=0.)
        
        self.autopilot_mode = ""
        self.full_autonomy_maneuver = ""
        self.autopilot_parameters = {}

        self.velocity_vector = np.array([0., 0.])
        self.speed = 0.
        self.heading = 0.
        self.desired_heading = 0.

        self.apparent_wind_vector = np.array([0., 0.])
        self.apparent_wind_speed = 0.
        self.apparent_wind_angle = 0.
        
        self.base64_encoded_current_rgb_image = None

        self.sail_angle = 0.
        self.rudder_angle = 0.
        
        self.time = 0 # keeps track of how many callbacks on the website telemetry were made
        
        self.vesc_data_rpm = 0
        self.vesc_data_duty_cycle = 0
        self.vesc_data_amp_hours = 0
        self.vesc_data_amp_hours_charged = 0
        self.vesc_data_current_to_vesc = 0
        self.vesc_data_voltage_to_motor = 0
        self.vesc_data_voltage_to_vesc = 0
        self.vesc_data_wattage_to_motor = 0
        self.vesc_data_time_since_vesc_startup_in_ms = 0
        self.vesc_data_motor_temperature = 0
        self.vesc_data_vesc_temperature = 0

    
    def desired_heading_callback(self, desired_heading: Float32):
        self.desired_heading = desired_heading.data

    def waypoints_list_callback(self, waypointsList: WaypointList):
        self.waypoints_list = waypointsList.waypoints
    
    def vesc_data_callback(self, vesc_data: VESCData):
        # self.get_logger().info("testing")
        self.vesc_data_rpm = vesc_data.rpm
        self.vesc_data_duty_cycle = vesc_data.duty_cycle
        self.vesc_data_amp_hours = vesc_data.amp_hours
        self.vesc_data_amp_hours_charged = vesc_data.amp_hours_charged
        self.vesc_data_current_to_vesc = vesc_data.current_to_vesc
        self.vesc_data_voltage_to_motor = vesc_data.voltage_to_motor
        self.vesc_data_voltage_to_vesc = vesc_data.voltage_to_vesc
        self.vesc_data_wattage_to_motor = vesc_data.wattage_to_motor
        self.vesc_data_time_since_vesc_startup_in_ms = vesc_data.time_since_vesc_startup_in_ms
        self.vesc_data_motor_temperature = vesc_data.motor_temperature
        self.vesc_data_vesc_temperature = vesc_data.vesc_temperature

    def cur_waypoint_index_callback(self, cur_waypoint_index: Int32):
        self.cur_waypoint_index = cur_waypoint_index.data
    
    def full_autonomy_maneuver_callback(self, full_autonomy_maneuver: String):
        self.full_autonomy_maneuver = full_autonomy_maneuver.data
        
    def autopilot_mode_callback(self, autopilot_mode: String):
        self.autopilot_mode = autopilot_mode.data
        
    def position_callback(self, position: NavSatFix):
        self.position = position

    def velocity_callback(self, velocity_vector: Twist):
        self.velocity_vector = np.array([velocity_vector.linear.x, velocity_vector.linear.y])
        self.speed = np.sqrt(velocity_vector.linear.x**2 + velocity_vector.linear.y**2)

    def heading_callback(self, heading: Float32):
        self.heading = heading.data

    def apparent_wind_vector_callback(self, apparent_wind_vector: Vector3):
        self.apparent_wind_vector = np.array([apparent_wind_vector.x, apparent_wind_vector.y])

        self.apparent_wind_speed, self.apparent_wind_angle = cartesian_vector_to_polar(apparent_wind_vector.x, apparent_wind_vector.y)
        
    def camera_rgb_image_callback(self, camera_rgb_image: Image):
        """refer to this stack overflow post: https://stackoverflow.com/questions/40928205/python-opencv-image-to-byte-string-for-json-transfer"""
        rgb_image_cv = self.cv_bridge.imgmsg_to_cv2(camera_rgb_image, "rgb8")
        rgb_image_cv = rgb_image_cv[80:1200,40:680] # crop the image to 640,640
        retval, buffer = cv2.imencode(".jpg", rgb_image_cv)
        
        # swap red and blue channels for correction
        red = rgb_image_cv[:,:,2].copy()
        blue = rgb_image_cv[:,:,0].copy()
        rgb_image_cv[:,:,0] = red
        rgb_image_cv[:,:,2] = blue
        
        cv2.imwrite('test.jpg', rgb_image_cv)

        self.base64_encoded_current_rgb_image = base64.b64encode(buffer).decode()


    
    def sail_angle_callback(self, sail_angle: Float32):
        self.sail_angle = sail_angle.data

    def rudder_angle_callback(self, rudder_angle: Float32):
        self.rudder_angle = rudder_angle.data
    
    def should_terminate_callback(self, msg: Bool):
        if msg.data == False: return
        rclpy.shutdown()
    
    
    
    
    def update_everything(self):
        self.update_boat_status()
        self.update_autopilot_parameters_from_telemetry()
        self.update_waypoints_from_telemetry()
    
    
    
    def update_boat_status(self):
        """
        FORMAT:
            position (Lat Lon tuple); current state; speed; bearing; heading; wind speed; wind direction; sail angle; rudder angle; cur_waypoint; the entire waypoint list for the current route
            each of them are separated as a semicolon
            tuples are denoted as: element1, element2, ... such as for latitude/ longitude
            different entries are denoted as: speed; heading; apparent_wind_speed
        """
        
        print(f"boat velocity: {self.velocity_vector}")
        print(f"AW vector: {self.apparent_wind_vector}")
        true_wind_vector = self.apparent_wind_vector + self.velocity_vector
        # print(f"TW vector: {true_wind_vector}")
        self.true_wind_speed, self.true_wind_angle = cartesian_vector_to_polar(true_wind_vector[0], true_wind_vector[1])
        # print(f"wind angle: {(self.true_wind_angle + self.heading) % 360}")
        # self.get_logger().info(f"{self.vesc_data_rpm}")

        telemetry_dict = {
            "position": (self.position.latitude, self.position.longitude), 
            "state": self.autopilot_mode,
            "full_autonomy_maneuver": self.full_autonomy_maneuver,
            "speed": self.speed,
            "velocity_vector": (self.velocity_vector[0], self.velocity_vector[1]),
            "bearing": self.desired_heading, "heading": self.heading,
            "true_wind_speed": self.true_wind_speed, "true_wind_angle": self.true_wind_angle,
            "apparent_wind_speed": self.apparent_wind_speed, "apparent_wind_angle": self.apparent_wind_angle,
            "sail_angle": self.sail_angle, "rudder_angle": self.rudder_angle,
            "current_waypoint_index": self.cur_waypoint_index,
            "current_route": [(waypoint.latitude, waypoint.longitude) for waypoint in self.waypoints_list],
            "parameters": self.autopilot_parameters,
            "current_camera_image": self.base64_encoded_current_rgb_image,

            "vesc_data_rpm": self.vesc_data_rpm,
            "vesc_data_duty_cycle": self.vesc_data_duty_cycle,
            "vesc_data_amp_hours": self.vesc_data_amp_hours,
            "vesc_data_amp_hours_charged": self.vesc_data_amp_hours_charged,
            "vesc_data_current_to_vesc": self.vesc_data_current_to_vesc,
            "vesc_data_voltage_to_motor": self.vesc_data_voltage_to_motor,
            "vesc_data_voltage_to_vesc": self.vesc_data_voltage_to_vesc, 
            "vesc_data_wattage_to_motor": self.vesc_data_wattage_to_motor,
            "vesc_data_time_since_vesc_startup_in_ms": self.vesc_data_time_since_vesc_startup_in_ms,
            "vesc_data_motor_temperature": self.vesc_data_motor_temperature,
            "vesc_data_vesc_temperature": self.vesc_data_vesc_temperature
        }

        requests.post(url=TELEMETRY_SERVER_URL + "/boat_status/set", json={"value": telemetry_dict})
        self.time+=1



    
    def get_raw_response(self, route):
        try:
            return requests.get(TELEMETRY_SERVER_URL + route, timeout=10).json()
        except Exception as e:
            print(e)
            time.sleep(1)
            print("retrying connection")
            return self.get_raw_response(route)
    
    def update_waypoints_from_telemetry(self):
        waypoints_list = self.get_raw_response("/waypoints/get")
        # print(f"waypoints_list: {waypoints_list}")
        
        if not waypoints_list: 
            return
        else: 
            # TODO eventually fix this so that we don't have to depend on deleting the waypoints
            # deleting the waypoints makes it so that we can't access it in the future from this route which is not entirely desirable    
            requests.post(TELEMETRY_SERVER_URL + "/waypoints/delete")
    
        # parse waypoints        
        waypoints_nav_sat_fix_list = []
        for waypoint in waypoints_list:
            if not waypoint: continue
            
            lat, lon = waypoint
            
            try: float(lat) and float(lon)
            except: raise Exception("Waypoints from Server Were Improperly Formatted")
            
            waypoints_nav_sat_fix_list.append(NavSatFix(latitude=float(lat), longitude=float(lon)))
            

        waypoint_list = WaypointList(waypoints=waypoints_nav_sat_fix_list)
        self.waypoints_list_publisher.publish(waypoint_list)
        
        
    def update_autopilot_parameters_from_telemetry(self):
        autopilot_parameters = self.get_raw_response("/autopilot_parameters/get")
        print(f"autopilot_parameters: {autopilot_parameters}")

        if not autopilot_parameters: 
            return
        else: 
            # TODO eventually fix this so that we don't have to depend on deleting the waypoints
            # deleting the waypoints makes it so that we can't access it in the future from this route which is not entirely desirable
            requests.post(TELEMETRY_SERVER_URL + "/autopilot_parameters/delete")
     
        # parse parameters
        serialized_parameters_string = String(data=json.dumps(autopilot_parameters))  # we already removed waypoints so all that are left are the parameters
        self.autopilot_parameters_publisher.publish(serialized_parameters_string)
        
        self.autopilot_parameters = autopilot_parameters


def main():


    rclpy.init()
    telem_node = TelemetryNode()
    rclpy.spin(telem_node)
    


if __name__ == "__main__": main()