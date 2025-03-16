from autopilot.autopilot import SailbotAutopilot
from autopilot.utils import *


import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.node import Node
from sailbot_msgs.msg import WaypointList, RCData, VESCControlData
from std_msgs.msg import Float32, String, Int32, Bool 
from geometry_msgs.msg import Vector3, Twist
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import Empty

import json, yaml
import os, time

class MotorboatAutopilotNode(Node):
    def __init__(self):
        super().__init__("motorboat_autopilot")

        cur_folder_path = os.path.dirname(os.path.realpath(__file__))
        with open(cur_folder_path + "/default_parameters.yaml", 'r') as stream:
            self.parameters: dict = yaml.safe_load(stream)
            
        self.sailbot_autopilot = SailbotAutopilot(parameters=self.parameters, logger=self.get_logger())


        # Initialize ros2 subscriptions, publishers, and timers
        self.autopilot_refresh_timer = self.create_timer(1 / self.parameters['autopilot_refresh_rate'], self.update_ros_topics)
        
        sensor_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.autopilot_parameters_listener = self.create_subscription(String, '/autopilot_parameters', callback=self.autopilot_parameters_callback, qos_profile=10)
            
        self.rc_listener = self.create_subscription(msg_type=RCData, topic="/rc_data", callback=self.rc_data_callback, qos_profile=sensor_qos_profile)
        
        self.autopilot_mode_publisher = self.create_publisher(String, "/autopilot_mode", qos_profile=sensor_qos_profile)
        self.autopilot_mode_listener = self.create_subscription(String, '/autopilot_mode', callback=self.autopilot_mode_callback, qos_profile=sensor_qos_profile)

        self.waypoints_list_listener = self.create_subscription(WaypointList, '/waypoints_list', self.waypoints_list_callback, 10)
        self.cur_waypoint_index_publisher = self.create_publisher(Int32, '/cur_waypoint_index', 10)
        
        self.position_listener = self.create_subscription(msg_type=NavSatFix, topic="/position", callback=self.position_callback, qos_profile=sensor_qos_profile)
        self.velocity_listener = self.create_subscription(msg_type=Twist, topic="/velocity", callback=self.velocity_callback, qos_profile=sensor_qos_profile)
        self.heading_listener = self.create_subscription(msg_type=Float32, topic="/heading", callback=self.heading_callback, qos_profile=sensor_qos_profile)

        self.full_autonomy_maneuver_publisher = self.create_publisher(msg_type=String, topic='/full_autonomy_maneuver', qos_profile=sensor_qos_profile)
        self.desired_heading_publisher = self.create_publisher(Float32, '/desired_heading', qos_profile=10)
        self.rudder_angle_publisher = self.create_publisher(msg_type=Float32, topic="/actions/rudder_angle", qos_profile=sensor_qos_profile)
        self.is_propeller_motor_enabled_publisher = self.create_publisher(Bool, "/is_propeller_motor_enabled", qos_profile=10)
        # self.zero_encoder_client = self.create_client(Empty, '/zero_rudder_encoder')
        self.zero_encoder_publisher = self.create_publisher(msg_type=Bool, topic="zero_rudder_encoder", qos_profile=10)


        self.motor_control_struct_publisher = self.create_publisher(msg_type= VESCControlData, topic="/motor_control_struct", qos_profile=sensor_qos_profile)
        """
        self.propellor_motor_rpm_value_publisher = self.create_publisher(msg_type=Float32, topic="vesc/rpm_value", qos_profile = sensor_qos_profile)
        self.propellor_motor_current_value_publisher = self.create_publisher(msg_type=Float32, topic="vesc/current_value", qos_profile = sensor_qos_profile)
        self.propellor_motor_duty_cycle_value_publisher = self.create_publisher(msg_type=Float32, topic="vesc/duty_cycle_value", qos_profile = sensor_qos_profile)
        """
        self.max_rpm = 10000

        #default values
        self.position = Position(longitude=0., latitude=0.)
        self.velocity = np.array([0., 0.])
        self.speed = 0.
        self.heading = 0.
        self.rudder_angle = 0.
        
        self.autopilot_mode = MotorboatAutopilotMode.Full_RC
        self.is_propeller_motor_enabled = False
        self.should_zero_encoder = False
        self.encoder_has_been_zeroed = False
        #self.full_autonomy_maneuver = Maneuvers.AUTOPILOT_DISABLED
        self.heading_to_hold = 0.
        
        self.last_rc_data_time = 0.    # used to check whether we have disconnected from the remote controller
        self.joystick_left_x = 0.
        self.joystick_left_y = 0.
        self.joystick_right_x = 0.
        self.joystick_right_y = 0.
        
        self.button_a = 0
        self.toggle_b = 0
        self.toggle_c = 0
        self.button_d = 0
        self.toggle_e = 0
        self.toggle_f = 0

        self.propeller_motor_control_mode = MotorboatControls.RPM        
        
    def rc_data_callback(self, joystick_msg: RCData):
        self.last_rc_data_time = time.time()
        
        if joystick_msg.toggle_f == 1 and self.toggle_f != 1:   # this means we have entered hold heading mode, so keep track of the current heading
            self.heading_to_hold = self.heading     
        
        if self.button_d == False and joystick_msg.button_d == True:
            self.should_zero_encoder = True
            self.encoder_has_been_zeroed = False
        elif self.encoder_has_been_zeroed:
            self.should_zero_encoder = False
        

        self.joystick_left_x = joystick_msg.joystick_left_x
        self.joystick_left_y = joystick_msg.joystick_left_y
        self.joystick_right_x = joystick_msg.joystick_right_x
        self.joystick_right_y = joystick_msg.joystick_right_y
        
        self.button_a = joystick_msg.button_a
        self.toggle_b = joystick_msg.toggle_b
        self.toggle_c = joystick_msg.toggle_c
        self.button_d = joystick_msg.button_d
        self.toggle_e = joystick_msg.toggle_e
        self.toggle_f = joystick_msg.toggle_f
        
        # kill switch
        if self.toggle_b == 0:
            self.is_propeller_motor_enabled = True
        elif self.toggle_b == 1:
            self.is_propeller_motor_enabled = False
            
        if self.toggle_b == 2:
            self.autopilot_mode = MotorboatAutopilotMode.Disabled
            
        # full autonomy
        elif self.toggle_f == 2:
            self.autopilot_mode = MotorboatAutopilotMode.Waypoint_Mission
            
        # hold heading to the direction that we started this mode in. the sail is controlled via RC
        elif self.toggle_f == 1:
            self.autopilot_mode = MotorboatAutopilotMode.Hold_Heading

        # remote controlled
        elif self.toggle_f == 0:
            self.autopilot_mode = MotorboatAutopilotMode.Full_RC
        # should not happen
        else:
            print("WARNING: INCORRECT COMBINATION OF RC SWITCHES USED")

        if self.toggle_c == 0:
            self.propeller_motor_control_mode = MotorboatControls.RPM
        elif self.toggle_c == 1:
            self.propeller_motor_control_mode = MotorboatControls.DUTY_CYCLE
        elif self.toggle_c == 2:
            self.propeller_motor_control_mode = MotorboatControls.CURRENT
        
        # if self.toggle_e == 2:
        #     self.is_propeller_motor_enabled = True
        # else:
        #     self.is_propeller_motor_enabled = False


    def autopilot_mode_callback(self, mode: String):
        if MotorboatAutopilotMode[mode.data] == MotorboatAutopilotMode.Hold_Heading and self.autopilot_mode != MotorboatAutopilotMode.Hold_Heading:
            self.heading_to_hold = self.heading

        self.autopilot_mode = MotorboatAutopilotMode[mode.data]


    def autopilot_parameters_callback(self, new_parameters: String):
        """
        Receives a serialized json (as a string) of parameters and sets them as constants.
        Any constant can be set as long as they are in the json
        """
        new_parameters_json: dict = json.loads(new_parameters.data)
        for new_parameter_name, new_parameter_value in new_parameters_json.items():
            if new_parameter_name not in self.parameters.keys():
                print("WARNING: Attempted to set an autopilot parameter that the autopilot doesn't know")
                print("If you would like to make a new autopilot parameter, please edit default_parameters.yaml")
                continue
            
            self.parameters[new_parameter_name] = new_parameter_value
        
        
        # special cases to handle since they do not update automatically
        if "autopilot_refresh_rate" in new_parameters_json.keys():
            self.destroy_timer(self.autopilot_refresh_timer)
            self.autopilot_refresh_timer = self.create_timer(1 / self.parameters['autopilot_refresh_rate'], self.update_ros_topics)

    def waypoints_list_callback(self, waypoints: WaypointList):
        """
        convert the list of Nav Sat Fix objects (ros2) to a list of Position objects, which are a custom datatype that has some useful helper methods.
        The Position object should be simpler to do calculations with
        """
        if len(waypoints.waypoints) == 0: return
        
        self.sailbot_autopilot.reset()
        
        gps_positions: list[NavSatFix] = waypoints.waypoints
        waypoints_list = []
        
        for gps_position in gps_positions:
            waypoints_list.append(Position(gps_position.longitude, gps_position.latitude))
            
        self.sailbot_autopilot.waypoints = waypoints_list
        self.sailbot_autopilot.cur_waypoint_index = 0
        

    def position_callback(self, position: NavSatFix):
        self.position = Position(longitude=position.longitude, latitude=position.latitude)

    def velocity_callback(self, velocity: Twist):
        self.velocity = np.array([velocity.linear.x, velocity.linear.y])
        self.speed = np.sqrt(velocity.linear.x**2 + velocity.linear.y**2)

    def heading_callback(self, heading: Float32):
        self.heading = heading.data
        


    def update_ros_topics(self):
        desired_rudder_angle = self.step()

        self.cur_waypoint_index_publisher.publish(Int32(data=self.sailbot_autopilot.cur_waypoint_index))
            
        self.autopilot_mode_publisher.publish(String(data=self.autopilot_mode.name))
        if self.autopilot_mode == MotorboatAutopilotMode.Waypoint_Mission:
            self.full_autonomy_maneuver_publisher.publish(String(data=self.sailbot_autopilot.current_state.name))
        else:
            self.full_autonomy_maneuver_publisher.publish(String(data="N/A"))

        
        if self.autopilot_mode == MotorboatAutopilotMode.Hold_Heading:
            self.desired_heading_publisher.publish(Float32(data=float(self.heading_to_hold)))
            
        elif self.autopilot_mode == MotorboatAutopilotMode.Waypoint_Mission and self.sailbot_autopilot.waypoints != None:
            current_waypoint = self.sailbot_autopilot.waypoints[self.sailbot_autopilot.cur_waypoint_index]
            bearing_to_waypoint = get_bearing(self.position, current_waypoint) #TODO make it so that this is the actual heading the autopilot is trying to follow (this is different when tacking)
            self.desired_heading_publisher.publish(Float32(data=float(bearing_to_waypoint)))
            
        else:
            self.desired_heading_publisher.publish(Float32(data=0.))
            
            
        if desired_rudder_angle != None:
            self.rudder_angle_publisher.publish(Float32(data=float(desired_rudder_angle)))


        # if we have not received data from the remote control for 3 seconds
        has_rc_disconnected = False
        # self.get_logger().info(f"{time.time() - self.last_rc_data_time}")
        if (time.time() - self.last_rc_data_time >= 3):
            has_rc_disconnected = True
            
            self.motor_control_struct_publisher.publish(
                VESCControlData(
                    control_type_for_vesc = "rpm", desired_vesc_current = 0.0, 
                    desired_vesc_rpm = 0.0, desired_vesc_duty_cycle = 0.0
                )
            )
            
            
        if self.autopilot_mode == MotorboatAutopilotMode.Full_RC and not has_rc_disconnected:
            if self.propeller_motor_control_mode == MotorboatControls.RPM:
                rpm_value = 100.0 * self.joystick_left_y #min -1e5 max 1e5
                self.motor_control_struct_publisher.publish(
                    VESCControlData(
                        control_type_for_vesc = "rpm", desired_vesc_current = 0.0, 
                        desired_vesc_rpm = rpm_value, desired_vesc_duty_cycle = 0.0
                    )
                )
                # self.get_logger().info(f'RPM {rpm_value}')
            
            elif self.propeller_motor_control_mode == MotorboatControls.DUTY_CYCLE:
                duty_cycle_value = self.joystick_left_y  #min 0 max 100
                self.motor_control_struct_publisher.publish(
                    VESCControlData(
                        control_type_for_vesc = "duty_value", desired_vesc_current = duty_cycle_value, 
                        desired_vesc_rpm = 0.0, desired_vesc_duty_cycle = 0.0
                    )
                )
                # self.get_logger().info(f'DUTY CYCLE {duty_cycle_value}')
            
            elif self.propeller_motor_control_mode == MotorboatControls.CURRENT:
                current_value = self.joystick_left_y
                self.motor_control_struct_publisher.publish(
                    VESCControlData(
                        control_type_for_vesc = "current", desired_vesc_current = current_value, 
                        desired_vesc_rpm = 0.0, desired_vesc_duty_cycle = 0.0
                    )
                )
                # self.get_logger().info(f'CURRENT {current_value}')
            
        
        self.is_propeller_motor_enabled_publisher.publish(Bool(data=self.is_propeller_motor_enabled))



        if self.should_zero_encoder:
            self.zero_encoder_publisher.publish(Bool(data=self.should_zero_encoder))
            self.encoder_has_been_zeroed = True


    def step(self):
        """
        Computes the best sail and rudder angles for the given mode and state
        
        Returns (tuple): (sail_angle, rudder_angle)
            sail angle or rudder angle are None if the autopilot doesn't have authority over them 
        """

        if self.autopilot_mode == MotorboatAutopilotMode.Waypoint_Mission and self.sailbot_autopilot.waypoints != None:
            _, rudder_angle = self.sailbot_autopilot.run_waypoint_mission_step(self.position, self.velocity, self.heading, self.apparent_wind_vector)
            
        elif self.autopilot_mode == MotorboatAutopilotMode.Hold_Heading:
            rudder_angle = self.sailbot_autopilot.get_optimal_rudder_angle(self.heading, self.heading_to_hold)
            
        elif self.autopilot_mode == MotorboatAutopilotMode.Full_RC:
            _, rudder_angle = self.sailbot_autopilot.run_rc_control(self.joystick_left_y, self.joystick_right_x)
            
        else: 
            return None
        
            
        return rudder_angle

def main():
    
    rclpy.init()
    autopilot_node = MotorboatAutopilotNode()
    rclpy.spin(autopilot_node)
        