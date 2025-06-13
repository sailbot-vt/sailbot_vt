from autopilot.autopilot import SailbotAutopilot
from autopilot.utils import *


import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.node import Node
from sailbot_msgs.msg import WaypointList, RCData
from std_msgs.msg import Float32, String, Int32, Bool
from geometry_msgs.msg import Vector3, Twist
from sensor_msgs.msg import NavSatFix

import json, yaml, os, time




class SailboatAutopilotNode(Node):
    """
    Handles communications between the autopilot and all of the other nodes/ topics through ros2
    The autopilot takes in a bunch of sensor data and waypoints (list of gps positions that represent waypoints) and attempts to traverses through the waypoints by continuously publishing to the sail angle and rudder angle topics.
        
    NOTE: All units are in standard SI units and angles are generally measured in degrees unless otherwise specified
    """
    
    def __init__(self):
        super().__init__("sailboat_autopilot")

        current_folder_path = os.path.dirname(os.path.realpath(__file__))
        with open(current_folder_path + "/default_parameters.yaml", 'r') as stream:
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
        
        self.rc_data_listener = self.create_subscription(msg_type=RCData, topic="/rc_data", callback=self.rc_data_callback, qos_profile=sensor_qos_profile)
        
        self.autopilot_mode_publisher = self.create_publisher(String, "/autopilot_mode", qos_profile=sensor_qos_profile)
        self.autopilot_mode_listener = self.create_subscription(String, '/autopilot_mode', callback=self.autopilot_mode_callback, qos_profile=sensor_qos_profile)
    
        self.waypoints_list_listener = self.create_subscription(WaypointList, '/waypoints_list', self.waypoints_list_callback, 10)
        self.current_waypoint_index_publisher = self.create_publisher(Int32, '/current_waypoint_index', 10)
        
        self.position_listener = self.create_subscription(msg_type=NavSatFix, topic="/position", callback=self.position_callback, qos_profile=sensor_qos_profile)
        self.velocity_listener = self.create_subscription(msg_type=Twist, topic="/velocity", callback=self.velocity_callback, qos_profile=sensor_qos_profile)
        self.heading_listener = self.create_subscription(msg_type=Float32, topic="/heading", callback=self.heading_callback, qos_profile=sensor_qos_profile)

        self.apparent_wind_vector_listener = self.create_subscription(msg_type=Vector3, topic="/apparent_wind_vector", callback=self.apparent_wind_vector_callback, qos_profile=sensor_qos_profile)
        
        
        self.full_autonomy_maneuver_publisher = self.create_publisher(msg_type=String, topic='/full_autonomy_maneuver', qos_profile=sensor_qos_profile)
        self.desired_heading_publisher = self.create_publisher(Float32, '/desired_heading', qos_profile=10)
        self.sail_angle_publisher = self.create_publisher(msg_type=Float32, topic="/actions/sail_angle", qos_profile=sensor_qos_profile)
        self.rudder_angle_publisher = self.create_publisher(msg_type=Float32, topic="/actions/rudder_angle", qos_profile=sensor_qos_profile)
       
        self.zero_rudder_encoder_publisher = self.create_publisher(msg_type=Bool, topic="/zero_rudder_encoder", qos_profile=10) 
        self.zero_winch_encoder_publisher = self.create_publisher(msg_type=Bool, topic="/zero_winch_encoder", qos_profile=10) 

        # Default values
        self.position = Position(longitude=0., latitude=0.)
        self.velocity = np.array([0., 0.])
        self.speed = 0.
        self.heading = 0.
        self.apparent_wind_vector = np.array([0., 0.])
        self.apparent_wind_angle = 0.
        self.sail_angle = 0.
        self.rudder_angle = 0.
        
        self.should_zero_rudder_encoder = False
        self.rudder_encoder_has_been_zeroed = False
        
        self.should_zero_winch_encoder = False
        self.winch_encoder_has_been_zeroed = False
        
        self.autopilot_mode = SailboatAutopilotMode.Waypoint_Mission    # Should this be by default: SailboatAutopilotMode.Full_RC
        self.heading_to_hold = 0.
     
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
        
        
        
        # Send default parameters to the telemetry server so that the groundstation can see what the default parameters are
        self.autopilot_parameters_publisher = self.create_publisher(String, '/autopilot_parameters', qos_profile=10)
        self.autopilot_parameters_publisher.publish(String(data = json.dumps(self.parameters)))
        
        del self.autopilot_parameters_publisher
        
        
        
    def rc_data_callback(self, joystick_msg: RCData):
        
        # self.get_logger().info(f"got to rc data callback")
        if joystick_msg.toggle_f == 1 and self.toggle_f != 1:   # this means we have entered hold heading mode, so keep track of the current heading
            self.heading_to_hold = self.heading     
        
        if self.button_d == False and joystick_msg.button_d == True:
            self.should_zero_rudder_encoder = True
            self.rudder_encoder_has_been_zeroed = False
        elif self.rudder_encoder_has_been_zeroed:
            self.should_zero_rudder_encoder = False
            
        if self.button_a == False and joystick_msg.button_a == True:
            self.should_zero_winch_encoder = True
            self.winch_encoder_has_been_zeroed = False
        elif self.winch_encoder_has_been_zeroed:
            self.should_zero_winch_encoder = False
            
            
            
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
        if self.toggle_b != 0:
            self.autopilot_mode = SailboatAutopilotMode.Disabled
            
        # full autonomy
        elif self.toggle_f == 2:
            self.autopilot_mode = SailboatAutopilotMode.Waypoint_Mission
            
        # hold heading to the direction that we started this mode in. the sail is controlled via RC
        elif self.toggle_f == 1 and self.toggle_c == 0:
            self.autopilot_mode = SailboatAutopilotMode.Hold_Heading
            
        # choose the best sail angle based on the lookup table. the rudder is controlled via RC
        elif self.toggle_f == 1 and self.toggle_c == 1:
            self.autopilot_mode = SailboatAutopilotMode.Hold_Best_Sail

        # hold heading and best sail
        elif self.toggle_f == 1 and self.toggle_c == 2:
            self.autopilot_mode = SailboatAutopilotMode.Hold_Heading_And_Best_Sail

        # remote controlled
        elif self.toggle_f == 0:
            self.autopilot_mode = SailboatAutopilotMode.Full_RC
        
        # this should never happen
        else:
            print("WARNING: INCORRECT COMBINATION OF RC SWITCHES USED")
            
        # self.get_logger().info(f"finished rc data callback")


    def autopilot_mode_callback(self, mode: String):
        if SailboatAutopilotMode[mode.data] == SailboatAutopilotMode.Hold_Heading and self.autopilot_mode != SailboatAutopilotMode.Hold_Heading:
            self.heading_to_hold = self.heading
            
        if SailboatAutopilotMode[mode.data] == SailboatAutopilotMode.Hold_Heading_And_Best_Sail and self.autopilot_mode != SailboatAutopilotMode.Hold_Heading_And_Best_Sail:
            self.heading_to_hold = self.heading
        
        self.autopilot_mode = SailboatAutopilotMode[mode.data]
        

        
        
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
        Convert the list of Nav Sat Fix objects (ros2) to a list of Position objects, which are a custom datatype that has some useful helper methods.
        The Position object should be simpler to do calculations with, so we would rather deal with them. There are many helper functios in utils.py for using Position objects
        """
        if len(waypoints.waypoints) == 0: return
        
        self.sailbot_autopilot.reset()
        
        gps_positions: list[NavSatFix] = waypoints.waypoints
        waypoints_list = []
        
        for gps_position in gps_positions:
            waypoints_list.append(Position(gps_position.longitude, gps_position.latitude))
            
        self.sailbot_autopilot.waypoints = waypoints_list
        self.sailbot_autopilot.current_waypoint_index = 0
        

    def position_callback(self, position: NavSatFix):
        self.position = Position(longitude=position.longitude, latitude=position.latitude)

    def velocity_callback(self, velocity: Twist):
        self.velocity = np.array([velocity.linear.x, velocity.linear.y])
        self.speed = np.sqrt(velocity.linear.x**2 + velocity.linear.y**2)

    def heading_callback(self, heading: Float32):
        self.heading = heading.data
    
    def apparent_wind_vector_callback(self, apparent_wind_vector: Vector3):
        self.apparent_wind_vector = np.array([apparent_wind_vector.x, apparent_wind_vector.y])
        _, self.apparent_wind_angle = cartesian_vector_to_polar(apparent_wind_vector.x, apparent_wind_vector.y)






    def step(self):
        """
        Computes the best sail and rudder angles for the given mode and state
        
        Returns (tuple): (sail_angle, rudder_angle)
            sail angle or rudder angle are None if the autopilot doesn't have authority over them 
        """

        
        if self.autopilot_mode == SailboatAutopilotMode.Waypoint_Mission and self.sailbot_autopilot.waypoints != None:
            sail_angle, rudder_angle = self.sailbot_autopilot.run_waypoint_mission_step(self.position, self.velocity, self.heading, self.apparent_wind_vector)
        
        elif self.autopilot_mode == SailboatAutopilotMode.Hold_Best_Sail:
            sail_angle = self.sailbot_autopilot.get_optimal_sail_angle(self.apparent_wind_angle)
            _, rudder_angle = self.sailbot_autopilot.run_rc_control(self.joystick_left_y, self.joystick_right_x)
            
        elif self.autopilot_mode == SailboatAutopilotMode.Hold_Heading:
            rudder_angle = self.sailbot_autopilot.get_optimal_rudder_angle(self.heading, self.heading_to_hold)
            sail_angle, _ = self.sailbot_autopilot.run_rc_control(self.joystick_left_y, self.joystick_right_x)
            
        elif self.autopilot_mode == SailboatAutopilotMode.Hold_Heading_And_Best_Sail:
            rudder_angle = self.sailbot_autopilot.get_optimal_rudder_angle(self.heading, self.heading_to_hold)
            sail_angle = self.sailbot_autopilot.get_optimal_sail_angle(self.apparent_wind_angle)
            
        elif self.autopilot_mode == SailboatAutopilotMode.Full_RC:
            sail_angle, rudder_angle = self.sailbot_autopilot.run_rc_control(self.joystick_left_y, self.joystick_right_x)
            
        else: 
            return None, None
        
            
        return rudder_angle, sail_angle


    
    def update_ros_topics(self):
        """
        This is the main function that is called constantely by the timer.
        
        Updates the sail_angle and rudder_angle topics based on the output of stepping in the autopilot controller
        
        Each call to this function takes around 2 milliseconds as of 5/26/2025 (aka this is not a super important place to find optimizations since it doesn't take that much time from a cpu core)
        """        
        
        # self.get_logger().info(f"got to update ros topics")
        desired_rudder_angle, desired_sail_angle = self.step()
        # self.get_logger().info(f"finished step")
        
        self.current_waypoint_index_publisher.publish(Int32(data=self.sailbot_autopilot.current_waypoint_index))
        
        
        # Publish the autonomy maneuever (aka whether we are currently CW tacking, CCW tacking, or normal sailing)
        self.autopilot_mode_publisher.publish(String(data=self.autopilot_mode.name))
        if self.autopilot_mode == SailboatAutopilotMode.Waypoint_Mission:
            self.full_autonomy_maneuver_publisher.publish(String(data=self.sailbot_autopilot.current_state.name))
            
        else:
            self.full_autonomy_maneuver_publisher.publish(String(data="N/A"))
    
    
    
    
        # Publish the desired heading
        if self.autopilot_mode == SailboatAutopilotMode.Hold_Heading or self.autopilot_mode == SailboatAutopilotMode.Hold_Heading_And_Best_Sail:
            self.desired_heading_publisher.publish(Float32(data=float(self.heading_to_hold)))
            
        elif self.autopilot_mode == SailboatAutopilotMode.Waypoint_Mission and self.sailbot_autopilot.waypoints != None:
            current_waypoint = self.sailbot_autopilot.waypoints[self.sailbot_autopilot.current_waypoint_index]
            
            # TODO make it so that the bearing is the actual heading the autopilot is trying to follow (this is different when tacking)
            # when tacking, the boat is not trying to head straight towards the waypoint, but rather, it is travelling on a tacking line
            bearing_to_waypoint = get_bearing(self.position, current_waypoint)
            self.desired_heading_publisher.publish(Float32(data=float(bearing_to_waypoint)))
            
        else:
            self.desired_heading_publisher.publish(Float32(data=0.))
            
            
            
        # Finally, ensure that we tell the motor driver what we want the rudder angle and the sail angle to do through ros
        if desired_rudder_angle != None:
            self.get_logger().info(f"desired rudder angle: {desired_rudder_angle}")
            self.rudder_angle_publisher.publish(Float32(data=float(desired_rudder_angle))) # the negative is a correction for how to actually turn the boat
            
        if desired_sail_angle != None:
            self.get_logger().info(f"desired sail angle: {desired_sail_angle}")
            self.sail_angle_publisher.publish(Float32(data=float(desired_sail_angle)))
            
            
            
        if self.should_zero_rudder_encoder:
            self.zero_rudder_encoder_publisher.publish(Bool(data=self.should_zero_rudder_encoder))
            self.rudder_encoder_has_been_zeroed = True

        if self.should_zero_winch_encoder:
            self.zero_winch_encoder_publisher.publish(Bool(data=self.should_zero_winch_encoder))
            self.winch_encoder_has_been_zeroed = True

        # self.get_logger().info(f"finished publishing")




def main():
    
    rclpy.init()
    sailboat_autopilot_node = SailboatAutopilotNode()
    rclpy.spin(sailboat_autopilot_node)

    sailboat_autopilot_node.destroy_node()
    rclpy.shutdown()




if __name__ == "__main__": 
    main()