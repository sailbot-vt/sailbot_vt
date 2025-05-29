#!usr/bin/python3
# TODO: Implement a graceful way to handle simulation termination with the control scripts

import random
import numpy as np, cv2
import gymnasium as gym
import navpy
import pandas as pd
import math
import os

from gymnasium.wrappers.time_limit import TimeLimit
from gymnasium.wrappers.record_video import RecordVideo
from sailboat_gym import CV2DRenderer, Observation

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Vector3, Twist
from sensor_msgs.msg import NavSatFix
from sailbot_msgs.msg import WaypointList



sim_time = 0



# get the real life data for the real life wind generation function
cur_folder_path = os.path.dirname(os.path.realpath(__file__))
wind_df = pd.read_csv(cur_folder_path + "/wind_data_claytor1.csv")
wind_data = []
for index, row in wind_df.iterrows():
    wind_data.append((row["wind_speed_m/s"], row["wind_direction_degrees_ccw_east"]))



# -------------------------------------------------------------------------------------
# Wind Generation Functions That Could Be Used to Generate Winds in the Simulation
# -------------------------------------------------------------------------------------

# randomize the wind direction and use real life data to be more like real life
def generate_wind_real_life_data(_): 
    index = math.floor(sim_time/250) % len(wind_data)
    WIND_SPEED = min(wind_data[index][0], 2.5)
    WIND_DIRECTION = wind_data[index][1]
 
    random_angle = WIND_DIRECTION + random.gauss(sigma=np.deg2rad(5), mu=0)
    generated_wind = np.array([np.cos(random_angle), np.sin(random_angle)]) * (min(WIND_SPEED, 2.5) + random.gauss(sigma=0.3, mu=0))
    return generated_wind



def generate_wind_down(_):
    # this is the wind direction measured as (what seems like) counter clockwise from true east
    return np.array([np.cos(np.deg2rad(-90)), np.sin(np.deg2rad(-90))])



# randomize the wind direction to be more like real life
def generate_wind_randomized(_): 
    random_angle = np.deg2rad(-90) + random.gauss(sigma=np.deg2rad(5), mu=0)
    generated_wind = np.array([np.cos(random_angle), np.sin(random_angle)]) * (1 + random.gauss(sigma=0.3, mu=0))
    return generated_wind

def generate_wind(_): 
    # wind_direction = 0.4 * np.sin(0 * np.deg2rad(sim_time)) + WIND_DIRECTION
    # return np.array([np.cos(WIND_DIRECTION) + random.random()*0.2, np.sin(WIND_DIRECTION) + random.random()*0.2]) * WIND_SPEED
    return np.array([np.cos(np.deg2rad(-90)) + random.random()*0., np.sin(np.deg2rad(-90)) + random.random()*0.])


def generate_wind_upwind(_):
    if sim_time >= 150:
        return np.array([np.cos(np.deg2rad(180)), np.sin(np.deg2rad(180))])
    return np.array([np.cos(0), np.sin(0)])










class SimulationNode(Node):

    def __init__(self):
        global sim_time

        super().__init__("simulation")
        
        sensor_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # NOTE: All units are in standard SI units and angle is measured in degrees
        self.desired_route_listener = self.create_subscription(WaypointList, '/desired_route', self.desired_route_callback, 10)
        
        self.position_publisher = self.create_publisher(msg_type=NavSatFix, topic="/position", qos_profile=sensor_qos_profile)
        self.velocity_publisher = self.create_publisher(msg_type=Twist, topic="/velocity", qos_profile=sensor_qos_profile)
        self.heading_publisher = self.create_publisher(msg_type=Float32, topic="/heading", qos_profile=sensor_qos_profile)
        self.apparent_wind_vector_publisher = self.create_publisher(msg_type=Vector3, topic="/apparent_wind_vector", qos_profile=sensor_qos_profile)
        
        self.rudder_angle_listener = self.create_subscription(msg_type=Float32, topic="/actions/rudder_angle", callback=self.rudder_angle_callback, qos_profile=sensor_qos_profile)
        self.sail_angle_listener = self.create_subscription(msg_type=Float32, topic="/actions/sail_angle", callback=self.sail_angle_callback, qos_profile=sensor_qos_profile)
        
        self.termination_listener = self.create_subscription(msg_type=Bool, topic="/should_terminate", callback=self.should_terminate_callback, qos_profile=10)
        
        self.env = gym.make('SailboatLSAEnv-v0',
            renderer=CV2DRenderer(),
            wind_generator_fn=generate_wind_real_life_data, 
            video_speed=20,
            map_scale=0.1,
            keep_sim_alive=True
        )
        self.env.NB_STEPS_PER_SECONDS = 1000
        
        self.episode_length = self.env.NB_STEPS_PER_SECONDS * 60 * 400000000
        sim_time = 0

        self.env = TimeLimit(self.env, max_episode_steps=self.episode_length)
        # self.env = RecordVideo(self.env, video_folder='./output/videos/')

        obs, info = self.env.reset(seed=10)
        
        # self.display_image(self.env.render())
        self.publish_observation_data(obs)
        
        self.desired_rudder_angle, self.desired_sail_angle = None, None
        self.apparent_wind_vector = Vector3(x=1.)
        self.true_wind_vector = Vector3(x=1.)
        self.route = None



    def __del__(self):
        self.env.close()
        rclpy.shutdown()



    def desired_route_callback(self, waypoint_list: WaypointList):
        self.route = [(waypoint.longitude, waypoint.latitude) for waypoint in waypoint_list.waypoints]
        
        
    def rudder_angle_callback(self, msg: Float32):
        self.desired_rudder_angle = np.array(msg.data)

        if self.desired_sail_angle != None:
            self.step_simulation()
            self.desired_rudder_angle, self.desired_sail_angle = None, None

    def sail_angle_callback(self, msg: Float32):
        
        _, true_wind_angle = self.cartesian_vector_to_polar(self.true_wind_vector.x, self.true_wind_vector.y)
        sail_dir_fix = -1 if 0 < true_wind_angle < 180 else 1
        self.desired_sail_angle = np.array(sail_dir_fix * msg.data)

        if self.desired_rudder_angle != None:
            self.step_simulation()
            self.desired_rudder_angle, self.desired_sail_angle = None, None
            
            
    def should_terminate_callback(self, msg: Bool):
        if msg.data == False: return
        
        self.env.close()
        rclpy.shutdown()
        
        
    def publish_observation_data(self, obs: Observation):

        # converts the position from local ned to longitude, latitude, altitude
        position = Vector3(x=obs["p_boat"][0].item(), y=obs["p_boat"][1].item(), z=-obs["p_boat"][2].item())
        gps_position = NavSatFix()
        
        if position.x == 0. and position.y == 0.:
            latitude, longitude = 0., 0.
        else:
            # navpy ned2lla throws a nan whenever x=0 and y=0 so we need to catch that
            latitude, longitude, _ = navpy.ned2lla([position.y, position.x, position.z], lat_ref=0., lon_ref=0., alt_ref=0)
        
        gps_position.latitude = latitude
        gps_position.longitude = longitude
    
    
        # saves the current velocity vector
        if np.isnan(obs["dt_p_boat"][0]) or np.isnan(obs["dt_p_boat"][1]) or np.isnan(obs["dt_p_boat"][2]):
            print("WARNING: VELOCITY IS NAN")
            boat_linear_velocity_vector = Vector3()
        else:
            boat_linear_velocity_vector = Vector3(x=obs["dt_p_boat"][0].item(), y=obs["dt_p_boat"][1].item(), z=obs["dt_p_boat"][2].item())
            
        boat_velocity = Twist(linear=boat_linear_velocity_vector)
        
        roll, pitch, yaw = obs["theta_boat"]
        
        
        # standard heading calculations from pitch, yaw, and roll
        heading_vector = np.array([np.cos(yaw) * np.cos(pitch), np.sin(yaw) * np.cos(pitch), np.sin(pitch)])
        heading_vector = Vector3(x = heading_vector[0].item(), y = heading_vector[1].item(), z = heading_vector[2].item())
        
        _, heading_angle = self.cartesian_vector_to_polar(heading_vector.x, heading_vector.y)
        heading_angle = Float32(data=heading_angle)

        # calculates the magnitude and direction of the wind velocity both true and apparent
        # Approximately 7:30 was most useful for me: https://www.youtube.com/watch?v=ndL1FcTRPwU&t=472s&ab_channel=BasicCruisingwithOwen
        # https://en.wikipedia.org/wiki/Apparent_wind#/media/File:DiagramApparentWind.png 
        # Apparent Wind = True Wind - Velocity
        # Global refers to being measured ccw from true east and not being measured from atop the boat
        # always remember that true wind and apparent wind are measured ccw from the centerline of the boat, 
        # while the global true wind is measured ccw from true east

        true_wind_speed, global_wind_angle = self.cartesian_vector_to_polar(obs["wind"][0].item(), obs["wind"][1].item())

        true_wind_angle = global_wind_angle - heading_angle.data
        
        self.get_logger().info(f"true wind: {true_wind_angle}")
        self.get_logger().info(f"velocity: {boat_linear_velocity_vector}")
        self.get_logger().info(f"position: {position}")
        self.get_logger().info("")
        
        self.true_wind_vector = Vector3(x= (true_wind_speed * np.cos(np.deg2rad(true_wind_angle))), y= (true_wind_speed * np.sin(np.deg2rad(true_wind_angle))))
        self.apparent_wind_vector = Vector3(x= (self.true_wind_vector.x - boat_linear_velocity_vector.x), y= (self.true_wind_vector.y - boat_linear_velocity_vector.y)) 
        
        self.position_publisher.publish(gps_position)
        self.velocity_publisher.publish(boat_velocity)
        self.heading_publisher.publish(heading_angle)
        self.apparent_wind_vector_publisher.publish(self.apparent_wind_vector)



    def step_simulation(self):
        global sim_time

        assert self.desired_rudder_angle != None
        assert self.desired_sail_angle != None
        
        action = {"theta_rudder": np.deg2rad(self.desired_rudder_angle), "theta_sail": np.deg2rad(self.desired_sail_angle)}
        obs, reward, terminated, truncated, info = self.env.step(action)

        sim_time += 1
        
        # self.display_image(self.env.render())

        self.publish_observation_data(obs)




    def cartesian_vector_to_polar(self, x, y):
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
    
    def display_image(self, img):
        cv2.imshow("Simulation Real Time", img)
        cv2.waitKey(1)
    

def main():

    rclpy.init()
    simulation_node = SimulationNode()
    rclpy.spin(simulation_node)
    
    simulation_node.destroy_node()
    rclpy.shutdown()
    
    


if __name__ == "__main__": 
    main()