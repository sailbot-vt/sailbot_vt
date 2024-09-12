from launch import LaunchDescription
from launch_ros.actions import Node

# VERY MUCH IN DEVELOPMENT THIS IS JUST A TEMPLATE
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='autopilot',
            executable='autopilot',
            name='autopilot',

            respawn=True, 
            respawn_delay=2.0,
            output="screen"
        ),
        
        
        Node(
            package='autopilot',
            executable='telemetry',
            name='telemetry',

            respawn=True, 
            respawn_delay=2.0,
            output="log"
        ),
        
        Node(
            package='rc',
            executable='rc',
            name='rc',

            respawn=True, 
            respawn_delay=2.0,
            output="log"
        ),
        
        Node(
            package='gps',
            executable='gps',
            name='gps',

            respawn=True, 
            respawn_delay=2.0,
            output="log"
        ),
        
        Node(
            package='mcu',
            executable='mcu',
            name='mcu',

            respawn=True, 
            respawn_delay=2.0,
            output="log"
        ),
        
        Node(
            package='wind_sensor',
            executable='wind_sensor',
            name='wind_sensor',

            respawn=True, 
            respawn_delay=2.0,
            output="log"
        ),
        
        Node(
            package='compass',
            executable='compass',
            name='compass',

            respawn=True, 
            respawn_delay=2.0,
            output="log"
        ),
        
        Node(
            package='realsense2_camera',
            executable='rs_launch.py',
            name='camera',

            respawn=True, 
            respawn_delay=0.5,
            output="log"
        ),
        
        # ros2 launch realsense2_camera rs_launch.py enable_rgbd:=true enable_sync:=true align_depth.enable:=true enable_color:=true enable_depth:=true rgb_camera.color_profile:=1280x720x15
    ])