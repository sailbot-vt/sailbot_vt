"""
TO DO:
launch gps node and pyvesc node
automatically install pyvesc //worry about it later (saturday)
search up how to launch microROS node // know for later (wednesday)
push to new remote branch (the local one i made today)

"""

from launch import LaunchDescription
from launch_ros.actions import Node

# VERY MUCH IN DEVELOPMENT THIS IS JUST A TEMPLATE
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gps',
            executable='gps',
            name='gps_publisher',

            respawn=True, 
            respawn_delay=2.0,
            output="log"
        ),
        
        Node(
            package='jetsonVESC',
            executable='jetsonVESC',
            name='pyvesc_publisher',

            respawn=True, 
            respawn_delay=2.0,
            output="log"
        )
])