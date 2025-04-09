"""
TO DO:
launch gps node and pyvesc node
automatically install pyvesc //worry about it later (saturday)
search up how to launch microROS node // know for later (wednesday)
push to new remote branch (the local one i made today)

"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='autopilot',
            executable='motorboat_autopilot',
            name='motorboat_autopilot',

            respawn=True,
            respawn_delay=2.0,
            # output="log"
        ),

        Node(
            package='autopilot',
            executable='telemetry',
            name='telemetry',

            respawn=True,
            respawn_delay=2.0,
            # output="log"
        ),

        Node(
            package='sensors',
            executable='gps',
            name='gps_publisher',

            respawn=True,
            respawn_delay=2.0,
            # output="log"
        ),

        Node(
            package='sensors',
            executable='rc',
            name='rc_publisher',

            respawn=True,
            respawn_delay=2.0,
            # output="log"
        ),


        Node(
            package='vesc',
            executable='vesc',
            name='vesc_communication',

            respawn=True,
            respawn_delay=0.01,
            # output="log"
        ),

        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            arguments=['serial', '--dev', "/dev/ttyACM0", "-b", "115200"]
        ),
    ]
)