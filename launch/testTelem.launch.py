from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bacok',
            executable='processTelem',
            name='telem',
            output='screen',
            remappings=[
                ('/fmu/out/vehicle_attitude', '/px4_1/fmu/out/vehicle_attitude'),
                ('/fmu/out/vehicle_gps_position', '/px4_1/fmu/out/vehicle_gps_position')

            ]
        ),
        # ExecuteProcess(
        #     cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        #     output='screen'
        # )
    ])

