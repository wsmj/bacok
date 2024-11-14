from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bacok',
            executable='offboard_dubins',
            name='dubins',
            output='screen',
            remappings=[
                ('/fmu/out/vehicle_attitude', '/px4_1/fmu/out/vehicle_attitude'),
                ('/fmu/out/vehicle_gps_position', '/px4_1/fmu/out/vehicle_gps_position')

            ]
        ),
        Node(
            package='gece_es',
            executable='offboard_dubins',
            name='team1',
            output='screen',
            remappings=[
                ('/fmu/out/vehicle_attitude', '/px4_1/fmu/out/vehicle_attitude'),
                ('/fmu/out/vehicle_gps_position', '/px4_1/fmu/out/vehicle_gps_position')

            ]
        ),
        Node(
            package='gece_es',
            executable='process_telemetry',
            name='team2',
            output='screen',
            remappings=[
                ('/fmu/out/vehicle_attitude', '/px4_2/fmu/out/vehicle_attitude'),
                ('/fmu/out/vehicle_gps_position', '/px4_2/fmu/out/vehicle_gps_position')

            ]
        )
    ])

