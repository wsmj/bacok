from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='ros_gz_image',
        #     executable='image_bridge',
        #     name='image_bridge_node',
        #     output='screen',
        #     arguments=['/camera']  
        # ),
        Node(
            package='bacok',
            executable='attitude_control',
            name='attitude_control',
            output='screen',
            remappings=[
                ('/fmu/in/offboard_control_mode', '/px4_1/fmu/in/offboard_control_mode'),
                ('/fmu/in/vehicle_command', '/px4_1/fmu/in/vehicle_command'),
                # ('/fmu/in/vehicle_attitude_setpoint', '/px4_1/fmu/in/vehicle_attitude_setpoint'),
                ('/fmu/in/vehicle_attitude_setpoint', '/px4_1/fmu/in/vehicle_attitude_setpoint'),
                ('/fmu/out/vehicle_gps_position', '/px4_1/fmu/out/vehicle_gps_position')

            ]
        )
    ])