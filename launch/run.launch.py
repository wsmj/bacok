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
            name='dubins',
            output='screen',
            remappings=[
                ('/fmu/in/offboard_control_mode', '/px4_1/fmu/in/offboard_control_mode'),
                ('/fmu/in/vehicle_command', '/px4_1/fmu/in/vehicle_command'),
                ('/fmu/in/vehicle_attitude_setpoint', '/px4_1/fmu/in/vehicle_attitude_setpoint'),
                # ('/fmu/out/vehicle_attitude', '/px4_1/fmu/out/vehicle_attitude'),
                ('/fmu/out/vehicle_gps_position', '/px4_1/fmu/out/vehicle_gps_position')

            ]
        )
        # ExecuteProcess(
        #     cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        #     output='screen'
        # )
        # ExecuteProcess(
        #     cmd=['./build/px4_sitl_default/bin/px4', '-i', '1'],
        #     output='screen',
        #     cwd=os.path.expanduser('~/Developer/PX4-Autopilot/'),
        #     env={
        #         'PX4_SYS_AUTOSTART': '4016',
        #         'PX4_SIM_MODEL': 'gz_rc_cessna_fpv',
        #         'PX4_GZ_WORLD': 'lawn'
        #     }
        # )
    ])

# PX4_SYS_AUTOSTART=4016 PX4_SIM_MODEL=gz_rc_cessna_fpv PX4_GZ_WORLD=lawn ./build/px4_sitl_default/bin/px4 -i 1