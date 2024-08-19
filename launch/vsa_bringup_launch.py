from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Include the rosbridge_server launch file
    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('rosbridge_server'),
            'launch/rosbridge_websocket_launch.xml'
        ]))
    )
 
     # Execute bash command 'ros2 run nucleus_driver'
    nucleus_node = ExecuteProcess(
        cmd=['ros2', 'run', 'nucleus_driver_ros2', 'nucleus_node'],
        output='screen'
    )
       
    # Execute bash command 'ros2 run nucleus_driver connect'
    nucleus_driver_node_connect = ExecuteProcess(
        cmd=['ros2', 'run', 'nucleus_driver_ros2', 'connect_tcp', '10.7.113.51', 'nortek'],
        output='screen'
    )
    
    # Execute bash command 'ros2 run nucleus_driver start'
    nucleus_driver_node_start = ExecuteProcess(
        cmd=['ros2', 'run', 'nucleus_driver_ros2', 'start'],
        output='screen'
    )
    
    # Include the duro_gps_driver launch file
    duro_gps_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('duro_gps_driver'),
            'launch/bringup_duro_gps.launch'
        ]))
    )

    # Include the primus_imu_driver launch file
    primus_imu_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('primus_imu_driver'),
            'launch/bringup_primus_imu.launch'
        ]))
    )
    
    # Record bag
    bag_recorder = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', \
             '/tf',                   \
             '/primus/imu',           \
             '/duro/gps',             \
             '/duro/heading',         \
             '/duro/n_sats',          \
             '/duro/current_pose',    \
             '/nucleus_node/ahrs_packets', \
             '/nucleus_node/altimeter_packets', \
             '/nucleus_node/bottom_track_packets', \
             '/nucleus_node/field_calibration_packets', \
             '/nucleus_node/imu_packets', \
             '/nucleus_node/water_track_packets'],
        output='screen'
    )

    return LaunchDescription([
        rosbridge_launch,
        
        TimerAction(period=1.0, actions=[
            nucleus_node,
        ]),

        TimerAction(period=6.0, actions=[
            nucleus_driver_node_connect,
        ]),
        
        TimerAction(period=10.0, actions=[
            nucleus_driver_node_start,
        ]),
                        
        TimerAction(period=2.0, actions=[
            duro_gps_launch,
        ]),
        
        TimerAction(period=3.0, actions=[
            primus_imu_launch,
        ]),
        
        TimerAction(period=15.0, actions=[
            bag_recorder,
        ]),
    ])
