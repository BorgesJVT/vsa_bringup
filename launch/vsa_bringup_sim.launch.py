from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
 
    # Include the vsa_stonefish launch file
    vsa_stonefish_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('vsa_stonefish'),
            'launch/lauv_open_sea.launch.py'
        ]))
    )
    
    # Execute bash command 'ros2 run imc_ros2_bridge vehicle_node'
    imc_ros_bridge_vehicle_node = ExecuteProcess(
            cmd=[
                'ros2', 'run', 'imc_ros2_bridge', 'vehicle_node',
                '--ros-args',
                '-p', 'bridge_address:=127.0.0.1',
                '-p', 'neptus_address:=127.0.0.1',
                '-p', 'sys_name:=VSA'
            ],
            # output='screen'
        )
    
    # Execute bash command 'ros2 run imc_ros2_bridge vehicle_supervisor_node'
    imc_ros_bridge_vehicle_supervisor_node = ExecuteProcess(
        cmd=['ros2', 'run', 'imc_ros2_bridge', 'vehicle_supervisor_node'],
        output='screen'
    )
    
    # Execute bash command 'ros2 run imc_ros2_bridge monitors_node'
    imc_ros_bridge_monitors_node = ExecuteProcess(
        cmd=['ros2', 'run', 'imc_ros2_bridge', 'monitors_node'],
        output='screen'
    )
    
    # Execute bash command 'simulate navigation'
    # sim_nav_node = ExecuteProcess(
    #     cmd=[
    #         "ros2", "topic", "pub", "-r", "20", "/estimated_state", 
    #         "neptus_msgs/msg/EstimatedState", 
    #         "{lat: 0.71881385, lon: -0.15195186, phi: 0.0, theta: 0.0, psi: 0.0}"
    #     ],
    #     # output="screen"
    # )
    
    # Execute bash command 'ros2 run vsa_guidance vsa_guidance_node'
    odometry_to_estimated_state_node = ExecuteProcess(
        cmd=['ros2', 'run', 'imc_ros2_bridge', 'odometry_to_estimated_state_node'],
        output='screen'
    )
    
    # Execute bash command 'ros2 run guidance guidance_node'
    guidance_node = ExecuteProcess(
        cmd=['ros2', 'run', 'guidance', 'guidance_node'],
        output='screen'
    )
    
    # Execute bash command 'ros2 run navigator'
    # navigator_node = ExecuteProcess(
    #     cmd=['ros2', 'run', 'navigator', 'navigator_node'],
    #     output='screen'
    # )
    
    # Record bag
    # bag_recorder = ExecuteProcess(
    #     cmd=['ros2', 'bag', 'record', \
    #          '/tf',                   \
    #          '/primus/imu',           \
    #          '/duro/gps',             \
    #          '/duro/heading',         \
    #          '/vehicle_state', \
    #          '/vehicle_state_navsatfix', \
    #          '/gps_in_meters'],
    #     output='screen'
    # )

    # bag_recorder = ExecuteProcess(
    #     cmd=['ros2', 'bag', 'record', '-a'],
    #     output='screen'
    # )

    return LaunchDescription([
        vsa_stonefish_launch,
        
        TimerAction(period=2.0, actions=[
            imc_ros_bridge_vehicle_node,
        ]),

        TimerAction(period=3.0, actions=[
            imc_ros_bridge_vehicle_supervisor_node,
        ]),
        
        TimerAction(period=3.0, actions=[
            imc_ros_bridge_monitors_node,
        ]),
        
        # TimerAction(period=3.0, actions=[
        #     sim_nav_node,
        # ]),
        
        TimerAction(period=3.0, actions=[
            odometry_to_estimated_state_node,
        ]),
        
        TimerAction(period=10.0, actions=[
            guidance_node,
        ]),
    ])
