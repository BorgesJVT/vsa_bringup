from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import ament_index_python

def generate_launch_description():
    scenario_file: str = "sparus2_tank.scn"

    source_path: str = ament_index_python.get_package_prefix('vsa_stonefish').replace("install", "src")
    sim_data_path: str = source_path + "/data"
    scenario_desc: str = source_path + "/scenarios/" + scenario_file

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('stonefish_ros2'),
                    'launch',
                    'stonefish_simulator.launch.py'
                ])
            ]),
            launch_arguments = {
                'simulation_data' : sim_data_path, # PathJoinSubstitution([FindPackageShare('vsa_stonefish'), 'data']),
                'scenario_desc' : scenario_desc, # PathJoinSubstitution([FindPackageShare('vsa_stonefish'), 'scenarios', 'lauv_tank.scn']),
                'simulation_rate' : '300.0',
                'window_res_x' : '1200',
                'window_res_y' : '800',
                'rendering_quality' : 'high'
            }.items()
        )
    ])