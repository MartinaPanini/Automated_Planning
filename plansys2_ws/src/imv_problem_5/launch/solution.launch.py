import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'imv_problem_5'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Percorso al PDDL (assicurati di aver copiato domain_4.pddl nella cartella pddl del package)
    domain_file = os.path.join(pkg_share, 'pddl', 'domain_4.pddl')

    # Include plansys2 launcher standard
    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
          'model_file': domain_file,
          }.items())

    # Lista dei nodi fake actions
    fake_actions = [
        'move',
        'pick_up_secure',
        'pick_up_fragile',
        'drop_secure',
        'drop_fragile',
        'load_pod',
        'unload_pod_secure',
        'unload_pod_fragile',
        'cool_down'
    ]

    ld = LaunchDescription()
    ld.add_action(plansys2_cmd)

    # Aggiungi ogni nodo al launcher
    for action in fake_actions:
        ld.add_action(Node(
            package=pkg_name,
            executable=action,
            name=action,
            output='screen',
            parameters=[]
        ))

    return ld