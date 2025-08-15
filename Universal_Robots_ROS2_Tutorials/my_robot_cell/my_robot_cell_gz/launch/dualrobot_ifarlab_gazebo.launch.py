import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    GroupAction,
    TimerAction,  # <-- GECİKME İÇİN GEREKLİ IMPORT
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    # Launch argümanları
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    world_file_path = LaunchConfiguration("world_file").perform(context)
    if LaunchConfiguration("gazebo_gui").perform(context) == 'true':
        gz_args_string = f"-r -v 4 {world_file_path}"
    else:
        gz_args_string = f"-s -r -v 4 {world_file_path}"

    gz_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={
            "gz_args": gz_args_string,
            "on_exit_shutdown": "true",
        }.items(),
    )

    # Bridge konfigürasyonu
    gz_sim_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "--ros-args",
            "-p", f"config_file:={os.path.join(get_package_share_directory('my_robot_cell_gz'), 'bridge', 'bridgos.yaml')}"
        ],
        output="screen",
    )
    
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("my_robot_cell_description"), "rviz", "dualrobot_urdf.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(launch_rviz),
    )

    # =================================================================================================
    # --- ROBOT 1 (Global Namespace, Prefix Yok) ---
    # =================================================================================================

    robot1_controllers_file = PathJoinSubstitution(
        [FindPackageShare("my_robot_cell_gz"), "config", "ur_controllers.yaml"]
    )

    robot1_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
            PathJoinSubstitution([FindPackageShare("my_robot_cell_description"), "urdf", "my_robot_cell.urdf.xacro"]), " ",
            "safety_limits:=", safety_limits, " ",
            "safety_pos_margin:=", safety_pos_margin, " ",
            "safety_k_position:=", safety_k_position, " ",
            "name:=", "ur", " ",
            "ur_type:=", ur_type, " ",
            "prefix:=", '""', " ",
            "sim_ignition:=true", " ",
            "simulation_controllers:=", robot1_controllers_file, " ",
            "use_sim_time:=", use_sim_time,
        ]
    )
    robot1_description = {"robot_description": robot1_description_content}

    robot1_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": use_sim_time}, robot1_description],
    )

    robot1_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string", robot1_description_content,
            "-name", "robot1",
            "-allow_renaming", "true",
            "-x", "0.0",
            "-y", "0.0", 
            "-z", "0.0",
        ],
    )

    robot1_joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    robot1_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["scaled_joint_trajectory_controller", "-c", "/controller_manager"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    robot1_controllers_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot1_spawn_entity,
            on_exit=[
                robot1_joint_state_broadcaster_spawner,
                robot1_controller_spawner,
            ],
        )
    )

    robot1_moveit = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('real_robot_moveit_config'),
                        'launch',
                        'move_group.launch.py'
                    )
                ),
                launch_arguments={
                    "use_sim_time": "true",
                }.items(),
            )

    # =================================================================================================
    # --- ROBOT 2 ("sim" Namespace, "sim_" Prefix'i ile) ---
    # =================================================================================================

    robot2_controllers_file = PathJoinSubstitution(
        [FindPackageShare("my_robot_cell_gz"), "config", "simrobot_ur_controllers.yaml"]
    )

    robot2_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare("my_robot_cell_description"), "urdf", "simrobot_my_robot_cell.urdf.xacro"]), " ",
        "ur_type:=", ur_type, " ",
        "tf_prefix:=sim_", " ",
        "sim_ignition:=true", " ",
        "simulation_controllers:=", robot2_controllers_file, " ",
        "use_sim_time:=", use_sim_time,
    ])

    robot2_description = {"robot_description": robot2_description_content}

    robot2_sim_group = GroupAction(
        actions=[
            PushRosNamespace('sim'),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="both",
                parameters=[{"use_sim_time": use_sim_time}, robot2_description],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["sim_joint_state_broadcaster", "--controller-manager", "/sim/controller_manager"],
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["sim_scaled_joint_trajectory_controller", "--controller-manager", "/sim/controller_manager"],
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('sim_robot_moveit_config'),
                        'launch',
                        'move_group.launch.py'
                    )
                ),
                launch_arguments={"use_sim_time": "true"}.items(),
            ),
        ]
    )
    
    robot2_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string", robot2_description_content,
            "-name", "robot2_sim",
            "-allow_renaming", "true",
            "-x", "0.0",
            "-y", "0.0", 
            "-z", "0.0",
        ],
    )

    # =================================================================================
    # --- DEĞİŞİKLİK: Robot2'yi 5 saniye gecikmeyle başlat ---
    # =================================================================================
    # Robot1'in spawn işlemi bittikten sonra 5 saniye bekleyip Robot2'yi spawn ediyoruz.
    delayed_robot2_spawn_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot1_spawn_entity,
            on_exit=[
                TimerAction(
                    period=5.0,
                    actions=[robot2_spawn_entity],
                )
            ],
        )
    )

    # Gazebo'da Robot2 spawn edildikten sonra ilgili group'u (controller, moveit vs.) başlat
    robot2_group_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot2_spawn_entity,
            on_exit=[robot2_sim_group],
        )
    )

    # =================================================================================
    # --- BAŞLATILACAK DÜĞÜMLER LİSTESİ ---
    # =================================================================================
    nodes_to_start = [
        gz_launch_description,
        gz_sim_bridge,
        rviz_node,
        robot1_state_publisher_node,
        robot1_spawn_entity,
        robot1_controllers_event,
        robot1_moveit,
        delayed_robot2_spawn_event,  # <-- GECİKMELİ EVENT HANDLER KULLANILIYOR
        robot2_group_event,
    ]
    
    return nodes_to_start

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument("ur_type", default_value="ur10e"))
    declared_arguments.append(DeclareLaunchArgument("safety_limits", default_value="true"))
    declared_arguments.append(DeclareLaunchArgument("safety_pos_margin", default_value="0.15"))
    declared_arguments.append(DeclareLaunchArgument("safety_k_position", default_value="20"))
    declared_arguments.append(DeclareLaunchArgument("use_sim_time", default_value="true"))
    declared_arguments.append(DeclareLaunchArgument("launch_rviz", default_value="true"))
    declared_arguments.append(DeclareLaunchArgument("gazebo_gui", default_value="true"))
    declared_arguments.append(DeclareLaunchArgument(
            "world_file",
            default_value="empty.sdf"
    ))

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])