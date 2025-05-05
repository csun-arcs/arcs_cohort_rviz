import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import ReplaceString


def generate_launch_description():
    # Defaults
    default_log_level = "INFO"

    # Declare launch arguments
    declare_namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description="Namespace under which to bring up nodes, topics, etc.",
    )
    declare_prefix_arg = DeclareLaunchArgument(
        "prefix",
        default_value="",
        description=(
            "A prefix for the names of joints, links, etc. in the robot model). "
            "E.g. 'base_link' will become 'cohort1_base_link' if prefix "
            "is set to 'cohort1'."
        ),
    )
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(
            get_package_share_directory('arcs_cohort_rviz'),
            'rviz',
            'cohort_default.rviz'
        ),
        description='Path to the RViz config file with <NAMESPACE> and <PREFIX> placeholders'
    )
    declare_use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation time"
    )
    declare_log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value=default_log_level,
        description="Set the log level for nodes.",
    )

    # Launch configurations
    namespace = LaunchConfiguration("namespace")
    prefix = LaunchConfiguration("prefix")
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")

    # Log info
    log_info = LogInfo(msg=['RViz bringup launching with namespace: ', namespace, ', prefix: ', prefix])

    # Use PushRosNamespace to apply the namespace to all nodes below
    push_namespace = PushRosNamespace(namespace=namespace)

    # Perform substitutions of <NAMESPACE> and <PREFIX> in the config file
    substituted_config = ReplaceString(
        source_file=rviz_config,
        replacements={
            '<NAMESPACE>': PythonExpression(["'/", namespace, "/' if '", namespace, "' else '/'"]),
            '<PREFIX>': prefix
        }
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=["-d", substituted_config, "--ros-args", "--log-level", log_level],
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
            ('/goal_pose', 'goal_pose'),
            ('/clicked_point', 'clicked_point'),
            ('/initialpose', 'initialpose'),
        ],
    )

    return LaunchDescription([
        # Declare launch arguments
        declare_namespace_arg,
        declare_prefix_arg,
        declare_rviz_config,
        declare_use_sim_time_arg,
        declare_log_level_arg,
        # Log info
        log_info,
        # Nodes
        GroupAction([
            push_namespace,
            rviz_node,
        ])
    ])
