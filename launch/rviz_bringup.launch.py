import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import ReplaceString


def generate_launch_description():
    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description="Namespace under which to bring up nodes, topics, etc.",
    )

    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(
            get_package_share_directory('arcs_cohort_rviz'),
            'rviz',
            'cohort_default.rviz'
        ),
        description='Path to the RViz config file with <NAMESPACE> placeholder'
    )

    # Perform substitution of <NAMESPACE> in the config file
    substituted_config = ReplaceString(
        source_file=LaunchConfiguration('rviz_config'),
        replacements={
            '<NAMESPACE>': LaunchConfiguration('namespace')
        }
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', substituted_config],
    )

    return LaunchDescription([
        declare_namespace,
        declare_rviz_config,
        rviz_node
    ])
