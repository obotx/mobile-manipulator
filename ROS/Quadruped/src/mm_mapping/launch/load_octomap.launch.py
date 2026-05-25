import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    mm_mapping_pkg_share = get_package_share_directory('mm_mapping')

    default_octomap_path = os.path.join(
        mm_mapping_pkg_share,
        'resources',
        'pick_place.bt'
    )

    # Launch args
    octomap_file_arg = DeclareLaunchArgument(
        'octomap_file',
        default_value=default_octomap_path,
        description='Path to .bt or .ot file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use sim time'
    )

    # Configs
    octomap_file = LaunchConfiguration('octomap_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Octomap server
    octomap_server_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[{
            'frame_id': 'map',
            'base_frame_id': 'obotx_base_footprint_platform',
            'publish_2d_map': True,
            'publish_pointcloud': True,
            'publish_free_space': True,
            'octomap_path': octomap_file,
            'track_unknown_space': True,
            'use_sim_time': use_sim_time,
            'publish_frequency': 2.0,
        }],
        remappings=[
            ('projected_map', 'map'),
            ('octomap_full', '/octomap_full'),
            ('octomap_binary', '/octomap_binary'),
        ]
    )

    # Map padder
    map_padding = Node(
        package='mm_mapping',
        executable='map_padder.py',
        name='map_padder',
        parameters=[
            {'padding_x': 3.0},
            {'padding_y': 3.0},
            {'use_sim_time': use_sim_time},
        ]
    )

    # MoveIt octomap bridge
    moveit_octomap_handler = Node(
        package='mm_mapping',
        executable='moveit_octomap_handler.py',
        name='moveit_octomap_handler',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )
    
    return LaunchDescription([
        octomap_file_arg,
        use_sim_time_arg,

        octomap_server_node,
        map_padding,
        moveit_octomap_handler,
    ])