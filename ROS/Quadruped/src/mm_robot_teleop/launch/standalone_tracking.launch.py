import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --- General Arguments ---
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false', description='Whether to use simulation time')
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true', description='Whether to launch RViz2')
    parent_frame_arg = DeclareLaunchArgument('parent_frame', default_value='world', description='Parent frame for the landmark static transform')
    offset_x_arg = DeclareLaunchArgument('offset_x', default_value='0.0', description='X offset for the landmark')
    offset_y_arg = DeclareLaunchArgument('offset_y', default_value='0.0', description='Y offset for the landmark')
    offset_z_arg = DeclareLaunchArgument('offset_z', default_value='0.0', description='Z offset for the landmark')

    # --- Arguments for Landmarks Node ---
    flip_hands_arg = DeclareLaunchArgument('flip_hands', default_value='true', description='Flip left/right hand labels')
    use_bpf_arg = DeclareLaunchArgument('use_bpf', default_value='true', description='Enable Body Pre-Focusing (BPF)')
    hand_padding_arg = DeclareLaunchArgument('hand_padding', default_value='3.0', description='Padding factor for hand crop around wrist')
    img_width_arg = DeclareLaunchArgument('image_width', default_value='640', description='Internal processing width for landmarks node')
    img_height_arg = DeclareLaunchArgument('image_height', default_value='480', description='Internal processing height for landmarks node')
    camera_topic_arg = DeclareLaunchArgument('camera_topic', default_value='/image_raw/compressed', description='Input camera topic')

    # --- Arguments for USB Camera ---
    video_device_arg = DeclareLaunchArgument('video_device', default_value='/dev/video0', description='Video device path')
    cam_width_arg = DeclareLaunchArgument('cam_width', default_value='1920', description='Camera hardware image width')
    cam_height_arg = DeclareLaunchArgument('cam_height', default_value='1080', description='Camera hardware image height')
    pixel_format_arg = DeclareLaunchArgument('pixel_format', default_value='mjpeg2rgb', description='Camera pixel format')
    autofocus_arg = DeclareLaunchArgument('autofocus', default_value='true', description='Enable autofocus')
    brightness_arg = DeclareLaunchArgument('brightness', default_value='140', description='Camera brightness')
    contrast_arg = DeclareLaunchArgument('contrast', default_value='40', description='Camera contrast')
    autoexposure_arg = DeclareLaunchArgument('autoexposure', default_value='true', description='Enable autoexposure')

    # --- Launch Configurations ---
    use_rviz = LaunchConfiguration('use_rviz')
    parent_frame = LaunchConfiguration('parent_frame')
    offset_x = LaunchConfiguration('offset_x')
    offset_y = LaunchConfiguration('offset_y')
    offset_z = LaunchConfiguration('offset_z')
    
    flip_hands = LaunchConfiguration('flip_hands')
    use_bpf = LaunchConfiguration('use_bpf')
    hand_padding = LaunchConfiguration('hand_padding')
    img_width = LaunchConfiguration('image_width')
    img_height = LaunchConfiguration('image_height')
    camera_topic = LaunchConfiguration('camera_topic')

    # --- 1. USB Camera Node ---
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[{
            'video_device': LaunchConfiguration('video_device'),
            'image_width': LaunchConfiguration('cam_width'),
            'image_height': LaunchConfiguration('cam_height'),
            'pixel_format': LaunchConfiguration('pixel_format'),
            'auto_focus': LaunchConfiguration('autofocus'), # Note: usb_cam uses 'auto_focus'
            'brightness': LaunchConfiguration('brightness'),
            'contrast': LaunchConfiguration('contrast'),
            'autoexposure': LaunchConfiguration('autoexposure'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # --- 2. Landmarks Node ---
    landmarks_node = Node(
        package='mm_robot_teleop', 
        executable='landmarks_node', 
        name='landmarks_node',
        output='screen',
        parameters=[{
            'flip_hands': flip_hands,
            'use_bpf': use_bpf,
            'hand_padding': hand_padding,
            'image_width': img_width,
            'image_height': img_height,
            'publish_2d': True,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        remappings=[
            # Remap the default usb_cam compressed topic to whatever the landmarks node expects
            ('/image_raw/compressed', camera_topic) 
        ]
    )

    # --- 3. Static TF Publisher ---
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='landmark_static_tf',
        output='screen',
        arguments=[
            '--x', offset_x, '--y', offset_y, '--z', offset_z,
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '3.14159265',
            '--frame-id', parent_frame,
            '--child-frame-id', 'landmark_root' 
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # --- 4. RViz2 ---
    rviz_config_dir = os.path.join(
        get_package_share_directory('mm_robot_teleop'),
        'rviz',
        'hand_tracking.rviz' 
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen',
        condition=IfCondition(use_rviz),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        # General
        use_sim_time, use_rviz_arg, parent_frame_arg, 
        offset_x_arg, offset_y_arg, offset_z_arg,
        # Landmarks Node
        flip_hands_arg, use_bpf_arg, hand_padding_arg, 
        img_width_arg, img_height_arg, camera_topic_arg,
        # USB Camera
        video_device_arg, cam_width_arg, cam_height_arg,
        pixel_format_arg, autofocus_arg, brightness_arg, 
        contrast_arg, autoexposure_arg,
        # Nodes
        usb_cam_node, landmarks_node, static_tf_node, rviz_node
    ])