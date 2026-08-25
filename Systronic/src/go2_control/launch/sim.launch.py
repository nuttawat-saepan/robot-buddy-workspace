import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import TimerAction


TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'waffle')
ROS_DISTRO = os.environ.get('ROS_DISTRO')


def generate_launch_description():
    go2_share = get_package_share_directory('go2_control')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'launch',
                'turtlebot3_house.launch.py'
            )
        ),
        launch_arguments={
            'x_pose': '-1.6',
            'y_pose': '0.55'
        }.items()
    )

    map_default = os.path.join(
        go2_share,
        'map',
        'house_map.yaml'
    )
    params_default = os.path.join(go2_share, 'nav2_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_dir = LaunchConfiguration('map')
    param_dir = LaunchConfiguration('params_file')
    enable_unitree_bridge = LaunchConfiguration('enable_unitree_bridge')
    enable_april = LaunchConfiguration('enable_april')
    enable_stream = LaunchConfiguration('enable_stream')
    enable_init_pose = LaunchConfiguration('enable_init_pose')
    enable_pc2scan = LaunchConfiguration('enable_pc2scan')
    enable_rviz = LaunchConfiguration('enable_rviz')

    urdf_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'

    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf',
        urdf_file_name
    )

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    pc2scan_param = os.path.join(go2_share, 'pc2scan_sim.yaml')
    april_localizer_param = os.path.join(
        get_package_share_directory('go2_control'),
        'config',
        'april_localizer.yaml'
    )

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    rviz_config_dir = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'),
        'rviz',
        'tb3_navigation2.rviz')
    
    frame_prefix = LaunchConfiguration('frame_prefix')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_default,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=params_default,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        
        DeclareLaunchArgument(
            'frame_prefix',
            default_value='',
            description='TF frame prefix'
        ),
        DeclareLaunchArgument(
            'enable_unitree_bridge',
            default_value='false',
            description='Start Unitree message bridge nodes'),
        DeclareLaunchArgument(
            'enable_april',
            default_value='false',
            description='Start AprilTag localization node'),
        DeclareLaunchArgument(
            'enable_stream',
            default_value='false',
            description='Start WebRTC camera stream node'),
        DeclareLaunchArgument(
            'enable_init_pose',
            default_value='false',
            description='Publish automatic initial pose'),
        DeclareLaunchArgument(
            'enable_pc2scan',
            default_value='false',
            description='Convert /utlidar/cloud back to /scan'),
        DeclareLaunchArgument(
            'enable_rviz',
            default_value='true',
            description='Start RViz'),

        gazebo_launch, 

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir}.items(),
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description,
                'frame_prefix': frame_prefix
            }]
        ),

        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pc2scan_sim',
            output='screen',

            remappings=[
                ('cloud_in', '/utlidar/cloud'),
                ('scan', '/scan')
            ],

            parameters=[pc2scan_param, {
                'use_sim_time': use_sim_time
            }],
            condition=IfCondition(enable_pc2scan)
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(enable_rviz),
            output='screen'),

        # ================= YOUR CONTROL NODES =================
        Node(
            package = 'go2_control',
            executable = 'gazebo_convert',
            name='gazebo_convert',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                ('/sim_joint_states', '/joint_states'),
                ('/sim_odom', '/odom'),
                ('/sim_imu', '/imu'),
            ],
            condition=IfCondition(enable_unitree_bridge),
            output='screen'
        ),

        Node(
            package='go2_control',
            executable='scan_to_pointcloud',
            name='scan_to_pointcloud',
            parameters=[{
                'use_sim_time': use_sim_time,
                'input_topic': '/scan',
                'output_topic': '/utlidar/cloud',
            }],
            condition=IfCondition(enable_unitree_bridge),
            output='screen'
        ),

        Node(
            package = 'go2_control',
            executable = 'go2w_read',
            name='go2w_read',
            parameters=[{'use_sim_time': use_sim_time,
                         'sim_mode': True}],
            condition=IfCondition(enable_unitree_bridge),
            output='screen'
        ),
        

        Node(
            package='go2_control',
            executable='main',
            name='main_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'sim_mode': True,
                         'enable_mqtt': False,
                         'cmd_vel_enabled': False
                         }]
        ),

        Node(
            package='go2_control',
            executable='april_localizer',
            name='april_localizer',
            output='screen',
            condition=IfCondition(enable_april),
            parameters=[april_localizer_param, {'use_sim_time': use_sim_time}]
        ),

        # Node(
        #     package='apriltag_ros',
        #     executable='apriltag_node',
        #     name='apriltag_node',
        #     output='screen',
        #     remappings=[
        #         ('image_rect', '/camera/image_raw'),
        #         ('camera_info', '/camera/camera_info'),
        #         ('detections', '/detections'),
        #     ],
        #     parameters=[{
        #         'family': '36h11',
        #         'use_sim_time': use_sim_time,
        #           'tag_sizes': [0.16], 
        #     }]
        # ),

        Node(
            package='go2_control',
            executable='Stream',
            name='Stream',
            output='screen',
            condition=IfCondition(enable_stream),
            parameters=[{'use_sim_time': use_sim_time,
                         'sim_mode': True
                         }]
        ),


        # ================= AUTO INIT POSE =================
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='go2_control',
                    executable='init_pose',
                    output='screen',
                    condition=IfCondition(enable_init_pose),
                    parameters=[{'use_sim_time': use_sim_time}]
                )
            ]
        ),
        
    ])
