"""Nav2, the watchdog and the guard against a Go2W-sized box in Gazebo.

The command path is the real one down to the guard, then a stand-in for the
robot. Nothing here publishes or reads /cmd_vel, and the relay and the Unitree
bridge are not started, so this cannot reach a robot even on a shared network:

    Nav2 -> /cmd_vel_nav_preview -> sensor_watchdog -> /cmd_vel_safe
         -> collision_guard -> /cmd_vel_guarded -> sim_actuator
         -> /sim/cmd_vel -> the box

AMCL and FAST-LIO are left out and map -> odom is fixed, so that a failure here
is about obstacles and not about localisation. /scan is projected from the
simulated cloud instead of FAST-LIO's output. See scripts/run_sim_obstacle.sh.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

NAV2_NODES = ['controller_server', 'planner_server', 'recoveries_server',
              'bt_navigator', 'waypoint_follower']


def generate_launch_description():
    share = get_package_share_directory('go2_control')
    gazebo_share = get_package_share_directory('gazebo_ros')
    bt_xml = os.path.join(share, 'config', 'navigate_backup_first.xml')
    nav2_params = LaunchConfiguration('nav2_params_file')
    amcl_params = LaunchConfiguration('amcl_params_file')
    sim = {'use_sim_time': True}
    wall = {'use_sim_time': False}

    def number(name):
        return ParameterValue(LaunchConfiguration(name), value_type=float)

    web = LaunchConfiguration('web')
    gui = LaunchConfiguration('gui')
    room = LaunchConfiguration('room')
    slam = LaunchConfiguration('slam')
    # AMCL and map_server stand down while slam_toolbox draws the map.
    localising = IfCondition(PythonExpression(
        ["'", web, "' == 'true' and '", slam, "' != 'true'"]))

    nodes = [
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='map_to_odom', parameters=[sim],
             arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
             condition=UnlessCondition(web)),
        # With the web, localisation is the real AMCL, and it needs the initial
        # pose from the web page like it does on site.
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='log',
             parameters=[amcl_params, sim],
             condition=localising),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_amcl', output='log',
             parameters=[sim, {'autostart': True, 'node_names': ['amcl']}],
             condition=localising),
        Node(package='go2_control', executable='map_maker', name='map_maker',
             output='log', parameters=[sim, {
                 'map_dir': os.path.expanduser('~/maps_sim'),
                 # A saved map has no Gazebo world to drive it in.
                 'offer_use': False,
                 'slam_command': (
                     'ros2 run slam_toolbox async_slam_toolbox_node --ros-args'
                     ' -r __node:=slam_toolbox --params-file '
                     + os.path.join(share, 'config', 'slam_livox.yaml')
                     + ' -p use_sim_time:=true')}],
             condition=IfCondition(slam)),
        Node(package='go2_control', executable='mqtt_mission_bridge',
             name='mqtt_mission_bridge', output='log',
             parameters=[sim, {'broker': '127.0.0.1', 'port': 1883}],
             condition=IfCondition(web)),
        # The main page's drive pad is hidden, so hand driving has its own page.
        Node(package='go2_control', executable='sim_manual_drive',
             name='sim_manual_drive', output='log', parameters=[wall],
             condition=IfCondition(web)),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
             parameters=[sim],
             arguments=['-d', os.path.join(share, 'rviz', 'livox_ground.rviz')],
             condition=IfCondition(gui)),
        # yaw pitch roll: the same mount as the guard and the SDF.
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='base_to_lidar', parameters=[sim],
             arguments=['0.30', '0', '0.35', '0', '0.2269', '0',
                        'base_link', 'livox_frame']),
        Node(package='pointcloud_to_laserscan',
             executable='pointcloud_to_laserscan_node',
             name='pc2scan_livox_lio', output='log',
             parameters=[os.path.join(share, 'pc2scan_livox_lio.yaml'), sim],
             remappings=[('cloud_in', '/livox/lidar'), ('scan', '/scan')]),
        Node(package='nav2_map_server', executable='map_server',
             name='map_server', output='log', condition=UnlessCondition(slam),
             parameters=[sim, {'yaml_filename': [
                 os.path.join(share, 'map', ''), room, '.yaml']}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_map', output='log',
             parameters=[sim, {'autostart': True,
                               'node_names': ['map_server']}],
             condition=UnlessCondition(slam)),
        Node(package='go2_control', executable='sensor_watchdog',
             name='sensor_watchdog', output='log', parameters=[wall]),
        Node(package='go2_control', executable='collision_guard',
             name='collision_guard', output='log',
             parameters=[wall, {'cloud_topic': '/livox/lidar',
                                'sensor_x': 0.30,
                                'sensor_height': 0.35,
                                'mount_pitch_deg': 13.0,
                                'mount_roll_deg': 0.0}]),
        Node(package='go2_control', executable='sim_actuator',
             name='sim_actuator', output='log',
             parameters=[wall, {'delay': number('delay'),
                                'linear_accel': number('linear_accel')}]),
    ]

    nav2 = [
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='log',
             parameters=[nav2_params, sim],
             remappings=[('cmd_vel', '/cmd_vel_nav_preview')]),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='log',
             parameters=[nav2_params, sim]),
        Node(package='nav2_recoveries', executable='recoveries_server',
             name='recoveries_server', output='log',
             parameters=[nav2_params, sim],
             remappings=[('cmd_vel', '/cmd_vel_nav_preview')]),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='log',
             parameters=[nav2_params, sim,
                         {'default_bt_xml_filename': bt_xml}]),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower',
             name='waypoint_follower', output='log',
             parameters=[nav2_params, sim]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='log',
             parameters=[sim, {'autostart': True, 'node_names': NAV2_NODES}]),
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'room', default_value='obstacle_room',
            description='A world and map of this name: obstacle_room, or '
                        'meeting_room from go2_control.sim_map_world.'),
        DeclareLaunchArgument(
            'slam', default_value='false',
            description='Make a map instead of using one: no AMCL or '
                        'map_server, and the map page at :8093/slam.'),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(share, 'config', 'nav2_livox_go2.yaml')),
        DeclareLaunchArgument(
            'amcl_params_file',
            default_value=os.path.join(share, 'config', 'amcl_livox.yaml')),
        DeclareLaunchArgument('start_x', default_value='2.0'),
        DeclareLaunchArgument('start_y', default_value='0.0'),
        DeclareLaunchArgument('delay', default_value='0.15'),
        DeclareLaunchArgument('linear_accel', default_value='1.0'),
        DeclareLaunchArgument(
            'web', default_value='false',
            description='Add AMCL and the MQTT bridge to the local broker, so '
                        'the web page drives the simulation.'),
        DeclareLaunchArgument(
            'gui', default_value='false',
            description='Open the Gazebo window and RViz.'),
        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            os.path.join(share, 'models') + ':'
            + os.environ.get('GAZEBO_MODEL_PATH', '')),
        SetEnvironmentVariable(
            'GAZEBO_PLUGIN_PATH', os.environ.get('GAZEBO_PLUGIN_PATH', '')),
        ExecuteProcess(
            cmd=['gzserver', '--verbose',
                 '-s', 'libgazebo_ros_init.so',
                 '-s', 'libgazebo_ros_factory.so',
                 [os.path.join(share, 'worlds', ''), room, '.world']],
            output='log'),
        ExecuteProcess(cmd=['gzclient'], output='log',
                       condition=IfCondition(gui)),
        TimerAction(period=3.0, actions=[
            Node(package='gazebo_ros', executable='spawn_entity.py',
                 name='spawn_robot', output='log',
                 arguments=['-entity', 'go2w_box',
                            '-file', os.path.join(
                                share, 'models', 'go2w_box_sim', 'model.sdf'),
                            '-x', LaunchConfiguration('start_x'),
                            '-y', LaunchConfiguration('start_y')]),
        ]),
        TimerAction(period=5.0, actions=nodes),
        # Once TF and /scan exist, as on the robot.
        TimerAction(period=9.0, actions=nav2),
    ])
