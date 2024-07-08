import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
   # world_path=os.path.join(pkg_share, 'world/my_world.sdf')
    pkg_share = launch_ros.substitutions.FindPackageShare(package='antonio_description').find('antonio_description')
    default_model_path = os.path.join(pkg_share, 'src/description/antonio_description.urdf')
    map_file = os.path.join(pkg_share,'map/map_2.yaml')
    nav2_param_file = os.path.join(pkg_share,'config/nav2_params.yaml')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
    
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )

    robot_localization_node = launch_ros.actions.Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    rplidar_Node = launch_ros.actions.Node(
        name='rplidar_composition',
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,  # A1 / A2
            # 'serial_baudrate': 256000, # A3
            'frame_id': 'lidar_link',
            'inverted': False,
            'angle_compensate': True,}],  
    )
    antonio_odom = launch_ros.actions.Node(
        package='antonio_description',
        executable='antonio_odom',
        name='antonio_odom_publisher'
    )
    antonio_odom_broadcaster = launch_ros.actions.Node(
        package='antonio_description',
        executable='antonio_odom_tf2_broadcaster',
        name='odom_tf2_frame_publisher'
    )
    
    laser_filter_node = launch_ros.actions.Node(
        package='antonio_description',
        executable='laser_filter',
        name='scan_F_filter'
    )
    IMU_odom_broadcaster = launch_ros.actions.Node(
            package='antonio_description',
            executable='imu-base_linck-tf2_brodcaster',
            name='broadcaster2'
    )
    map_server_Node = launch_ros.actions.Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_file}],               
                        
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    scan_to_pointcloud_node  = launch_ros.actions.Node(
            package='pointcloud_to_laserscan',
            executable='laserscan_to_pointcloud_node',
            name='laserscan_to_pointcloud',
            remappings=[('scan_in',  '/scan_filtered'),
                        ('cloud',    '/cloud')],
            parameters=[{'target_frame': 'Floor_scan', 'transform_tolerance': 0.01}]
        )


    return launch.LaunchDescription([


        #Node(
        #    package='nav2_amcl',
        #    executable='amcl',
        #    name='amcl',
        #    output='screen',
        #    parameters=[nav2_param_file]
        #),
        #
        #Node(
        #    package='nav2_lifecycle_manager',
        #    executable='lifecycle_manager',
        #    name='lifecycle_manager_localization',
        #    output='screen',
        #    parameters=[{'use_sim_time': True},
        #                {'autostart': True},
        #                {'node_names': ['map_server', 'amcl']}]
        #), 
   #     Node(
   #         package='robot_state_publisher',
   #         executable='robot_state_publisher',
   #         parameters=[{'robot_description': default_model_path,}]
   # ),       



        #Node(
        #    package='antonio_description',
        #    executable='imu_camera_link-tf2_brodcaster',
        #    name='broadcaster3'
        #),

        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='map', default_value=map_file,
                                            description='Absolute path to map.yaml file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='false',
                                            description='Flag to enable use_sim_time'), 
                                   
        #joint_state_publisher_node,
        #joint_state_publisher_gui_node,
        robot_state_publisher_node,
        laser_filter_node,
        antonio_odom,
        antonio_odom_broadcaster,
        #rplidar_Node,
        #robot_localization_node,
        scan_to_pointcloud_node,
        rviz_node,        
	    #map_server_Node,
        
   
    ])
