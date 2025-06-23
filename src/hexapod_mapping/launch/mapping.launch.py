import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Configure ROS nodes for localization using RTAB-Map and IMU
    
    # Parameters
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    qos = LaunchConfiguration('qos', default='2')
    
    # Get the URDF file
    urdf_file = os.path.join(get_package_share_directory('hexapod_description'), 'urdf', 'hexapod.urdf.xacro')
    
    # Nodes to launch
    return LaunchDescription([
        # Robot State Publisher
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher',
        #     output='screen',
        #     parameters=[{'use_sim_time': use_sim_time}],
        #     arguments=[urdf_file]),
        
        # IMU Filter
        # Node(
        #     package='imu_filter_madgwick',
        #     executable='imu_filter_madgwick_node',
        #     name='imu_filter',
        #     output='screen',
        #     parameters=[
        #         {'use_mag': False},
        #         {'publish_tf': False},
        #         {'world_frame': 'enu'},
        #         {'fixed_frame': 'imu_link'},
        #         {'orientation_stddev': 0.0}],
        #     remappings=[
        #         ('/imu/data_raw', '/camera/imu')]),
        
        # # IMU to Odometry conversion
        # Node(
        #     package='rtabmap_ros',
        #     executable='imu_to_tf',
        #     name='imu_to_tf',
        #     output='screen',
        #     parameters=[{
        #         'frame_id': 'base_link',
        #         'child_frame_id': 'odom',
        #         'frequency': 200.0,
        #         'use_sim_time': use_sim_time,
        #     }],
        #     remappings=[
        #         ('/imu/data', '/imu/data'),
        #         ('/odom', '/odom_imu')]),
        
        # Custom Depth to LaserScan Node
        # Node(
        #     package='depth_to_laserscan',
        #     executable='depth_to_laserscan',
        #     name='depth_to_laserscan',
        #     output='screen',
        #     parameters=[{
        #         'fx': 617.0,
        #         'fy': 617.0,
        #         'cx': 320.0,
        #         'cy': 240.0,
        #         'angle_min': -1.57,  # -90 degrees
        #         'angle_max': 1.57,   # +90 degrees
        #         'range_min': 0.3,
        #         'range_max': 5.0}],
        #     remappings=[
        #         ('/depth/image', '/camera/depth/image_rect_raw'),
        #         ('/scan', '/scan')]),
        
        # RTAB-Map Node
        # Node(
        #     package='rtabmap_ros',
        #     executable='rtabmap',
        #     name='rtabmap',
        #     output='screen',
        #     parameters=[{
        #         'frame_id': 'base_link',
        #         'odom_frame_id': 'odom',
        #         'subscribe_odom_info': True,
        #         'subscribe_rgbd': True,
        #         'subscribe_scan': True,
        #         'use_sim_time': use_sim_time,
        #         'qos_scan': qos,
        #         'qos_image': qos,
        #         'qos_imu': qos,
        #         'Reg/Strategy': '1',  # 1=ICP
        #         'Icp/CorrespondenceRatio': '0.3',
        #         'Vis/MinInliers': '15',
        #         'RGBD/NeighborLinkRefining': 'True',
        #         'Grid/FromDepth': 'False',
        #         'Grid/3D': 'False',
        #         'Grid/CellSize': '0.05',
        #         'Grid/RangeMax': '20.0',
        #         'Grid/MaxObstacleHeight': '0.3',
        #         'Grid/MinGroundHeight': '-0.1',
        #         'Mem/IncrementalMemory': 'True',
        #         'Mem/InitWMWithAllNodes': 'False',
        #         'Optimizer/GravitySigma': '0.3',
        #         'Reg/Force3DoF': 'true',
        #         'odom_sensor_sync': 'false'}],
        #     remappings=[
        #         ('/odom', '/odom_imu'),
        #         ('/scan', '/scan'),
        #         ('/rgb/image', '/camera/color/image_raw'),
        #         ('/depth/image', '/camera/depth/image_raw'),
        #         ('/rgb/camera_info', '/camera/color/camera_info'),
        #         ('/imu', '/imu/data')],
        #     arguments=['--delete_db_on_start']),
        
        # Visualization
        # Node(
        #     package='rtabmap_ros',
        #     executable='rtabmapviz',
        #     name='rtabmapviz',
        #     output='screen',
        #     parameters=[{
        #         'frame_id': 'base_link',
        #         'odom_frame_id': 'odom',
        #         'subscribe_odom_info': True,
        #         'subscribe_rgbd': True,
        #         'subscribe_scan': True,
        #         'use_sim_time': use_sim_time,
        #         'qos_scan': qos,
        #         'qos_image': qos,
        #         'qos_imu': qos}],
        #     remappings=[
        #         ('/odom', '/odom_imu'),
        #         ('/scan', '/scan'),
        #         ('/rgb/image', '/camera/color/image_raw'),
        #         ('/depth/image', '/camera/depth/image_raw'),
        #         ('/rgb/camera_info', '/camera/color/camera_info'),
        #         ('/imu', '/imu/data')]),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'odom_frame': 'odom',
                'map_frame': 'map',
                'base_frame': 'base_link',
                'odom_topic': '/odom',              # <-- IMPORTANT LINE
                'scan_topic': '/scan',
                'queue_size': 500,
                'transform_publish_period': 0.1,
                'mode': 'mapping',
                'max_laser_range': 5.0,
                'min_laser_range': 0.3,
                'minimum_time_interval': 1.0,
                'throttle_scans': 20,
                'resolution': 0.10,
                'enable_interactive_mode': True
            }]
        )
    ])