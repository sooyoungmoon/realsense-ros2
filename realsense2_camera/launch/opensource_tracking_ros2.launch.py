# example.launch.py

import os

from ament_index_python import get_package_share_directory

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetLaunchConfiguration
from launch.actions import GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def generate_launch_description():

    remappings=[
        ('/camera/camera/aligned_depth_to_color/image_raw', '/camera/aligned_depth_to_color/image_raw'),
        ('/camera/camera/aligned_depth_to_color/camera_info', '/camera/aligned_depth_to_color/camera_info'),
        ('/camera/camera/aligned_depth_to_color/image_raw/compressed', '/camera/aligned_depth_to_color/image_raw/compressed'),
        ('/camera/camera/aligned_depth_to_color/image_raw/compressedDepth', '/camera/aligned_depth_to_color/image_raw/compressedDepth'),
        ('/camera/camera/aligned_depth_to_color/image_raw/theora', '/camera/aligned_depth_to_color/image_raw/theora'),
        ('/camera/camera/accel/imu_info', '/camera/accel/imu_info'),
        ('/camera/camera/accel/metadata', '/camera/accel/metadata'),
        ('/camera/camera/accel/sample', '/camera/accel/sample'),
        ('/camera/camera/color/image_raw/compressed', '/camera/color/image_raw/compressed'),
        ('/camera/camera/color/image_raw/compressedDepth', '/camera/color/image_raw/compressedDepth'),
        ('/camera/camera/color/image_raw/theora', '/camera/color/image_raw/theora'),
        ('/camera/camera/color/metadata', '/camera/color/metadata'),
        ('/camera/camera/depth/camera_info', '/camera/depth/camera_info'),
        ('/camera/camera/depth/image_rect_raw', '/camera/depth/image_rect_raw'),
        ('/camera/camera/depth/image_rect_raw/compressed', '/camera/depth/image_rect_raw/compressed'),
        ('/camera/camera/depth/image_rect_raw/compressedDepth', '/camera/depth/image_rect_raw/compressedDepth'),
        ('/camera/camera/depth/image_rect_raw/theora', '/camera/depth/image_rect_raw/theora'),
        ('/camera/camera/depth/metadata', '/camera/depth/metadata'),
        ('/camera/camera/extrinsics/depth_to_accel', '/camera/extrinsics/depth_to_accel'),
        ('/camera/camera/extrinsics/depth_to_color', '/camera/extrinsics/depth_to_color'),
        ('/camera/camera/extrinsics/depth_to_gyro', '/camera/extrinsics/depth_to_gyro'),
        ('/camera/camera/gyro/imu_info', '/camera/gyro/imu_info'),
        ('/camera/camera/gyro/metadata', '/camera/gyro/metadata'),
        ('/camera/camera/gyro/sample', '/camera/gyro/sample'),
        ('/camera/camera/imu', '/camera/imu'),
        ('/camera/camera/color/camera_info', '/camera/color/camera_info'),
        ('/camera/camera/color/image_raw', '/camera/color/image_raw')  
    ]


    # args that can be set from the command line or a default will be used
    enable_accel_arg = DeclareLaunchArgument(
        "enable_accel",
        default_value=TextSubstitution(text="true")
    )
    enable_gyro_arg = DeclareLaunchArgument(
        "enable_gyro",
        default_value=TextSubstitution(text="true")
    )
    offline_arg = DeclareLaunchArgument(
        "offline", 
        default_value=TextSubstitution(text="false")
    )
    align_depth_arg = DeclareLaunchArgument(
        "align_depth.enable", 
        default_value=TextSubstitution(text="true")
    )

    linear_accel_cov_arg = DeclareLaunchArgument(
        "linear_accel_cov", 
        default_value=TextSubstitution(text="1.0")
    )

    unite_imu_method_arg = DeclareLaunchArgument(
        "unite_imu_method",
        default_value=TextSubstitution(text="2")
    )

    use_mag_arg = DeclareLaunchArgument(
        "use_mag",
        default_value=TextSubstitution(text="false")
    )

    _publish_tf_arg = DeclareLaunchArgument(
        "_publish_tf",
        default_value=TextSubstitution(text="false")
    )

    _world_frame_arg = DeclareLaunchArgument(
        "_world_frame",
        default_value=TextSubstitution(text="enu")
    )

    ## arguments for rtabmap
    # Choose between depth and stereo, set both to false to do only scan
    stereo_arg = DeclareLaunchArgument(
        "stereo",
        default_value=TextSubstitution(text="false")
    )
    depth_arg = DeclareLaunchArgument(
        "depth",
        default_value=TextSubstitution(text="true")
    )

    subscribe_rgb_arg = DeclareLaunchArgument(
        "subscribe_rgb",
        default_value=LaunchConfiguration('depth')
    )

    #Localization-only mode 
    localization_arg = DeclareLaunchArgument(
        "localization",
        default_value=TextSubstitution(text="false")
    )
    initial_pose_arg = DeclareLaunchArgument(
        "initial_pose",
        default_value=TextSubstitution(text="")
    )
    # sim time for convenience, if playing a rosbag
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value=TextSubstitution(text="false")
    )
    # Corresponding config files
    cfg_arg = DeclareLaunchArgument(
        "cfg",
        default_value=TextSubstitution(text="")
    )
    gui_cfg_arg = DeclareLaunchArgument(
        "gui_cfg",
        default_value=TextSubstitution(text="")
    )
    rviz_config_file = os.path.join(
        get_package_share_directory('rtabmap_launch'),
        'launch',
        'config',
        'rgbd.rviz'
    )
    rviz_cfg_arg = DeclareLaunchArgument(
        "rviz_cfg",
        default_value=rviz_config_file
    )
    frame_id_arg = DeclareLaunchArgument(
        "frame_id",
        default_value=TextSubstitution(text="camera_link")
    )
    odom_frame_id_arg = DeclareLaunchArgument(
        "odom_frame_id",
        default_value=TextSubstitution(text="")
    )
    odom_frame_id_init_arg = DeclareLaunchArgument(
        "odom_frame_id_init",
        default_value=TextSubstitution(text="")
    )
    map_frame_id_arg = DeclareLaunchArgument(
        "map_frame_id",
        default_value=TextSubstitution(text="map")
    )
    ground_truth_frame_id_arg = DeclareLaunchArgument(
        "ground_truth_frame_id",
        default_value=TextSubstitution(text="")
    )
    ground_truth_base_frame_id_arg = DeclareLaunchArgument(
        "ground_truth_base_frame_id",
        default_value=TextSubstitution(text="")
    )
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=TextSubstitution(text="rtabmap")
    )
    database_path_arg = DeclareLaunchArgument(
        "database_path",
        default_value=TextSubstitution(text="~/.ros/rtabmap.db")
    )
    queue_size_arg = DeclareLaunchArgument(
        "queue_size",
        default_value=TextSubstitution(text="10")
    )
    wait_for_transform_arg = DeclareLaunchArgument(
        "wait_for_transform",
        default_value=TextSubstitution(text="0.2")
    )
    gdb_arg = DeclareLaunchArgument(
        "gdb",
        default_value=TextSubstitution(text="false")
    )
    launch_prefix_arg = DeclareLaunchArgument(
        "launch_prefix",
        default_value=TextSubstitution(text="")
    )
    #SetLaunchConfiguration(
    #    'launch_prefix', 'true', condition=IfCondition(LaunchConfiguration('gdb'))
    #)
    #SetLaunchConfiguration(
    #    'launch_prefix', 'false', condition=UnlessCondition(LaunchConfiguration('gdb'))
    #)
    clear_params_arg = DeclareLaunchArgument(
        "clear_params",
        default_value=TextSubstitution(text="true")
    )
    output_arg = DeclareLaunchArgument(
        "output",
        default_value=TextSubstitution(text="screen")
    )
    publish_tf_map_arg = DeclareLaunchArgument(
        "publish_tf_map",
        default_value=TextSubstitution(text="true")
    )

    args_arg = DeclareLaunchArgument(
        "args",
        default_value=TextSubstitution(text="--delete_db_on_start")
    )

    rgb_topic_arg = DeclareLaunchArgument(
        "rgb_topic",
        default_value=TextSubstitution(text="/camera/color/image_raw")
    )

    depth_topic_arg = DeclareLaunchArgument(
        "depth_topic",
        default_value=TextSubstitution(text="/camera/aligned_depth_to_color/image_raw")
    )

    camera_info_topic_arg = DeclareLaunchArgument(
        "camera_info_topic",
        default_value=TextSubstitution(text="/camera/color/camera_info")
    )

    depth_camera_info_topic_arg = DeclareLaunchArgument(
        "depth_camera_info_topic",
        default_value=TextSubstitution(text="/camera/depth/camera_info")   
    )

    rtabmap_viz_arg = DeclareLaunchArgument(
        "rtabmap_viz",
        default_value=TextSubstitution(text="false")
    )
    rtabmapviz_arg = DeclareLaunchArgument(
        "rtabmapviz",
        default_value=TextSubstitution(text="false")
    )

    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value=TextSubstitution(text="true")
    )

    approx_sync_arg = DeclareLaunchArgument(
        "approx_sync",
        default_value= LaunchConfiguration('depth')
    )
    approx_sync_max_interval_arg = DeclareLaunchArgument(
        "approx_sync_max_interval",
        default_value=TextSubstitution(text="0.0")
    )
  
    # stereo related topics
    stereo_namespace_arg = DeclareLaunchArgument(
        "stereo_namespace",
        default_value=TextSubstitution(text="/stereo_camera")
    )
    left_image_topic_arg = DeclareLaunchArgument(
        "left_image_topic",
        default_value=TextSubstitution(text="/stereo_camera/left/image_rect_color")
    )
    right_image_topic_arg = DeclareLaunchArgument(
        "right_image_topic",
        default_value=TextSubstitution(text="/stereo_camera/right/image_rect_color")
    )
    left_camera_info_topic_arg = DeclareLaunchArgument(
        "left_camera_info_topic",
        default_value=TextSubstitution(text="/stereo_camera/left/camera_info")
    )
    right_camera_info_topic_arg = DeclareLaunchArgument(
        "right_camera_info_topic",
        default_value=TextSubstitution(text="/stereo_camera/right/camera_info")
    )

    # Already synchronized RGB-D related topic, with rtabmap_sync/rgbd_sync nodelet
    rgbd_sync_arg = DeclareLaunchArgument(
        "rgbd_sync",
        default_value=TextSubstitution(text="false")
    )
    approx_rgbd_sync_arg = DeclareLaunchArgument(
        "approx_rgbd_sync",
        default_value=TextSubstitution(text="true")
    )
    subscribe_rgbd_arg = DeclareLaunchArgument(
        "subscribe_rgbd",
        default_value = LaunchConfiguration('rgbd_sync')
    )
    rgbd_topic_arg = DeclareLaunchArgument(
        "rgbd_topic",
        default_value=TextSubstitution(text="rgbd_image")
    )
    depth_scale_arg = DeclareLaunchArgument(
        "depth_scale",
        default_value=TextSubstitution(text="1.0")
    )
    rgbd_depth_scale_arg = DeclareLaunchArgument(
        "rgbd_depth_scale",
        default_value=LaunchConfiguration('depth_scale')
    )
    rgbd_decimation_arg = DeclareLaunchArgument(
        "rgbd_decimation",
        default_value=TextSubstitution(text="1")
    )
    compressed_arg = DeclareLaunchArgument(
        "compressed",
        default_value=TextSubstitution(text="false")
    )
    rgb_image_transport_arg = DeclareLaunchArgument(
        "rgb_image_transport",
        default_value=TextSubstitution(text="compressed")
    )
    depth_image_transport_arg = DeclareLaunchArgument(
        "depth_image_transport",
        default_value=TextSubstitution(text="compressedDepth")
    )
    gen_cloud_arg = DeclareLaunchArgument(
        "gen_cloud",
        default_value=TextSubstitution(text="false")
    )
    gen_cloud_decimation_arg = DeclareLaunchArgument(
        "gen_cloud_decimation",
        default_value=TextSubstitution(text="4")
    )
    gen_cloud_voxel_arg = DeclareLaunchArgument(
        "gen_cloud_voxel",
        default_value=TextSubstitution(text="0.05")
    )
    subscribe_scan_arg = DeclareLaunchArgument(
        "subscribe_scan",
        default_value= TextSubstitution(text="false")
    )
    scan_cloud_topic_arg = DeclareLaunchArgument(
        "scan_cloud_topic",
        default_value=TextSubstitution(text="/scan_cloud") 
    )
    subscribe_scan_descriptor_arg = DeclareLaunchArgument(
        "subscribe_scan_descriptor",
        default_value=TextSubstitution(text="false")
    )
    scan_descriptor_topic_arg = DeclareLaunchArgument(
        "scan_descriptor_topic",
        default_value=TextSubstitution(text="/scan_descriptor")
    )
    scan_deskewing_arg = DeclareLaunchArgument(
        "scan_deskewing",
        default_value=TextSubstitution(text="false")
    )
    scan_deskewing_slerp_arg = DeclareLaunchArgument(
        "scan_deskewing_slerp",
        default_value=TextSubstitution(text="false")
    )
    scan_cloud_max_points_arg = DeclareLaunchArgument(
        "scan_cloud_max_points",
        default_value=TextSubstitution(text="0")
    )
    scan_cloud_filtered_arg = DeclareLaunchArgument(
        "scan_cloud_filtered",
        default_value=LaunchConfiguration('scan_deskewing')
    )
    gen_scan_arg = DeclareLaunchArgument(
        "gen_scan",
        default_value=TextSubstitution(text="false")
    )
    gen_depth_arg = DeclareLaunchArgument(
        "gen_depth",
        default_value=TextSubstitution(text="false")
    )
    gen_depth_decimation_arg = DeclareLaunchArgument(
        "gen_depth_decimation",
        default_value=TextSubstitution(text="1")
    )
    gen_depth_fill_holes_size_arg = DeclareLaunchArgument(
        "gen_depth_fill_holes_size",
        default_value=TextSubstitution(text="0")
    )
    gen_depth_fill_iterations_arg = DeclareLaunchArgument(
        "gen_depth_fill_iterations",
        default_value=TextSubstitution(text="1")
    )
    gen_depth_fill_holes_error_arg = DeclareLaunchArgument(
        "gen_depth_fill_holes_error",
        default_value=TextSubstitution(text="0.1")
    )
    visual_odometry_arg = DeclareLaunchArgument(
        "visual_odometry",
        default_value=TextSubstitution(text="true")
    )
    icp_odometry_arg = DeclareLaunchArgument(
        "icp_odometry",
        default_value=TextSubstitution(text="false")
    )
    odom_topic_arg = DeclareLaunchArgument(
        "odom_topic",
        default_value=TextSubstitution(text="odom")
    )
    vo_frame_id_arg = DeclareLaunchArgument(
        "vo_frame_id",
        default_value=LaunchConfiguration('odom_topic')
    )
    publish_tf_odom_arg = DeclareLaunchArgument(
        "publish_tf_odom",
        default_value=TextSubstitution(text="true")
    )
    odom_tf_angular_variance_arg = DeclareLaunchArgument(
        "odom_tf_angular_variance",
        default_value=TextSubstitution(text="0.001")
    )
    odom_tf_linear_variance_arg = DeclareLaunchArgument(
        "odom_tf_linear_variance",
        default_value=TextSubstitution(text="0.001")
    )
    odom_args_arg = DeclareLaunchArgument(
        "odom_args",
        default_value=TextSubstitution(text="")
    )
    odom_sensor_sync_arg = DeclareLaunchArgument(
        "odom_sensor_sync",
        default_value=TextSubstitution(text="false")
    )
    odom_guess_frame_id_arg = DeclareLaunchArgument(
        "odom_guess_frame_id",
        default_value=TextSubstitution(text="")
    )
    odom_guess_min_translation_arg = DeclareLaunchArgument(
        "odom_guess_min_translation",
        default_value=TextSubstitution(text="0.0")
    )
    odom_guess_min_rotation_arg = DeclareLaunchArgument(
        "odom_guess_min_rotation",
        default_value=TextSubstitution(text="0.0")
    )
    odom_max_rate_arg = DeclareLaunchArgument(
        "odom_max_rate",
        default_value=TextSubstitution(text="0.0")
    )
    odom_expected_rate_arg = DeclareLaunchArgument(
        "odom_expected_rate",
        default_value=TextSubstitution(text="0.0")
    )
    imu_topic_arg = DeclareLaunchArgument(
        "imu_topic",
        default_value=TextSubstitution(text="/imu/data")
    )
    wait_imu_to_init_arg = DeclareLaunchArgument(
        "wait_imu_to_init",
        default_value=TextSubstitution(text="false")
    )
    use_odom_features_arg = DeclareLaunchArgument(
        "use_odom_features",
        default_value=TextSubstitution(text="false")
    )

    scan_cloud_assembling_arg = DeclareLaunchArgument(
        "scan_cloud_assembling",
        default_value=TextSubstitution(text="false")
    )
    scan_cloud_assembling_time_arg = DeclareLaunchArgument(
        "scan_cloud_assembling_time",
        default_value=TextSubstitution(text="1")
    )
    scan_cloud_assembling_max_clouds_arg = DeclareLaunchArgument(
        "scan_cloud_assembling_max_clouds",
        default_value=TextSubstitution(text="0")
    )
    scan_cloud_assembling_fixed_frame_arg = DeclareLaunchArgument(
        "scan_cloud_assembling_fixed_frame",
        default_value=TextSubstitution(text="")
    )
    scan_cloud_assembling_voxel_size_arg = DeclareLaunchArgument(
        "scan_cloud_assembling_voxel_size",
        default_value=TextSubstitution(text="0.05")
    )
    scan_cloud_assembling_range_min_arg = DeclareLaunchArgument(
        "scan_cloud_assembling_range_min",
        default_value=TextSubstitution(text="0.0")
    )
    scan_cloud_assembling_range_max_arg = DeclareLaunchArgument(
        "scan_cloud_assembling_range_max",
        default_value=TextSubstitution(text="0.0")
    )
    scan_cloud_assembling_noise_radius_arg = DeclareLaunchArgument(
        "scan_cloud_assembling_noise_radius",
        default_value=TextSubstitution(text="0.0")
    )
    scan_cloud_assembling_noise_min_neighbors_arg = DeclareLaunchArgument(
        "scan_cloud_assembling_noise_min_neighbors",
        default_value=TextSubstitution(text="5")
    )
    subscribe_user_data_arg = DeclareLaunchArgument(
        "subscribe_user_data",
        default_value=TextSubstitution(text="false")
    )
    user_data_topic_arg = DeclareLaunchArgument(
        "user_data_topic",
        default_value=TextSubstitution(text="/user_data")
    )
    user_data_async_topic_arg = DeclareLaunchArgument(
        "user_data_async_topic",
        default_value=TextSubstitution(text="/user_data_async")
    )
    gps_topic_arg = DeclareLaunchArgument(
        "gps_topic",
        default_value=TextSubstitution(text="/gps/fix")
    )
    tag_topic_arg = DeclareLaunchArgument(
        "tag_topic",
        default_value=TextSubstitution(text="/tag_detections")
    )
    tag_linear_variance_arg = DeclareLaunchArgument(
        "tag_linear_variance",
        default_value=TextSubstitution(text="0.0001")
    )
    tag_angular_variance_arg = DeclareLaunchArgument(
        "tag_angular_variance",
        default_value=TextSubstitution(text="9999.0")
    )
    fiducial_topic_arg = DeclareLaunchArgument(
        "fiducial_topic",
        default_value=TextSubstitution(text="/fiducial_transforms")
    )

    rgb_topic_relay_arg = DeclareLaunchArgument(
        "rgb_topic_relay",
        default_value=  TextSubstitution(text="/camera/color/image_raw")
    )
    depth_topic_relay_arg = DeclareLaunchArgument(
        "depth_topic_relay",
        default_value= TextSubstitution(text="/camera/aligned_depth_to_color/image_raw")
    )
    left_image_topic_relay_arg = DeclareLaunchArgument(
        "left_image_topic_relay",
        default_value= TextSubstitution(text="/stereo_camera/left/image_rect_color")
    )    
    right_image_topic_relay_arg = DeclareLaunchArgument(
        "right_image_topic_relay",
        default_value= TextSubstitution(text="/stereo_camera/right/image_rect_color")
    )
    rgbd_topic_relay_arg = DeclareLaunchArgument(
        "rgbd_topic_relay",
        default_value= TextSubstitution(text="/rgbd_image_relay")
    )
    
    # arguments for robot_localization
    frequency_arg = DeclareLaunchArgument(
        "frequency",
        default_value=TextSubstitution(text="300")
    )

    base_link_frame_arg = DeclareLaunchArgument(
        "base_link_frame",
        default_value=TextSubstitution(text="camera_link")
    )

    odom0_arg = DeclareLaunchArgument(
        "odom0",
        default_value=TextSubstitution(text="rtabmap/odom")
    )

    odom0_config_arg = DeclareLaunchArgument(
        "odom0_config",
        default_value=TextSubstitution(text="true,true,true,true,true,true,true,true,true,true,true,true,true,true,true")
    )


    odom0_relative_arg = DeclareLaunchArgument(
        "odom0_relative",
        default_value=TextSubstitution(text="true")    
    )

    odom0_pose_rejection_threshold_arg = DeclareLaunchArgument( 
        "odom0_pose_rejection_threshold",
        default_value=TextSubstitution(text="10000000")
    )

    odom0_twist_rejection_threshold_arg = DeclareLaunchArgument(    
        "odom0_twist_rejection_threshold",
        default_value=TextSubstitution(text="10000000")
    )

    imu0_arg = DeclareLaunchArgument(
        "imu0",
        default_value=TextSubstitution(text="/imu/data")
    )   

    imu0_config_arg = DeclareLaunchArgument(
        "imu0_config",
        default_value=TextSubstitution(text="false,false,false,true,true,true,true,true,true,true,true,true,true,true,true")
                                           
    )   

    imu0_differential_arg = DeclareLaunchArgument(
        "imu0_differential",
        default_value=TextSubstitution(text="true")
    )

    imu0_relative_arg = DeclareLaunchArgument(
        "imu0_relative",
        default_value=TextSubstitution(text="false")
    )       

    use_control_arg = DeclareLaunchArgument(
        "use_control",
        default_value=TextSubstitution(text="false")
    )

    rs2_camera_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(           
            [ThisLaunchFileDir(), '/rs_launch.py']),
        launch_arguments={
            'remappings': str(remappings),
            'enable_accel': LaunchConfiguration('enable_accel'),
            'enable_gyro': LaunchConfiguration('enable_gyro'),
            'align_depth.enable': LaunchConfiguration('align_depth.enable'), 
            'linear_accel_cov': LaunchConfiguration('linear_accel_cov'),
            'unite_imu_method': LaunchConfiguration('unite_imu_method')}.items()            
    )

    imu_filter_madgwick_node = Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='ImuFilter',
            remappings=[
                ('/imu/data_raw', '/camera/imu')
            ],
            parameters=[{
                'use_mag': False,
                'publish_tf': False,
                'world_frame': 'enu',
            }]
        )
    rtabmap_ros_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rtabmap_launch'),
                'launch/rtabmap.launch.py')),
                launch_arguments={
                'args': LaunchConfiguration('args'), 
                'rgb_topic': LaunchConfiguration('rgb_topic'),
                'depth_topic': LaunchConfiguration('depth_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'depth_camera_info_topic': LaunchConfiguration('depth_camera_info_topic'),
                'rtabmap_viz': LaunchConfiguration('rtabmap_viz'),
                'rviz': LaunchConfiguration('rviz'),
                'stereo': LaunchConfiguration('stereo'),
                'depth': LaunchConfiguration('depth'),
                'subscribe_rgb': LaunchConfiguration('subscribe_rgb'),
                'rtabmap_viz': LaunchConfiguration('rtabmap_viz'),
                'localization': LaunchConfiguration('localization'),
                'initial_pose': LaunchConfiguration('initial_pose'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'cfg': LaunchConfiguration('cfg'),
                'gui_cfg': LaunchConfiguration('gui_cfg'),
                'rviz_cfg': LaunchConfiguration('rviz_cfg'),
                'frame_id': LaunchConfiguration('frame_id'),
                'odom_frame_id': LaunchConfiguration('odom_frame_id'),
                'odom_frame_id_init': LaunchConfiguration('odom_frame_id_init'),
                'map_frame_id': LaunchConfiguration('map_frame_id'),
                'ground_truth_frame_id': LaunchConfiguration('ground_truth_frame_id'),
                'ground_truth_base_frame_id': LaunchConfiguration('ground_truth_base_frame_id'),
                'namespace': LaunchConfiguration('namespace'),
                'database_path': LaunchConfiguration('database_path'),
                'queue_size': LaunchConfiguration('queue_size'),
                'wait_for_transform': LaunchConfiguration('wait_for_transform'),
                'gdb': LaunchConfiguration('gdb'),
                'launch_prefix': LaunchConfiguration('launch_prefix'),
                'clear_params': LaunchConfiguration('clear_params'),
                'output': LaunchConfiguration('output'),
                'publish_tf_map': LaunchConfiguration('publish_tf_map'),   

                'approx_sync': LaunchConfiguration('approx_sync'),
                'approx_sync_max_interval': LaunchConfiguration('approx_sync_max_interval'),
                'rgb_topic': LaunchConfiguration('rgb_topic'),
                'depth_topic': LaunchConfiguration('depth_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'depth_camera_info_topic': LaunchConfiguration('depth_camera_info_topic'),
                'stereo_namespace': LaunchConfiguration('stereo_namespace'),
                'left_image_topic': LaunchConfiguration('left_image_topic'),
                'right_image_topic': LaunchConfiguration('right_image_topic'),
                'left_camera_info_topic': LaunchConfiguration('left_camera_info_topic'),
                'right_camera_info_topic': LaunchConfiguration('right_camera_info_topic'),
                'rgbd_sync': LaunchConfiguration('rgbd_sync'),
                'approx_rgbd_sync': LaunchConfiguration('approx_rgbd_sync'),
                'subscribe_rgbd': LaunchConfiguration('subscribe_rgbd'),
                'rgbd_topic': LaunchConfiguration('rgbd_topic'),
                'depth_scale': LaunchConfiguration('depth_scale'),
                'rgbd_depth_scale': LaunchConfiguration('rgbd_depth_scale'),
                'rgbd_decimation': LaunchConfiguration('rgbd_decimation'),
                'compressed': LaunchConfiguration('compressed'),
                'rgb_image_transport': LaunchConfiguration('rgb_image_transport'),
                'depth_image_transport': LaunchConfiguration('depth_image_transport'),
             
                'gen_cloud': LaunchConfiguration('gen_cloud'),
                'gen_cloud_decimation': LaunchConfiguration('gen_cloud_decimation'),
                'gen_cloud_voxel': LaunchConfiguration('gen_cloud_voxel'),
                'subscribe_scan': LaunchConfiguration('subscribe_scan'),
                'scan_cloud_topic': LaunchConfiguration('scan_cloud_topic'),
                'subscribe_scan_descriptor': LaunchConfiguration('subscribe_scan_descriptor'),
                'scan_descriptor_topic': LaunchConfiguration('scan_descriptor_topic'),
                'scan_deskewing': LaunchConfiguration('scan_deskewing'),
                'scan_deskewing_slerp': LaunchConfiguration('scan_deskewing_slerp'),
                'scan_cloud_max_points': LaunchConfiguration('scan_cloud_max_points'),
                'scan_cloud_filtered': LaunchConfiguration('scan_cloud_filtered'),
                'gen_scan': LaunchConfiguration('gen_scan'),
              
                'gen_depth': LaunchConfiguration('gen_depth'),
                'gen_depth_decimation': LaunchConfiguration('gen_depth_decimation'),
                'gen_depth_fill_holes_size': LaunchConfiguration('gen_depth_fill_holes_size'),
                'gen_depth_fill_iterations': LaunchConfiguration('gen_depth_fill_iterations'),
                'gen_depth_fill_holes_error': LaunchConfiguration('gen_depth_fill_holes_error'),
               
                'visual_odometry': LaunchConfiguration('visual_odometry'),
                'icp_odometry': LaunchConfiguration('icp_odometry'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'vo_frame_id': LaunchConfiguration('vo_frame_id'),
                'publish_tf_odom': LaunchConfiguration('publish_tf_odom'),
                'odom_tf_angular_variance': LaunchConfiguration('odom_tf_angular_variance'),
                'odom_tf_linear_variance': LaunchConfiguration('odom_tf_linear_variance'),
                'odom_args': LaunchConfiguration('odom_args'),
                'odom_sensor_sync': LaunchConfiguration('odom_sensor_sync'),
                'odom_guess_frame_id': LaunchConfiguration('odom_guess_frame_id'),
                'odom_guess_min_translation': LaunchConfiguration('odom_guess_min_translation'),
                'odom_guess_min_rotation': LaunchConfiguration('odom_guess_min_rotation'),
                'odom_max_rate': LaunchConfiguration('odom_max_rate'),
                'odom_expected_rate': LaunchConfiguration('odom_expected_rate'),
                'imu_topic': LaunchConfiguration('imu_topic'),
                'wait_imu_to_init': LaunchConfiguration('wait_imu_to_init'),
                'use_odom_features': LaunchConfiguration('use_odom_features'),
               
                'scan_cloud_assembling': LaunchConfiguration('scan_cloud_assembling'),
                'scan_cloud_assembling_time': LaunchConfiguration('scan_cloud_assembling_time'),
                'scan_cloud_assembling_max_clouds': LaunchConfiguration('scan_cloud_assembling_max_clouds'),
                'scan_cloud_assembling_fixed_frame': LaunchConfiguration('scan_cloud_assembling_fixed_frame'),
                'scan_cloud_assembling_voxel_size': LaunchConfiguration('scan_cloud_assembling_voxel_size'),
                'scan_cloud_assembling_range_min': LaunchConfiguration('scan_cloud_assembling_range_min'),
                'scan_cloud_assembling_range_max': LaunchConfiguration('scan_cloud_assembling_range_max'),
                'scan_cloud_assembling_noise_radius': LaunchConfiguration('scan_cloud_assembling_noise_radius'),
                'scan_cloud_assembling_noise_min_neighbors': LaunchConfiguration('scan_cloud_assembling_noise_min_neighbors'),
               
                'subscribe_user_data': LaunchConfiguration('subscribe_user_data'),
                'user_data_topic': LaunchConfiguration('user_data_topic'),
                'user_data_async_topic': LaunchConfiguration('user_data_async_topic'),
                'gps_topic': LaunchConfiguration('gps_topic'),
                'tag_topic': LaunchConfiguration('tag_topic'),
                'tag_linear_variance': LaunchConfiguration('tag_linear_variance'),
                'tag_angular_variance': LaunchConfiguration('tag_angular_variance'),
                'fiducial_topic': LaunchConfiguration('fiducial_topic'),
               
                'rgb_topic_relay': LaunchConfiguration('rgb_topic_relay'),
                'depth_topic_relay': LaunchConfiguration('depth_topic_relay'),
                'left_image_topic_relay': LaunchConfiguration('left_image_topic_relay'),
                'right_image_topic_relay': LaunchConfiguration('right_image_topic_relay'),
                'rgbd_topic_relay' : LaunchConfiguration('rgbd_topic_relay'), 
                #'approx_sync': 'true',
                #'approx_sync_max_interval': '0.02'
                }.items()                               
    )

    robot_localization_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robot_localization'),
                'launch/ukf.launch.py'))     
    )   

    return LaunchDescription([
        enable_accel_arg,
        enable_gyro_arg,
        offline_arg,
        align_depth_arg,
        linear_accel_cov_arg,
        unite_imu_method_arg,
        use_mag_arg,
        _publish_tf_arg,
        _world_frame_arg,
        args_arg,
        rgb_topic_arg,
        depth_topic_arg,
        camera_info_topic_arg,
        depth_camera_info_topic_arg,
        rtabmap_viz_arg,
        rtabmapviz_arg,
        rviz_arg,
        stereo_arg,
        depth_arg,       
        subscribe_rgb_arg,
        localization_arg,
        initial_pose_arg,
        use_sim_time_arg,
        cfg_arg,
        gui_cfg_arg,
        rviz_cfg_arg,
        frame_id_arg,
        odom_frame_id_arg,
        odom_frame_id_init_arg,
        map_frame_id_arg,
        ground_truth_frame_id_arg,
        ground_truth_base_frame_id_arg,
        namespace_arg,
        database_path_arg,
        queue_size_arg,
        wait_for_transform_arg,
        gdb_arg,
        launch_prefix_arg,      
        clear_params_arg,
        output_arg,
        publish_tf_map_arg,
        approx_sync_arg,
        approx_sync_max_interval_arg,
        rgb_topic_arg,
        depth_topic_arg,
        camera_info_topic_arg,
        depth_camera_info_topic_arg,
        stereo_namespace_arg,
        left_image_topic_arg,
        right_image_topic_arg,
        left_camera_info_topic_arg,
        right_camera_info_topic_arg,
        rgbd_sync_arg,
        approx_rgbd_sync_arg,
        subscribe_rgbd_arg,
        rgbd_topic_arg,
        depth_scale_arg,
        rgbd_depth_scale_arg,
        rgbd_decimation_arg,
        compressed_arg,
        rgb_image_transport_arg,
        depth_image_transport_arg,
        gen_cloud_arg,
        gen_cloud_decimation_arg,
        gen_cloud_voxel_arg,
        subscribe_scan_arg,
        scan_cloud_topic_arg,
        subscribe_scan_descriptor_arg,
        scan_descriptor_topic_arg,
        scan_deskewing_arg,
        scan_deskewing_slerp_arg,
        scan_cloud_max_points_arg,
        scan_cloud_filtered_arg,
        gen_scan_arg,
        gen_depth_arg,
        gen_depth_decimation_arg,
        gen_depth_fill_holes_size_arg,
        gen_depth_fill_iterations_arg,
        gen_depth_fill_holes_error_arg,
        visual_odometry_arg,
        icp_odometry_arg,
        odom_topic_arg,
        vo_frame_id_arg,
        publish_tf_odom_arg,
        odom_tf_angular_variance_arg,
        odom_tf_linear_variance_arg,
        odom_args_arg,
        odom_sensor_sync_arg,
        odom_guess_frame_id_arg,
        odom_guess_min_translation_arg,
        odom_guess_min_rotation_arg,
        odom_max_rate_arg,
        odom_expected_rate_arg,
        imu_topic_arg,
        wait_imu_to_init_arg,
        use_odom_features_arg,
        scan_cloud_assembling_arg,
        scan_cloud_assembling_time_arg,
        scan_cloud_assembling_max_clouds_arg,
        scan_cloud_assembling_fixed_frame_arg,
        scan_cloud_assembling_voxel_size_arg,
        scan_cloud_assembling_range_min_arg,
        scan_cloud_assembling_range_max_arg,
        scan_cloud_assembling_noise_radius_arg,
        scan_cloud_assembling_noise_min_neighbors_arg,
        subscribe_user_data_arg,
        user_data_topic_arg,
        user_data_async_topic_arg,
        gps_topic_arg,
        tag_topic_arg,
        tag_linear_variance_arg,
        tag_angular_variance_arg,
        fiducial_topic_arg,
        rgb_topic_relay_arg,
        depth_topic_relay_arg,
        left_image_topic_relay_arg,
        right_image_topic_relay_arg,
        rgbd_topic_relay_arg,
        
        frequency_arg,
        base_link_frame_arg,
        odom0_arg,
        odom0_config_arg,
        odom0_relative_arg,
        odom0_pose_rejection_threshold_arg,
        odom0_twist_rejection_threshold_arg,
        imu0_arg,
        imu0_config_arg,
        imu0_differential_arg,
        imu0_relative_arg,
        use_control_arg,
        rs2_camera_launch_include,
        imu_filter_madgwick_node,
        rtabmap_ros_launch_include,
        robot_localization_launch_include
    ])