"""Independent RGB-D SLAM for the x500_depth simulation or a matching bag."""

import fcntl
from pathlib import Path
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    supplied = arg('database_path')
    db = (Path(supplied).expanduser() if supplied else
          Path(tempfile.mkdtemp(prefix='powerplant-slam-')) / 'rtabmap.db')
    if not db.is_absolute():
        raise ValueError('database_path must be absolute')
    db.parent.mkdir(parents=True, exist_ok=True)
    # Keep the advisory lock alive through launch, including the interval before
    # RTAB-Map first opens its SQLite file. The OS releases it on launcher exit.
    lock = (db.parent / (db.name + '.lock')).open('a')
    try:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        if db.exists():
            raise FileExistsError(f'Select a fresh database_path: {db}')
    except Exception:
        lock.close()
        raise
    release_lock = RegisterEventHandler(OnShutdown(
        on_shutdown=[OpaqueFunction(function=lambda _: lock.close())]))

    use_sim_time = arg('use_sim_time') == 'true'
    loops = arg('loop_closure') == 'true'
    tf_topics = [('/tf', '/slam/tf'), ('/tf_static', '/slam/tf_static')]

    def node(package, executable, name, parameters=None, remappings=None, arguments=None):
        return Node(package=package, executable=executable, namespace='slam', name=name,
                    parameters=[{'use_sim_time': use_sim_time, **(parameters or {})}],
                    remappings=tf_topics + (remappings or []), arguments=arguments, output='screen')

    # Two Gazebo optical sensors share their pose in OakD-Lite/model.sdf.
    # x500_depth mount (.12,.03,.242) + camera sensor (.01233,-.03,.01878).
    camera_tf = node('tf2_ros', 'static_transform_publisher', 'camera_extrinsics', arguments=[
        '--x', '0.13233', '--y', '0', '--z', '0.26078',
        '--qx', '-0.5', '--qy', '0.5', '--qz', '-0.5', '--qw', '0.5',
        '--frame-id', 'slam_body', '--child-frame-id', 'slam_camera_optical'])
    inputs = node('px4_powerplant_slam', 'simulation_inputs', 'simulation_inputs',
                  parameters={'occlusion_start_s': float(arg('occlusion_start_s')),
                              'occlusion_duration_s': float(arg('occlusion_duration_s')),
                              'wait_for_capture_start': arg('wait_for_capture_start') == 'true'})
    attitude = node('imu_filter_madgwick', 'imu_filter_madgwick_node', 'inertial_attitude',
                    parameters={'use_mag': False, 'publish_tf': False, 'world_frame': 'enu'})
    image_topics = [('depth/image', 'depth_registered/image'), ('imu', 'imu/data'),
                    ('odom', 'odometry')]
    shared = {'frame_id': 'slam_body', 'approx_sync': False, 'qos': 2,
              'qos_camera_info': 2, 'qos_imu': 2, 'publish_tf': True}
    odometry = node('rtabmap_odom', 'rgbd_odometry', 'rgbd_odometry',
                    parameters={**shared, 'odom_frame_id': 'slam_odom', 'wait_imu_to_init': True,
                                'publish_null_when_lost': True},
                    remappings=image_topics)
    # Disabling loop detection changes the graph side only. Keep local VO,
    # keyframe selection, occupancy generation and neighbor links identical.
    loop_args = [] if loops else [
        '--Rtabmap/LoopThr', '2.0', '--RGBD/AggressiveLoopThr', '2.0',
        '--RGBD/ProximityBySpace', 'false', '--RGBD/ProximityByTime', 'false']
    graph = node('rtabmap_slam', 'rtabmap', 'rtabmap',
                 parameters={**shared, 'subscribe_depth': True, 'subscribe_rgb': True,
                             'subscribe_scan': False, 'subscribe_odom_info': False,
                             'map_frame_id': 'slam_map', 'database_path': str(db)},
                 remappings=image_topics, arguments=loop_args)
    return [release_lock, camera_tf, inputs, attitude, odometry, graph]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('database_path', default_value='',
                              description='Fresh absolute SQLite output path, default unique temp dir.'),
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('loop_closure', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('occlusion_start_s', default_value='-1.0'),
        DeclareLaunchArgument('occlusion_duration_s', default_value='0.0'),
        DeclareLaunchArgument('wait_for_capture_start', default_value='false',
                              choices=['true', 'false'],
                              description='Hold image and IMU inputs until the first occlusion marker.'),
        OpaqueFunction(function=_setup),
    ])
