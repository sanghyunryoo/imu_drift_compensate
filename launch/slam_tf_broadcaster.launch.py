from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # ✅ Launch Arguments 설정 (IMU 토픽 & use_sim_time)
    imu_topic = LaunchConfiguration('imu_topic')
    cloud_in_topic = LaunchConfiguration('cloud_in_topic')
    cloud_out_topic = LaunchConfiguration('cloud_out_topic')

    use_sim_time = LaunchConfiguration('use_sim_time')

    translation_x = LaunchConfiguration('translation_x')
    translation_y = LaunchConfiguration('translation_y')
    translation_z = LaunchConfiguration('translation_z')

    return LaunchDescription([
        # ✅ IMU 토픽을 설정할 수 있도록 설정 (기본값: "/robot0/imu")
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/robot0/imu',
            description='IMU topic name for TF broadcaster'
        ),
        DeclareLaunchArgument(
            'cloud_in_topic',
            default_value='/robot0/point_cloud2',
            description='Pointcloud in topic name for TF broadcaster'
        ),
        
        DeclareLaunchArgument(
            'cloud_out_topic',
            default_value='/robot0/point_cloud2_adjusted',
            description='Pointcloud out topic name for TF broadcaster'
        ),
        # ✅ 시뮬레이션 사용 여부 설정 (기본값: false)
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time (Gazebo or other simulators)'
        ),
        # ✅ Translation X 값
        DeclareLaunchArgument(
            'translation_x',
            default_value='0.0',
            description='X-axis translation offset for PointCloud'
        ),
        # ✅ Translation Y 값
        DeclareLaunchArgument(
            'translation_y',
            default_value='0.0',
            description='Y-axis translation offset for PointCloud'
        ),
        # ✅ Translation Z 값
        DeclareLaunchArgument(
            'translation_z',
            default_value='0.66288',
            description='Z-axis translation offset for PointCloud'
        ),

        # ✅ `slam_tf_broadcaster` 실행 시 `use_sim_time`과 `imu_topic` 전달
        Node(
            package='slam_tf_broadcaster',
            executable='slam_tf_broadcaster',
            name='slam_tf_broadcaster',
            output='screen',
            parameters=[{
                "imu_topic": imu_topic,
                "cloud_in_topic": cloud_in_topic,
                "cloud_out_topic": cloud_out_topic,
                "use_sim_time": use_sim_time,  
                "translation_x": translation_x,
                "translation_y": translation_y,
                "translation_z": translation_z,
            }]
        )
    ])
