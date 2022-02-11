"""Launch file for Epson G320/G354/G364 imu_node for epson_imu_spi_ros2 ROS2 package"""

from launch import LaunchDescription
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name="frame_id",
            default_value="imu_link",
            description="imu message frame_id field"),
        DeclareLaunchArgument(
            name="imu_topic",
            default_value="/epson_imu/data_raw",
            description="topic name for publishing imu messages."),
        DeclareLaunchArgument(
            name="burst_polling_rate",
            default_value="4000.0",
            description="Set to atleast 2x the output rate of IMU. Should not need to change."),
        DeclareLaunchArgument(
            name="imu_dout_rate",
            # value: output rate (Hz)
            # 0: 2000
            # 1: 1000
            # 2: 500
            # 3: 250
            # 4: 125
            # 5: 62.5
            # 6: 31.25
            # 7: 15.625
            # 8: 400
            # 9: 200
            # 10: 100
            # 11: 80
            # 12: 40
            # 13: 20
            default_value="4",
            description="Sets data output rate of IMU"),
        DeclareLaunchArgument(
            name="imu_filter_sel",
            # value: Filter Setting
            # 0: bypass
            # 1: Moving Average TAP2
            # 2: Moving Average TAP4
            # 3: Moving Average TAP8
            # 4: Moving Average TAP16
            # 5: Moving Average TAP32
            # 6: Moving Average TAP64
            # 7: Moving Average TAP128
            # 8: KAISER TAP32 Fc=50 Hz
            # 9: KAISER TAP32 Fc=100 Hz
            # 10: KAISER TAP32 Fc=200 Hz
            # 11: KAISER TAP32 Fc=400 Hz
            # 12: KAISER TAP64 Fc=50 Hz
            # 13: KAISER TAP64 Fc=100 Hz
            # 14: KAISER TAP64 Fc=200 Hz
            # 15: KAISER TAP64 Fc=400 Hz
            # 16: KAISER TAP128 Fc=50 Hz
            # 17: KAISER TAP128 Fc=100 Hz
            # 18: KAISER TAP128 Fc=200 Hz
            # 19: KAISER TAP128 Fc=400 Hz
            default_value="6",
            description="Sets the IMU filter"),
        DeclareLaunchArgument(
            name="output_32bit_en",
            default_value="true",
            description="Enables all sensor data output in 32-bit resolution or 16-bit resolution."),
        DeclareLaunchArgument(
            name="time_correction_en",
            default_value="false",
            description="Enables using IMU external counter reset function for timestamp with external 1PPS connected to IMU input pin for GPIO2/EXT"),
        DeclareLaunchArgument(
            name="ext_trigger_en",
            default_value="false",
            description="Enables using IMU external trigger function for sending IMU samples with external trigger signal connected to IMU input pin for GPIO2/EXT"),
        launch_ros.actions.Node(
            package='epson_imu_spi_ros2',
            node_executable='imu_node',
            output='screen',
            parameters=[{'__log_level': 'INFO',
                'frame_id': LaunchConfiguration("frame_id"),
                'imu_topic': LaunchConfiguration("imu_topic"),
                'burst_polling_rate': LaunchConfiguration("burst_polling_rate"),
                'imu_dout_rate': LaunchConfiguration("imu_dout_rate"),
                'imu_filter_sel': LaunchConfiguration("imu_filter_sel"),
                'output_32bit_en': LaunchConfiguration("output_32bit_en"),
                'time_correction_en': LaunchConfiguration("time_correction_en"),
                'ext_trigger_en': LaunchConfiguration("ext_trigger_en"),
            }])
    ])
