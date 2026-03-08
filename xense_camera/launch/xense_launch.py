from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    camera_name = LaunchConfiguration('camera_name').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)

    parameters = [
        {
            'enable_raw': LaunchConfiguration('enable_raw'),
            'enable_rectified': LaunchConfiguration('enable_rectified'),
            'enable_diff_single': LaunchConfiguration('enable_diff_single'),
            'enable_diff_continuous': LaunchConfiguration('enable_diff_continuous'),
            'device_serial': LaunchConfiguration('device_serial'),
            'device_index': LaunchConfiguration('device_index'),
            'inference_backend': LaunchConfiguration('inference_backend'),
            'use_gpu': LaunchConfiguration('use_gpu'),
            'camera_frame_id': LaunchConfiguration('camera_frame_id'),
            'publish_tf': LaunchConfiguration('publish_tf'),
        }
    ]

    if params_file:
        parameters = [params_file] + parameters

    node = Node(
        package='xense_camera',
        executable='xense_camera_node',
        name=camera_name,
        parameters=parameters,
        output='screen',
        emulate_tty=True,
    )

    return [node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_name',
            default_value='xense_camera',
            description='Name of the camera node'),
        DeclareLaunchArgument(
            'params_file',
            default_value='',
            description='Path to a YAML parameters file (optional)'),
        DeclareLaunchArgument(
            'device_serial',
            default_value='',
            description='Xense sensor serial number. Empty = auto-detect.'),
        DeclareLaunchArgument(
            'device_index',
            default_value='-1',
            description='V4L2 device index. -1 = auto-detect.'),
        DeclareLaunchArgument(
            'enable_raw',
            default_value='false',
            description='Publish raw 640x480 BGR8 frames on ~/raw/image_raw'),
        DeclareLaunchArgument(
            'enable_rectified',
            default_value='true',
            description='Publish 400x700 rectified BGR8 frames on ~/rectified/image'),
        DeclareLaunchArgument(
            'enable_diff_single',
            default_value='false',
            description='Publish SingleInference diff on ~/diff/single/image (fast, reference inferred once)'),
        DeclareLaunchArgument(
            'enable_diff_continuous',
            default_value='false',
            description='Publish PerFrameInference diff on ~/diff/continuous/image (accurate, per-frame NN)'),
        DeclareLaunchArgument(
            'inference_backend',
            default_value='Auto',
            description='Inference backend: Auto, CPU, ONNX, MIGraphX, RKNN, CoreML, OpenVINO, DirectML'),
        DeclareLaunchArgument(
            'use_gpu',
            default_value='true',
            description='Use GPU acceleration for inference if available'),
        DeclareLaunchArgument(
            'camera_frame_id',
            default_value='xense_camera_link',
            description='TF frame ID for all published sensor data'),
        DeclareLaunchArgument(
            'publish_tf',
            default_value='true',
            description='Publish a static transform from world -> camera_frame_id'),

        OpaqueFunction(function=launch_setup),
    ])
