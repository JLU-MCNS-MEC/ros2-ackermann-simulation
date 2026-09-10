"""Start visual demonstration recording or policy shadow inference."""

from pathlib import Path

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch(context: LaunchContext):
    """Validate file arguments and create the selected visual node."""
    mode = LaunchConfiguration('mode').perform(context)
    common = {
        'use_sim_time': True,
        'image_topic': LaunchConfiguration('image_topic'),
        'teacher_topic': LaunchConfiguration('teacher_topic'),
        'plan_topic': LaunchConfiguration('plan_topic'),
    }
    if mode == 'record':
        parameters = {
            **common,
            'output_directory': LaunchConfiguration('dataset'),
            'sample_rate': LaunchConfiguration('rate'),
        }
        executable = 'visual_dataset_recorder'
    elif mode == 'shadow':
        model = Path(LaunchConfiguration('model').perform(context))
        if not model.is_file():
            raise FileNotFoundError(
                f'visual policy model does not exist: {model}'
            )
        parameters = {
            **common,
            'model_path': str(model),
            'metrics_file': LaunchConfiguration('metrics'),
            'inference_rate': LaunchConfiguration('rate'),
        }
        executable = 'visual_policy'
    else:
        raise ValueError("mode must be 'record' or 'shadow'")
    return [
        Node(
            package='ackermann_line_following_controller',
            executable=executable,
            name=executable,
            output='screen',
            parameters=[parameters],
        )
    ]


def generate_launch_description() -> LaunchDescription:
    """Declare the model-independent visual navigation workflow."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'mode', default_value='record', choices=['record', 'shadow']
            ),
            DeclareLaunchArgument(
                'dataset', default_value='/tmp/visual_navigation_dataset'
            ),
            DeclareLaunchArgument('model', default_value=''),
            DeclareLaunchArgument(
                'metrics',
                default_value='/tmp/visual_navigation_shadow_metrics.json',
            ),
            DeclareLaunchArgument('rate', default_value='5.0'),
            DeclareLaunchArgument('image_topic', default_value='/rgbd/image'),
            DeclareLaunchArgument(
                'teacher_topic', default_value='/cmd_vel_smoothed'
            ),
            DeclareLaunchArgument('plan_topic', default_value='/plan'),
            OpaqueFunction(function=_launch),
        ]
    )
