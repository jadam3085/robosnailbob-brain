from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robosnailbob_brain',
            executable='voice_io_node',
            name='voice_io_node',
            output='screen',
            parameters=[{
                'vad_aggressiveness': 3,
                'silence_ms': 800,
                'beam_size': 1,
            }]
        ),
        Node(
            package='robosnailbob_brain',
            executable='llm_brain_node',
            name='llm_brain_node',
            output='screen',
            parameters=[{
                'model': 'llama3.2:3b',
                'num_ctx': 1024,
                'num_predict': 80,
                'temperature': 0.7,
                'keep_alive': '30m',
            }]
        ),
    ])
