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
                'vad_aggressiveness': 2,
                'beam_size': 1,
                'wakeword_model': 'hey_snailbob',
                'wakeword_threshold': 0.5,
            }]
        ),
        Node(
            package='robosnailbob_brain',
            executable='llm_brain_node',
            name='llm_brain_node',
            output='screen',
            parameters=[{
                # 1b is ~3x faster than 3b on CPU with acceptable quality for banter
                # run: ollama pull llama3.2:1b
                'model': 'llama3.2:1b',
                'num_ctx': 2048,      # KV cache is RAM-cheap (64GB box); buys conversation memory
                'num_predict': 50,    # hard cap; 2 short sentences fits easily
                'temperature': 0.8,
                'keep_alive': '-1m',  # never evict — reload from disk was the "slow to respond" pause
                                      # (must be '-1m' not '-1': Ollama parses it as a Go duration)
            }]
        ),
    ])
