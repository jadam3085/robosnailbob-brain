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
                # VAD 2 = less aggressive silence cutting than 3,
                # gives more natural speech pause tolerance
                'vad_aggressiveness': 2,
                'beam_size': 1,
                # Custom wake word — place hey_snailbob.onnx in OWW_MODELS_DIR
                # to activate. Falls back to hey_jarvis until then.
                # Train: https://github.com/dscripka/openWakeWord#custom-models
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
                # llama3.2:3b — solid for casual banter on CPU
                # For faster responses: try llama3.2:1b or gemma2:2b
                # (run: ollama pull llama3.2:1b)
                'model': 'llama3.2:3b',
                'num_ctx': 1024,
                'num_predict': 60,   # ~2 short sentences; hard cap on runaway replies
                'temperature': 0.7,
                'keep_alive': '60m',  # was 30m — keeps model warm longer
            }]
        ),
    ])
