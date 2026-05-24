import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robosnailbob_brain')

    # ── DDS tuning ────────────────────────────────────────────────────────────
    # Point CycloneDDS at our local-only config:
    #   - multicast disabled (WiFi APs drop multicast → remote viewer STALL)
    #   - large UDP socket buffers (prevent camera burst frame loss)
    #   - faster heartbeats (detect dead subscribers quickly)
    #   - unicast localhost discovery only
    cyclone_cfg = os.path.join(pkg_share, 'config', 'dds', 'cyclone_local.xml')
    set_dds_uri = SetEnvironmentVariable(
        name='CYCLONEDDS_URI',
        value=f'file://{cyclone_cfg}',
    )

    # ── Nodes ─────────────────────────────────────────────────────────────────

    voice_io = Node(
        package='robosnailbob_brain',
        executable='voice_io_node',
        name='voice_io_node',
        output='screen',
        parameters=[{
            'vad_aggressiveness': 2,
            'beam_size':          1,
            'wakeword_model':     'hey_snailbob',
            'wakeword_threshold': 0.5,
        }],
    )

    llm_brain = Node(
        package='robosnailbob_brain',
        executable='llm_brain_node',
        name='llm_brain_node',
        output='screen',
        parameters=[{
            # 1b is ~3x faster than 3b on CPU with acceptable quality for banter
            # run: ollama pull llama3.2:1b
            'model':       'llama3.2:1b',
            'num_ctx':     512,    # smaller = faster prefill
            'num_predict': 50,     # hard cap; 2 short sentences fits easily
            'temperature': 0.8,
            'keep_alive':  '60m',
        }],
    )

    # ── Camera watchdog ───────────────────────────────────────────────────────
    # Monitors camera topics for frame stalls.  Uses BEST_EFFORT QoS so it
    # never slows down camera publishers.  Publishes /camera/health (JSON).
    # Topic list covers Kinect2 (depth primary), RealSense, and USB cams.
    # Adjust this list to match whichever cameras are active on your robot.
    camera_watchdog = Node(
        package='robosnailbob_brain',
        executable='camera_watchdog_node',
        name='camera_watchdog_node',
        output='screen',
        parameters=[{
            'stall_timeout_s':  5.0,    # declare stall after 5 s with no frame
            'check_interval_s': 2.0,    # evaluate health every 2 s
            'topics': [
                # Kinect2 — depth is most reliable, list first
                '/kinect2/sd/image_depth_rect',
                '/kinect2/hd/image_color_rect',
                # RealSense / generic depth cam
                '/camera/depth/image_rect_raw',
                '/camera/color/image_raw',
                # Generic USB cam fallback
                '/usb_cam/image_raw',
            ],
        }],
    )

    # ── Network monitor ───────────────────────────────────────────────────────
    # Pings the local gateway to detect WiFi/connectivity stalls.
    # Logs WARN when loss >= 50% or RTT >= 500 ms (configurable).
    # Publishes /network/health (JSON) and /network/stall (Bool).
    # ping_host defaults to the detected default gateway if left empty.
    network_monitor = Node(
        package='robosnailbob_brain',
        executable='network_monitor_node',
        name='network_monitor_node',
        output='screen',
        parameters=[{
            'ping_host':        '',      # auto-detect gateway
            'ping_interval_s':  10.0,   # check every 10 s (4-ping burst each time)
            'stall_loss_pct':   50.0,   # STALL if >50% packet loss
            'stall_latency_ms': 500.0,  # STALL if avg RTT >500 ms
        }],
    )

    return LaunchDescription([
        set_dds_uri,
        voice_io,
        llm_brain,
        camera_watchdog,
        network_monitor,
    ])
