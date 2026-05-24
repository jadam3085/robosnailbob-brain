#!/usr/bin/env python3
"""
llm_brain_node — LLM reasoning with robot state context injection

Subscribes: /voice/input    (std_msgs/String)
            /battery/state  (sensor_msgs/BatteryState)
            /odom           (nav_msgs/Odometry)
            /imu/data       (sensor_msgs/Imu)        — heading/orientation
            /fix            (sensor_msgs/NavSatFix)  — GPS position
Publishes:  /voice/output   (std_msgs/String) — sentence chunks as generated
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import BatteryState, Imu, NavSatFix
from nav_msgs.msg import Odometry

import requests, json, re, threading, os, yaml
from ament_index_python.packages import get_package_share_directory

OLLAMA_URL = 'http://localhost:11434/api/chat'

# Require whitespace after sentence-ending punctuation to avoid splitting on
# abbreviations (Mr., 2.5, etc.) mid-stream.
SENTENCE_END = re.compile(r'([^.!?]*[.!?])\s+')


class LLMBrainNode(Node):

    def __init__(self):
        super().__init__('llm_brain_node')

        self.declare_parameter('model',       'llama3.2:3b')
        self.declare_parameter('num_ctx',     1024)
        self.declare_parameter('num_predict', 100)
        self.declare_parameter('temperature', 0.7)
        self.declare_parameter('keep_alive',  '60m')
        self.declare_parameter('personality', 'default')

        self.model       = self.get_parameter('model').value
        self.num_ctx     = self.get_parameter('num_ctx').value
        self.num_predict = self.get_parameter('num_predict').value
        self.temperature = self.get_parameter('temperature').value
        self.keep_alive  = self.get_parameter('keep_alive').value

        personality_name  = self.get_parameter('personality').value
        self.personality  = self._load_personality(personality_name)

        # Robot state — None until a message arrives on that topic
        self.battery_pct  = None
        self.battery_volt = None
        self.odom_x       = None
        self.odom_y       = None
        self.odom_speed   = None   # m/s forward speed
        self.odom_angular = None   # rad/s yaw rate
        self.heading_deg  = None   # compass bearing 0-360 from IMU
        self.gps_lat      = None
        self.gps_lon      = None
        self.gps_alt      = None
        self.gps_valid    = False

        self.conversation_history = []
        self.busy = threading.Lock()

        # Publishers / subscribers
        self.pub = self.create_publisher(String, '/voice/output', 10)
        self.create_subscription(
            String,      '/voice/input',   self._on_voice_input, 10)
        self.create_subscription(
            BatteryState, '/battery/state', self._on_battery,    10)
        self.create_subscription(
            Odometry,     '/odom',          self._on_odom,        10)
        self.create_subscription(
            Imu,          '/imu/data',      self._on_imu,         10)
        self.create_subscription(
            NavSatFix,    '/fix',           self._on_gps,         10)

        self.get_logger().info(
            f'llm_brain_node ready — model: {self.model}, '
            f'personality: {personality_name}')

    # ── Personality ───────────────────────────────────────────────────────────

    def _load_personality(self, name: str) -> dict:
        try:
            pkg_share = get_package_share_directory('robosnailbob_brain')
            path = os.path.join(
                pkg_share, 'config', 'personalities', f'{name}.yaml')
            with open(path, 'r') as f:
                p = yaml.safe_load(f)
            self.get_logger().info(f'Loaded personality: {name}')
            return p
        except Exception as e:
            self.get_logger().warn(
                f'Could not load personality "{name}": {e} — using fallback')
            return {
                'identity':  'You are RoboSnailBob, an outdoor patrol robot.',
                'style':     'Keep replies short and conversational.',
                'honesty':   'Never invent sensor data you do not have.',
                'knowledge': ''
            }

    def _bearing_to_cardinal(self, deg: float) -> str:
        dirs = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW']
        return dirs[round(deg / 45) % 8]

    def _build_system_prompt(self) -> str:
        p = self.personality
        base = ' '.join(filter(None, [
            p.get('identity',  ''),
            p.get('style',     ''),
            p.get('honesty',   ''),
            p.get('knowledge', ''),
        ]))

        # Only inject sensor values that have actually arrived on their topics
        state_lines = []
        if self.battery_pct is not None:
            state_lines.append(
                f'Battery: {self.battery_pct:.0f}% ({self.battery_volt:.1f}V)')

        if self.heading_deg is not None:
            cardinal = self._bearing_to_cardinal(self.heading_deg)
            state_lines.append(f'Heading: {self.heading_deg:.0f}° ({cardinal})')

        if self.odom_x is not None:
            state_lines.append(
                f'Position: x={self.odom_x:.1f}m  y={self.odom_y:.1f}m (odom frame)')

        if self.odom_speed is not None:
            if self.odom_speed < 0.05:
                motion = 'stopped'
            else:
                turning = ''
                if self.odom_angular is not None and abs(self.odom_angular) > 0.05:
                    turning = ', turning'
                motion = f'{self.odom_speed:.2f} m/s{turning}'
            state_lines.append(f'Speed: {motion}')

        if self.gps_valid and self.gps_lat is not None:
            state_lines.append(
                f'GPS: {self.gps_lat:.6f}°N  {self.gps_lon:.6f}°E'
                f'  alt={self.gps_alt:.1f}m')

        if state_lines:
            base += '\n\nCURRENT SENSOR DATA:\n' + '\n'.join(state_lines)
        else:
            base += '\n\nCURRENT SENSOR DATA: No sensor data available yet.'

        return base

    # ── Subscribers ───────────────────────────────────────────────────────────

    def _on_battery(self, msg: BatteryState):
        self.battery_pct  = msg.percentage * 100.0
        self.battery_volt = msg.voltage

    def _on_odom(self, msg: Odometry):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.odom_speed   = math.sqrt(vx * vx + vy * vy)
        self.odom_angular = msg.twist.twist.angular.z

    def _on_imu(self, msg: Imu):
        q = msg.orientation
        # Yaw from quaternion (ENU frame: 0=East, CCW positive)
        yaw_rad = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        # Convert ENU yaw → compass bearing (North=0, clockwise positive)
        self.heading_deg = (90.0 - math.degrees(yaw_rad)) % 360.0

    def _on_gps(self, msg: NavSatFix):
        # status.status >= 0 means at least a basic GPS fix (-1 = no fix)
        self.gps_valid = msg.status.status >= 0
        if self.gps_valid:
            self.gps_lat = msg.latitude
            self.gps_lon = msg.longitude
            self.gps_alt = msg.altitude

    def _on_voice_input(self, msg: String):
        text = msg.data.strip()
        if not text:
            return
        if not self.busy.acquire(blocking=False):
            self.get_logger().warn('Still processing previous input — ignoring.')
            return
        threading.Thread(target=self._process, args=(text,), daemon=True).start()

    # ── LLM pipeline ─────────────────────────────────────────────────────────

    def _process(self, user_text: str):
        try:
            self.get_logger().info(f'Processing: {user_text}')

            self.conversation_history.append(
                {'role': 'user', 'content': user_text})
            if len(self.conversation_history) > 6:
                self.conversation_history = self.conversation_history[-6:]

            payload = {
                'model': self.model,
                'messages': (
                    [{'role': 'system', 'content': self._build_system_prompt()}]
                    + self.conversation_history
                ),
                'stream':     True,
                'keep_alive': self.keep_alive,
                'options': {
                    'num_ctx':     self.num_ctx,
                    'num_predict': self.num_predict,
                    'temperature': self.temperature,
                    'stop':        ['\nUser:', '\nAssistant:'],
                }
            }

            full_reply = ''
            buffer     = ''

            with requests.post(
                OLLAMA_URL, json=payload, stream=True, timeout=60
            ) as resp:
                resp.raise_for_status()
                for line in resp.iter_lines():
                    if not line:
                        continue
                    chunk = json.loads(line)
                    token = chunk.get('message', {}).get('content', '')
                    # Normalize whitespace in-stream to keep TTS clean
                    token = token.replace('\n', ' ')
                    buffer     += token
                    full_reply += token

                    while True:
                        m = SENTENCE_END.match(buffer)
                        if not m:
                            break
                        sentence = ' '.join(m.group(1).split())
                        buffer   = buffer[m.end():]
                        if sentence:
                            self.get_logger().info(f'-> {sentence}')
                            self._publish(sentence)

                    if chunk.get('done'):
                        break

            # Flush any remaining text that didn't end with whitespace
            remainder = ' '.join(buffer.split())
            if remainder:
                self.get_logger().info(f'-> {remainder}')
                self._publish(remainder)

            self.conversation_history.append(
                {'role': 'assistant', 'content': full_reply.strip()})

        except Exception as e:
            self.get_logger().error(f'LLM error: {e}')
        finally:
            self.busy.release()

    def _publish(self, text: str):
        msg = String()
        msg.data = text
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LLMBrainNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
