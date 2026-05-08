#!/usr/bin/env python3
"""
llm_brain_node — LLM reasoning with robot state context injection

Subscribes: /voice/input    (std_msgs/String)
            /battery/state  (sensor_msgs/BatteryState)
            /odom           (nav_msgs/Odometry)
Publishes:  /voice/output   (std_msgs/String) — sentence chunks as generated
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry

import requests, json, re, threading, os, yaml
from ament_index_python.packages import get_package_share_directory

OLLAMA_URL   = 'http://localhost:11434/api/chat'
SENTENCE_END = re.compile(r'([^.!?]*[.!?])\s*')


class LLMBrainNode(Node):

    def __init__(self):
        super().__init__('llm_brain_node')

        self.declare_parameter('model',       'llama3.2:3b')
        self.declare_parameter('num_ctx',     1024)
        self.declare_parameter('num_predict', 80)
        self.declare_parameter('temperature', 0.7)
        self.declare_parameter('keep_alive',  '30m')
        self.declare_parameter('personality', 'default')

        self.model       = self.get_parameter('model').value
        self.num_ctx     = self.get_parameter('num_ctx').value
        self.num_predict = self.get_parameter('num_predict').value
        self.temperature = self.get_parameter('temperature').value
        self.keep_alive  = self.get_parameter('keep_alive').value

        personality_name  = self.get_parameter('personality').value
        self.personality  = self._load_personality(personality_name)

        # Robot state — None until a message arrives
        self.battery_pct  = None
        self.battery_volt = None
        self.odom_x       = None
        self.odom_y       = None

        self.conversation_history = []
        self.busy = threading.Lock()

        # Publishers / subscribers
        self.pub = self.create_publisher(String, '/voice/output', 10)
        self.create_subscription(
            String,       '/voice/input',   self._on_voice_input, 10)
        self.create_subscription(
            BatteryState, '/battery/state', self._on_battery,     10)
        self.create_subscription(
            Odometry,     '/odom',          self._on_odom,        10)

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
                'style':     'Reply in one sentence.',
                'honesty':   'Never invent sensor data you do not have.',
                'knowledge': ''
            }

    def _build_system_prompt(self) -> str:
        p = self.personality
        base = ' '.join(filter(None, [
            p.get('identity',  ''),
            p.get('style',     ''),
            p.get('honesty',   ''),
            p.get('knowledge', ''),
        ]))

        # Only inject sensor values that have actually arrived
        state_lines = []
        if self.battery_pct is not None:
            state_lines.append(
                f'Battery: {self.battery_pct:.0f}% ({self.battery_volt:.1f}V)')
        if self.odom_x is not None:
            state_lines.append(
                f'Position: x={self.odom_x:.1f}m  y={self.odom_y:.1f}m (odom frame)')

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
            if len(self.conversation_history) > 12:
                self.conversation_history = self.conversation_history[-12:]

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
                    buffer     += token
                    full_reply += token

                    while True:
                        m = SENTENCE_END.match(buffer)
                        if not m:
                            break
                        sentence = m.group(1).strip()
                        buffer   = buffer[m.end():]
                        if sentence:
                            self.get_logger().info(f'-> {sentence}')
                            self._publish(sentence)

                    if chunk.get('done'):
                        break

            if buffer.strip():
                self.get_logger().info(f'-> {buffer.strip()}')
                self._publish(buffer.strip())

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
