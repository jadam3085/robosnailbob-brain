#!/usr/bin/env python3
"""
network_monitor_node — Monitor network/WiFi health and detect STALL conditions.

Pings the local default gateway every ping_interval_s seconds.
Monitors WiFi link quality via `iw`.
Publishes /network/health (String, JSON) and /network/stall (Bool).
Logs WARN on STALL transitions and INFO on recovery.

STALL is declared when packet loss >= stall_loss_pct OR
average RTT >= stall_latency_ms.
"""

import json
import re
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


# ── helpers ───────────────────────────────────────────────────────────────────

def _default_gateway() -> str:
    """Return the default gateway IP, or '192.168.1.1' as fallback."""
    try:
        out = subprocess.check_output(
            ['ip', 'route', 'show', 'default'], text=True, timeout=3)
        m = re.search(r'default via (\S+)', out)
        if m:
            return m.group(1)
    except Exception:
        pass
    return '192.168.1.1'


def _ping(host: str, count: int = 4, wait_s: int = 2) -> dict:
    """Run ping and parse loss% and avg RTT."""
    try:
        res = subprocess.run(
            ['ping', '-c', str(count), '-W', str(wait_s), host],
            capture_output=True, text=True,
            timeout=count * (wait_s + 1) + 5,
        )
        out = res.stdout
        loss_pct = 100.0
        avg_ms: float | None = None

        for line in out.splitlines():
            m = re.search(r'([\d.]+)%\s+packet loss', line)
            if m:
                loss_pct = float(m.group(1))
            # Linux: "rtt min/avg/max/mdev = 1.2/3.4/5.6/0.7 ms"
            # macOS: "round-trip min/avg/max/stddev = 1.2/3.4/5.6/0.7 ms"
            m = re.search(r'[\d.]+/([\d.]+)/[\d.]+', line)
            if m:
                avg_ms = float(m.group(1))

        return {'host': host, 'loss_pct': loss_pct, 'avg_ms': avg_ms, 'error': None}

    except subprocess.TimeoutExpired:
        return {'host': host, 'loss_pct': 100.0, 'avg_ms': None, 'error': 'timeout'}
    except Exception as exc:
        return {'host': host, 'loss_pct': 100.0, 'avg_ms': None, 'error': str(exc)}


def _wifi_info() -> dict:
    """Return WiFi interface, signal dBm, and tx Mbps; empty dict if unavailable."""
    try:
        iface_out = subprocess.check_output(
            ['sh', '-c',
             'iw dev 2>/dev/null | awk \'/Interface/{print $2; exit}\''],
            text=True, timeout=3,
        ).strip()
        if not iface_out:
            return {}
        link_out = subprocess.check_output(
            ['iw', 'dev', iface_out, 'link'],
            text=True, timeout=3,
        )
        info: dict = {'interface': iface_out}
        m = re.search(r'signal:\s*([-\d.]+)\s*dBm', link_out)
        if m:
            info['signal_dbm'] = float(m.group(1))
        m = re.search(r'tx bitrate:\s*([\d.]+)', link_out)
        if m:
            info['tx_mbps'] = float(m.group(1))
        return info
    except Exception:
        return {}


# ── node ──────────────────────────────────────────────────────────────────────

class NetworkMonitorNode(Node):

    def __init__(self):
        super().__init__('network_monitor_node')

        self.declare_parameter('ping_host',        '')
        self.declare_parameter('ping_interval_s',  10.0)
        self.declare_parameter('stall_loss_pct',   50.0)
        self.declare_parameter('stall_latency_ms', 500.0)

        host = self.get_parameter('ping_host').value or _default_gateway()
        self._host          = host
        self._interval      = self.get_parameter('ping_interval_s').value
        self._stall_loss    = self.get_parameter('stall_loss_pct').value
        self._stall_latency = self.get_parameter('stall_latency_ms').value
        self._prev_stall    = False

        self._health_pub = self.create_publisher(String, '/network/health', 10)
        self._stall_pub  = self.create_publisher(Bool,   '/network/stall',  10)

        threading.Thread(target=self._loop, daemon=True).start()

        self.get_logger().info(
            f'network_monitor_node ready — pinging {self._host} '
            f'every {self._interval}s'
        )

    def _is_stall(self, ping: dict) -> bool:
        if ping['loss_pct'] >= self._stall_loss:
            return True
        if ping['avg_ms'] is not None and ping['avg_ms'] >= self._stall_latency:
            return True
        return False

    def _loop(self):
        while rclpy.ok():
            ping = _ping(self._host)
            wifi = _wifi_info()
            stall = self._is_stall(ping)

            if stall and not self._prev_stall:
                self.get_logger().warn(
                    f'Network STALL — host={self._host} '
                    f'loss={ping["loss_pct"]:.0f}% '
                    f'avg_rtt={ping["avg_ms"]}ms '
                    f'wifi={wifi.get("signal_dbm", "N/A")}dBm '
                    f'tx={wifi.get("tx_mbps", "N/A")}Mbps'
                )
            elif not stall and self._prev_stall:
                self.get_logger().info(
                    f'Network RECOVERED — host={self._host} '
                    f'loss={ping["loss_pct"]:.0f}% avg_rtt={ping["avg_ms"]}ms'
                )
            self._prev_stall = stall

            report = {
                'ts':    time.time(),
                'stall': stall,
                'ping':  ping,
                'wifi':  wifi,
            }
            h_msg = String()
            h_msg.data = json.dumps(report)
            self._health_pub.publish(h_msg)

            s_msg = Bool()
            s_msg.data = stall
            self._stall_pub.publish(s_msg)

            time.sleep(self._interval)


def main(args=None):
    rclpy.init(args=args)
    node = NetworkMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
