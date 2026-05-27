#!/usr/bin/env python3
"""RoboSnailBob Teleop & Status Dashboard
Tabs: STATUS | TELEOP | DRIVE | NODES | CAMERA | MAP | SYSTEM | LOG
"""
import math
import json
import subprocess
import threading
import time
from collections import deque, OrderedDict

import numpy as np

try:
    import psutil
    HAS_PSUTIL = True
except ImportError:
    HAS_PSUTIL = False

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, NavSatFix, Image
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool, Int32, Float32MultiArray
import dearpygui.dearpygui as dpg

# ══════════════════════════════════════════════════════════════════════
# SHARED STATE
# ══════════════════════════════════════════════════════════════════════
_lock  = threading.Lock()
_state = {
    "battery_pct": 0.0, "voltage":    0.0,  "current":    0.0,
    "power":       0.0, "bat_last":   0.0,
    "odom_x":      0.0, "odom_y":     0.0,  "lin_vel":    0.0,
    "ang_vel":     0.0, "odom_yaw":   0.0,  "odom_last":  0.0,
    "gps_lat":     0.0, "gps_lon":    0.0,  "gps_alt":    0.0,
    "gps_fix":    -1,   "gps_last":   0.0,
    "gps_info":    {},
    "m_mode":    False, "m_throttle": 0,    "m_steering": 0,
    "m_left":      0,   "m_right":    0,    "m_sonics":   [0.0]*8,
    "m_last":      0.0, "ros_ok":    False,
}
STALE = 6.0

def GS():
    with _lock: return dict(_state)

def SS(**kw):
    with _lock: _state.update(kw)

# ══════════════════════════════════════════════════════════════════════
# HISTORY BUFFERS
# ══════════════════════════════════════════════════════════════════════
_t0      = time.time()
_vh      = deque(maxlen=120)
_ch      = deque(maxlen=120)
_gtrack  = deque(maxlen=500)
_gorigin = None
_hl      = threading.Lock()

def push_power(v, c):
    t = time.time() - _t0
    with _hl: _vh.append((t, v)); _ch.append((t, c))

def push_gps_pos(lat, lon):
    global _gorigin
    with _hl:
        if _gorigin is None: _gorigin = (lat, lon)
        lat0, lon0 = _gorigin
        R = 6_371_000.0
        x = (lon - lon0) * math.cos(math.radians(lat0)) * math.pi / 180 * R
        y = (lat - lat0) * math.pi / 180 * R
        _gtrack.append((x, y))

def snap_buf(b):
    with _hl: return list(b)

# ══════════════════════════════════════════════════════════════════════
# CAMERA STATE
# ══════════════════════════════════════════════════════════════════════
CAM_W, CAM_H    = 960, 540
CAM_DISP_W      = 520
CAM_DISP_H      = 293

_cam_lock       = threading.Lock()
_cam_rgba       = np.zeros(CAM_W * CAM_H * 4, dtype=np.float32)
_cam_rgba[3::4] = 0.3
_cam_dirty      = False
_cam_last_t     = 0.0
_cam_fps_val    = 0.0
_cam_connected  = False

def _process_camera(msg):
    global _cam_dirty, _cam_last_t, _cam_fps_val, _cam_connected, _cam_rgba
    now = time.time()
    dt  = now - _cam_last_t
    if dt < 0.067: return
    _cam_fps_val   = 1.0 / dt if dt > 0 else 0.0
    _cam_last_t    = now
    _cam_connected = True
    try:
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        enc = msg.encoding.lower()
        if enc in ('bgr8', 'bgr'):
            arr  = arr.reshape(msg.height, msg.width, 3)
            rgba = np.zeros((msg.height, msg.width, 4), dtype=np.float32)
            rgba[:,:,0] = arr[:,:,2] / 255.0
            rgba[:,:,1] = arr[:,:,1] / 255.0
            rgba[:,:,2] = arr[:,:,0] / 255.0
            rgba[:,:,3] = 1.0
        elif enc in ('rgb8', 'rgb'):
            arr  = arr.reshape(msg.height, msg.width, 3)
            rgba = np.zeros((msg.height, msg.width, 4), dtype=np.float32)
            rgba[:,:,0] = arr[:,:,0] / 255.0
            rgba[:,:,1] = arr[:,:,1] / 255.0
            rgba[:,:,2] = arr[:,:,2] / 255.0
            rgba[:,:,3] = 1.0
        else:
            return
        if msg.height != CAM_H or msg.width != CAM_W:
            yi   = (np.arange(CAM_H) * msg.height / CAM_H).astype(int).clip(0, msg.height-1)
            xi   = (np.arange(CAM_W) * msg.width  / CAM_W ).astype(int).clip(0, msg.width -1)
            rgba = rgba[np.ix_(yi, xi)]
        with _cam_lock:
            _cam_rgba  = rgba.flatten()
            _cam_dirty = True
    except Exception:
        pass

# ══════════════════════════════════════════════════════════════════════
# MAP STATE
# ══════════════════════════════════════════════════════════════════════
MAP_TEX          = 512
_map_lock        = threading.Lock()
_map_rgba        = np.full(MAP_TEX * MAP_TEX * 4, 0.42, dtype=np.float32)
_map_rgba[3::4]  = 1.0
_map_dirty       = False
_map_bounds      = [[0.0, 0.0], [10.0, 10.0]]
_map_info_cache  = None
_map_connected   = False
_path_lock       = threading.Lock()
_path_xs: list   = []
_path_ys: list   = []

def _process_map(msg):
    global _map_dirty, _map_bounds, _map_info_cache, _map_connected, _map_rgba
    w, h = msg.info.width, msg.info.height
    if w == 0 or h == 0: return
    _map_connected = True
    try:
        data = np.array(msg.data, dtype=np.int8).reshape(h, w)
        yi   = (np.arange(MAP_TEX) * h / MAP_TEX).astype(int).clip(0, h-1)
        xi   = (np.arange(MAP_TEX) * w / MAP_TEX).astype(int).clip(0, w-1)
        data = data[np.ix_(yi, xi)]
        data = np.flipud(data)
        rgba = np.zeros((MAP_TEX, MAP_TEX, 4), dtype=np.float32)
        rgba[data ==   0] = [0.88, 0.88, 0.88, 1.0]
        rgba[data == 100] = [0.10, 0.10, 0.10, 1.0]
        rgba[data ==  -1] = [0.42, 0.44, 0.50, 1.0]
        ox  = msg.info.origin.position.x
        oy  = msg.info.origin.position.y
        res = msg.info.resolution
        with _map_lock:
            _map_rgba       = rgba.flatten()
            _map_bounds     = [[ox, oy], [ox + w*res, oy + h*res]]
            _map_info_cache = msg.info
            _map_dirty      = True
    except Exception:
        pass

# ══════════════════════════════════════════════════════════════════════
# SYSTEM METRICS
# ══════════════════════════════════════════════════════════════════════
_sl = threading.Lock()
_sm = {
    "cpu_pct": [],  "cpu_temp":  0.0,
    "ram_used":  0, "ram_total": 1,
    "disk_used": 0, "disk_total":1,
    "rx_bps":  0.0, "tx_bps":   0.0,
    "procs":    {}, "ros_nodes": [],
}

WATCH_PROCS = {
    "Ollama":      "ollama",
    "Kinect2":     "kinect2_bridge",
    "RTABMap":     "rtabmap",
    "MegaBridge":  "mega_bridge_node",
    "Battery":     "pzem_battery",
    "GPS":         "gps_node",
    "microROS":    "micro_ros_agent",
    "TeleopUI":    "teleop_ui",
    "NodeManager": "node_manager_node",
}

def _sys_thread():
    if not HAS_PSUTIL: return
    psutil.cpu_percent(percpu=True)
    net0 = psutil.net_io_counters()
    while True:
        time.sleep(2.0)
        try:
            cpu  = psutil.cpu_percent(percpu=True, interval=None)
            temp = 0.0
            try:
                ts  = psutil.sensors_temperatures()
                src = ts.get('coretemp') or ts.get('acpitz') or []
                for e in src:
                    if 'Package' in getattr(e, 'label', '') or src.index(e) == 0:
                        temp = e.current; break
            except Exception: pass
            ram  = psutil.virtual_memory()
            dsk  = psutil.disk_usage('/')
            net1 = psutil.net_io_counters()
            rx   = (net1.bytes_recv - net0.bytes_recv) / 2.0
            tx   = (net1.bytes_sent - net0.bytes_sent) / 2.0
            net0 = net1
            procs = {}
            try:
                rows = []
                for p in psutil.process_iter(['name', 'cmdline']):
                    try:
                        rows.append(((p.info['name'] or '') + ' ' +
                                     ' '.join(p.info['cmdline'] or [])).lower())
                    except (psutil.NoSuchProcess, psutil.AccessDenied): pass
                for lbl, kw in WATCH_PROCS.items():
                    procs[lbl] = any(kw in r for r in rows)
            except Exception: pass
            with _sl:
                _sm.update(cpu_pct=cpu,  cpu_temp=temp,
                           ram_used=ram.used,   ram_total=ram.total,
                           disk_used=dsk.used,  disk_total=dsk.total,
                           rx_bps=rx, tx_bps=tx, procs=procs)
        except Exception: pass

def get_sys():
    with _sl: return dict(_sm)

# ══════════════════════════════════════════════════════════════════════
# PROCESS MANAGER  (node failover + launch controls)
# ══════════════════════════════════════════════════════════════════════

_SRC = ("source /opt/ros/jazzy/setup.bash && "
        "source /home/jadam/robot_ws/install/setup.bash")

NODE_DEFS: OrderedDict = OrderedDict([
    # ── SLAM stack ──────────────────────────────────────────────────
    ("RTABMap",     dict(
        ros_name="rtabmap",
        critical=True,
        cmd=f"bash -c '{_SRC} && ros2 launch robot_bringup robot.launch.py'",
        group="SLAM")),
    ("ICP Odom",    dict(
        ros_name="icp_odometry",
        critical=True,
        cmd=None,
        group="SLAM")),
    ("Kinect2",     dict(
        ros_name="kinect2_bridge",
        critical=True,
        cmd=f"bash -c '{_SRC} && ros2 launch kinect2_ros2_cuda kinect2_bridge.launch.py'",
        group="SLAM")),
    # ── I/O nodes ───────────────────────────────────────────────────
    ("MegaBridge",  dict(
        ros_name="mega_bridge_node",
        critical=True,
        cmd=f"bash -c '{_SRC} && ros2 run robot_bringup mega_bridge_node.py'",
        group="IO")),
    ("Battery",     dict(
        ros_name="pzem_battery_node",
        critical=True,
        cmd=f"bash -c '{_SRC} && ros2 run robot_bringup pzem_battery_node.py'",
        group="IO")),
    ("GPS",         dict(
        ros_name="gps_node",
        critical=True,
        cmd=f"bash -c '{_SRC} && ros2 run robot_bringup gps_node.py'",
        group="IO")),
    # ── Optional / toggleable ────────────────────────────────────────
    ("RTABMap Viz", dict(
        ros_name="rtabmap_viz",
        critical=False,
        cmd=f"bash -c '{_SRC} && ros2 run rtabmap_viz rtabmap_viz'",
        group="VIZ")),
    ("RViz2",       dict(
        ros_name="rviz2",
        critical=False,
        cmd=f"bash -c '{_SRC} && ros2 run rviz2 rviz2'",
        group="VIZ")),
    ("Foxglove",    dict(
        ros_name="foxglove_bridge",
        critical=False,
        cmd=(f"bash -c '{_SRC} && ros2 run foxglove_bridge foxglove_bridge "
             f"--ros-args -p port:=8765 -p address:=::'"),
        group="VIZ")),
    ("Brain",       dict(
        ros_name="llm_brain_node",
        critical=False,
        cmd=f"bash -c '{_SRC} && ros2 launch robosnailbob_brain brain.launch.py'",
        group="BRAIN")),
    # ── Navigation ──────────────────────────────────────────────────
    ("Nav2",        dict(
        ros_name="bt_navigator",
        critical=False,
        cmd=f"bash -c '{_SRC} && ros2 launch robosnailbob_navigation nav2.launch.py'",
        group="NAV")),
    ("Explorer",    dict(
        ros_name="frontier_explorer_node",
        critical=False,
        cmd=f"bash -c '{_SRC} && ros2 run robosnailbob_navigation frontier_explorer_node.py'",
        group="NAV")),
    # ── System ──────────────────────────────────────────────────────
    ("NodeManager", dict(
        ros_name="node_manager_node",
        critical=True,
        cmd=f"bash -c '{_SRC} && ros2 run robot_bringup node_manager_node.py'",
        group="SYS")),
])

# ── Process tracking ──────────────────────────────────────────────────
_proc_lock    = threading.Lock()
_procs:       dict = {}
_last_restart: dict = {}

STARTUP_GRACE    = 20.0
RESTART_COOLDOWN = 30.0

_startup_time = time.time()
_auto_restart = True

_theme_red   = None
_theme_grn   = None
_theme_dim   = None
_theme_amber = None


def make_btn_themes():
    global _theme_red, _theme_grn, _theme_dim, _theme_amber

    with dpg.theme() as t:
        with dpg.theme_component(dpg.mvButton):
            dpg.add_theme_color(dpg.mvThemeCol_Button,        (150, 22, 22, 235))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (200, 40, 40, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive,  (110, 12, 12, 255))
            dpg.add_theme_color(dpg.mvThemeCol_Text,          (255, 210, 210, 255))
    _theme_red = t

    with dpg.theme() as t:
        with dpg.theme_component(dpg.mvButton):
            dpg.add_theme_color(dpg.mvThemeCol_Button,        (12, 105, 40, 220))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (18, 145, 58, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive,  (8,  78, 30, 255))
            dpg.add_theme_color(dpg.mvThemeCol_Text,          (200, 255, 215, 255))
    _theme_grn = t

    with dpg.theme() as t:
        with dpg.theme_component(dpg.mvButton):
            dpg.add_theme_color(dpg.mvThemeCol_Button,        (38, 46, 58, 210))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (50, 60, 76, 230))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive,  (28, 35, 46, 210))
            dpg.add_theme_color(dpg.mvThemeCol_Text,          (100, 120, 145, 200))
    _theme_dim = t

    with dpg.theme() as t:
        with dpg.theme_component(dpg.mvButton):
            dpg.add_theme_color(dpg.mvThemeCol_Button,        (120, 78,  8, 220))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (160, 108,14, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive,  ( 90, 56,  4, 220))
            dpg.add_theme_color(dpg.mvThemeCol_Text,          (255, 225, 170, 255))
    _theme_amber = t


def launch_proc(label: str) -> None:
    nd  = NODE_DEFS.get(label)
    cmd = nd['cmd'] if nd else None
    if not cmd:
        push_log("NODES", f"No solo restart cmd for [{label}] — restart SLAM stack manually")
        return
    with _proc_lock:
        old = _procs.get(label)
        if old and old.poll() is None:
            try: old.terminate()
            except Exception: pass
        try:
            p = subprocess.Popen(
                cmd, shell=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=lambda: None)
            _procs[label]        = p
            _last_restart[label] = time.time()
            push_log("NODES", f"Launched [{label}] pid={p.pid}")
        except Exception as e:
            push_log("NODES", f"Failed to launch [{label}]: {e}")


def launch_cmd_once(label: str, cmd: str) -> None:
    with _proc_lock:
        old = _procs.get(label)
        if old and old.poll() is None:
            push_log("NODES", f"[{label}] already running (pid={old.pid}), skipped")
            return
        try:
            p = subprocess.Popen(
                cmd, shell=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL)
            _procs[label]        = p
            _last_restart[label] = time.time()
            push_log("NODES", f"Launched [{label}] pid={p.pid}")
        except Exception as e:
            push_log("NODES", f"Launch error [{label}]: {e}")


def kill_proc(label: str) -> None:
    nd = NODE_DEFS.get(label)
    with _proc_lock:
        p = _procs.pop(label, None)
        if p and p.poll() is None:
            try:
                p.terminate()
                push_log("NODES", f"Terminated [{label}] pid={p.pid}")
                return
            except Exception as e:
                push_log("NODES", f"Terminate error [{label}]: {e}")
    if nd:
        try:
            subprocess.run(["pkill", "-f", nd['ros_name']], check=False)
            push_log("NODES", f"pkill -f {nd['ros_name']}")
        except Exception: pass


def _node_btn_cb(label: str) -> None:
    nd = NODE_DEFS.get(label)
    if not nd: return
    with _sl: nodes = list(_sm.get('ros_nodes', []))
    alive = any(nd['ros_name'] in n for n in nodes)
    if alive and not nd['critical']:
        kill_proc(label)
    elif not alive:
        launch_proc(label)


def _auto_restart_tick() -> None:
    global _auto_restart
    if not _auto_restart: return
    now = time.time()
    if now - _startup_time < STARTUP_GRACE: return
    with _sl: nodes = list(_sm.get('ros_nodes', []))
    for label, nd in NODE_DEFS.items():
        if not nd['critical'] or not nd['cmd']: continue
        alive = any(nd['ros_name'] in n for n in nodes)
        if alive: continue
        last = _last_restart.get(label, 0.0)
        if now - last > RESTART_COOLDOWN:
            push_log("AUTO", f"Auto-restarting dead critical node [{label}]")
            launch_proc(label)

# ══════════════════════════════════════════════════════════════════════
# VOICE LOG
# ══════════════════════════════════════════════════════════════════════
_log = deque(maxlen=120)
_ll  = threading.Lock()

def push_log(src: str, txt: str) -> None:
    ts = time.strftime("%H:%M:%S")
    with _ll: _log.append(f"[{ts}][{src}] {txt}")

def snap_log():
    with _ll: return list(_log)

# ══════════════════════════════════════════════════════════════════════
# KEY STATE
# ══════════════════════════════════════════════════════════════════════
KEY_HOLD = 0.15
_kt = {}
_kl = threading.Lock()

def on_key(k):
    with _kl: _kt[k] = time.time()

def held(k):
    with _kl: t = _kt.get(k, 0.0)
    return (time.time() - t) < KEY_HOLD

# ══════════════════════════════════════════════════════════════════════
# ROS2 NODE
# ══════════════════════════════════════════════════════════════════════
_node = None

class UINode(Node):
    def __init__(self):
        super().__init__("teleop_ui_node")
        sub = self.create_subscription
        sub(BatteryState,      "/battery/state",            self._bat,          10)
        sub(Odometry,          "/odom",                     self._odom,         10)
        sub(NavSatFix,         "/fix",                      self._gps,          10)
        sub(String,            "/gps/info",                 self._gps_info_cb,  10)
        sub(String,            "/voice/input",  lambda m: push_log("hear", m.data), 10)
        sub(String,            "/voice/output", lambda m: push_log("say",  m.data), 10)
        sub(Bool,   "/mega/mode",
            lambda m: SS(m_mode=bool(m.data),    m_last=time.time()), 10)
        sub(Int32,  "/mega/motor_left",
            lambda m: SS(m_left=int(m.data),     m_last=time.time()), 10)
        sub(Int32,  "/mega/motor_right",
            lambda m: SS(m_right=int(m.data),    m_last=time.time()), 10)
        sub(Int32,  "/mega/throttle",
            lambda m: SS(m_throttle=int(m.data), m_last=time.time()), 10)
        sub(Int32,  "/mega/steering",
            lambda m: SS(m_steering=int(m.data), m_last=time.time()), 10)
        sub(Float32MultiArray, "/mega/ultrasonics",         self._sonic,  10)
        sub(Image,             "/kinect2/qhd/image_color",  self._cam,     1)
        sub(OccupancyGrid,     "/map",                      self._map,     1)
        sub(Path,              "/plan",                     self._path,    1)
        self._pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_timer(5.0, self._nodes)
        SS(ros_ok=True)
        push_log("boot", "teleop_ui_node ready")

    def _bat(self, m):
        pct = m.percentage * 100.0 if m.percentage <= 1.0 else m.percentage
        v, c = float(m.voltage), abs(float(m.current))
        SS(battery_pct=float(pct), voltage=v, current=c, power=v*c,
           bat_last=time.time())
        push_power(v, c)

    def _odom(self, m):
        q   = m.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        SS(odom_x=float(m.pose.pose.position.x),
           odom_y=float(m.pose.pose.position.y),
           odom_yaw=float(yaw),
           lin_vel=float(m.twist.twist.linear.x),
           ang_vel=float(m.twist.twist.angular.z),
           odom_last=time.time())

    def _gps(self, m):
        lat, lon, fix = float(m.latitude), float(m.longitude), int(m.status.status)
        SS(gps_lat=lat, gps_lon=lon, gps_alt=float(m.altitude),
           gps_fix=fix, gps_last=time.time())
        if fix >= 0: push_gps_pos(lat, lon)

    def _gps_info_cb(self, m):
        """Receive full GPS signal metrics from /gps/info (JSON)."""
        try:
            SS(gps_info=json.loads(m.data))
        except Exception:
            pass

    def _sonic(self, m):
        v = list(m.data[:8]) + [0.0] * (8 - len(m.data))
        SS(m_sonics=v, m_last=time.time())

    def _cam(self,  m): _process_camera(m)
    def _map(self,  m): _process_map(m)

    def _path(self, m):
        global _path_xs, _path_ys
        xs = [p.pose.position.x for p in m.poses]
        ys = [p.pose.position.y for p in m.poses]
        with _path_lock: _path_xs = xs; _path_ys = ys

    def _nodes(self):
        """Poll ROS2 node list every 5 s — stored in _sm['ros_nodes']."""
        try:
            with _sl: _sm['ros_nodes'] = sorted(self.get_node_names())
        except Exception: pass

    def pub(self, lin, ang):
        t = Twist(); t.linear.x = float(lin); t.angular.z = float(ang)
        self._pub.publish(t)


def _ros_thread():
    global _node
    rclpy.init(); _node = UINode()
    try:    rclpy.spin(_node)
    except Exception: pass
    finally:
        SS(ros_ok=False); push_log("warn", "ROS2 node stopped")
        try: _node.destroy_node(); rclpy.shutdown()
        except Exception: pass
        _node = None

# ══════════════════════════════════════════════════════════════════════
# THEME
# ══════════════════════════════════════════════════════════════════════
def apply_theme():
    with dpg.theme() as th:
        with dpg.theme_component(dpg.mvAll):
            for col, val in [
                (dpg.mvThemeCol_WindowBg,        ( 13,  17,  23, 255)),
                (dpg.mvThemeCol_ChildBg,          ( 20,  26,  35, 255)),
                (dpg.mvThemeCol_PopupBg,          ( 20,  26,  35, 255)),
                (dpg.mvThemeCol_Border,           ( 48,  68,  95, 180)),
                (dpg.mvThemeCol_FrameBg,          ( 30,  40,  55, 255)),
                (dpg.mvThemeCol_FrameBgHovered,   ( 45,  62,  85, 255)),
                (dpg.mvThemeCol_TitleBg,          ( 10,  14,  20, 255)),
                (dpg.mvThemeCol_TitleBgActive,    (  0, 140, 200, 255)),
                (dpg.mvThemeCol_Tab,              ( 20,  26,  35, 255)),
                (dpg.mvThemeCol_TabHovered,       (  0, 140, 200, 180)),
                (dpg.mvThemeCol_TabActive,        (  0, 100, 160, 255)),
                (dpg.mvThemeCol_Header,           (  0, 100, 160, 180)),
                (dpg.mvThemeCol_HeaderHovered,    (  0, 140, 200, 200)),
                (dpg.mvThemeCol_Button,           (  0, 100, 160, 220)),
                (dpg.mvThemeCol_ButtonHovered,    (  0, 140, 200, 255)),
                (dpg.mvThemeCol_ButtonActive,     (  0,  80, 130, 255)),
                (dpg.mvThemeCol_Text,             (200, 220, 240, 255)),
                (dpg.mvThemeCol_CheckMark,        (  0, 200, 255, 255)),
                (dpg.mvThemeCol_SliderGrab,       (  0, 140, 200, 255)),
                (dpg.mvThemeCol_SliderGrabActive, (  0, 200, 255, 255)),
                (dpg.mvThemeCol_PlotHistogram,    (  0, 160, 220, 255)),
                (dpg.mvThemeCol_PlotLines,        (  0, 200, 255, 255)),
            ]:
                dpg.add_theme_color(col, val)
            for sty, args in [
                (dpg.mvStyleVar_WindowRounding, (6,)),
                (dpg.mvStyleVar_FrameRounding,  (4,)),
                (dpg.mvStyleVar_TabRounding,    (4,)),
                (dpg.mvStyleVar_WindowPadding,  (12, 12)),
                (dpg.mvStyleVar_ItemSpacing,    ( 8,  5)),
                (dpg.mvStyleVar_FramePadding,   ( 6,  4)),
            ]:
                dpg.add_theme_style(sty, *args)
    dpg.bind_theme(th)

# ══════════════════════════════════════════════════════════════════════
# HELPERS
# ══════════════════════════════════════════════════════════════════════
def sec(label):
    dpg.add_spacer(height=8)
    dpg.add_text(label, color=(0, 160, 220, 255))
    dpg.add_separator()
    dpg.add_spacer(height=3)

def fmt_bytes(n):
    for u in ('B/s', 'KB/s', 'MB/s', 'GB/s'):
        if n < 1024: return f"{n:.0f} {u}"
        n /= 1024
    return f"{n:.1f} TB/s"

def fmt_runtime(pct, amps):
    if amps < 0.2: return "–"
    h  = 50.0 * (pct / 100.0) / amps
    hh, mm = int(h), int((h % 1) * 60)
    return f"~{hh}h {mm:02d}m" if hh > 0 else f"~{mm}m"

def fix_label(f):
    return {-1: "NO FIX", 0: "GPS", 1: "SBAS", 2: "RTK/DGPS"}.get(f, "UNKNOWN")

def fix_color(f, live):
    if not live: return (120, 120, 140, 255)
    if f == 2:   return (  0, 220, 120, 255)
    if f >= 0:   return (220, 180,   0, 255)
    return              (220,  60,  60, 255)

def dot_upd(tag, ok, label):
    dpg.set_value(tag, f"● {label}")
    dpg.configure_item(tag, color=(0, 220, 120, 255) if ok else (120, 120, 140, 255))

# ══════════════════════════════════════════════════════════════════════
# TAB: STATUS
# ══════════════════════════════════════════════════════════════════════
def build_status():
    with dpg.tab(label="  STATUS  "):
        sec("POWER")
        dpg.add_text("Battery    0%", tag="s_bat")
        dpg.add_progress_bar(default_value=0.0, width=-1, height=16,
                             tag="s_bat_bar", overlay="0%")
        dpg.add_spacer(height=2)
        dpg.add_text("Voltage    0.00 V",   tag="s_volt")
        dpg.add_text("Current    0.00 A",   tag="s_curr")
        dpg.add_text("Power      0.0 W",    tag="s_pwr")
        dpg.add_text("Runtime    –",        tag="s_runtime",
                     color=(160, 220, 160, 255))

        sec("VOLTAGE HISTORY")
        with dpg.plot(height=80, width=-1, no_mouse_pos=True):
            dpg.add_plot_axis(dpg.mvXAxis, tag="vx", no_tick_labels=True)
            with dpg.plot_axis(dpg.mvYAxis, label="V", tag="vy"):
                dpg.add_line_series([], [], tag="vs")
            dpg.set_axis_limits("vy", 20.0, 29.0)

        sec("CURRENT HISTORY")
        with dpg.plot(height=80, width=-1, no_mouse_pos=True):
            dpg.add_plot_axis(dpg.mvXAxis, tag="cx", no_tick_labels=True)
            with dpg.plot_axis(dpg.mvYAxis, label="A", tag="cy"):
                dpg.add_line_series([], [], tag="cs")
            dpg.set_axis_limits("cy", 0.0, 20.0)

        sec("GPS")
        dpg.add_text("Fix     NO FIX",  tag="s_gfix",  color=(120, 120, 140, 255))
        dpg.add_text("Lat     --",      tag="s_glat")
        dpg.add_text("Lon     --",      tag="s_glon")
        dpg.add_text("Alt     -- m",    tag="s_galt")
        dpg.add_spacer(height=2)
        dpg.add_text("Sats    -- / --", tag="s_gsats", color=(160, 200, 220, 255))
        dpg.add_text("HDOP    --",      tag="s_ghdop")
        dpg.add_text("VDOP    --",      tag="s_gvdop")
        dpg.add_text("SNR     --",      tag="s_gsnr",  color=(160, 200, 220, 255))
        dpg.add_text("Speed   --",      tag="s_gspd")
        dpg.add_text("Updated --",      tag="s_glast", color=(120, 120, 140, 255))

        sec("GPS TRACK")
        with dpg.plot(height=140, width=-1, no_mouse_pos=True):
            dpg.add_plot_axis(dpg.mvXAxis, label="E (m)", tag="gx")
            with dpg.plot_axis(dpg.mvYAxis, label="N (m)", tag="gy"):
                dpg.add_line_series([],  [],  tag="gtrack")
                dpg.add_scatter_series([], [], tag="gpos")

        sec("ODOMETRY")
        dpg.add_text("X        0.000 m",     tag="s_ox")
        dpg.add_text("Y        0.000 m",     tag="s_oy")
        dpg.add_text("Linear   0.000 m/s",   tag="s_lv")
        dpg.add_text("Angular  0.000 rad/s", tag="s_av")

        sec("CONNECTIONS")
        dpg.add_text("● ROS2",    tag="d_ros",  color=(120, 120, 140, 255))
        dpg.add_text("● Battery", tag="d_bat",  color=(120, 120, 140, 255))
        dpg.add_text("● Odom",    tag="d_odom", color=(120, 120, 140, 255))
        dpg.add_text("● GPS",     tag="d_gps",  color=(120, 120, 140, 255))
        dpg.add_text("● Drive",   tag="d_drv",  color=(120, 120, 140, 255))
        dpg.add_text("● Camera",  tag="d_cam",  color=(120, 120, 140, 255))
        dpg.add_text("● Map",     tag="d_map",  color=(120, 120, 140, 255))

# ══════════════════════════════════════════════════════════════════════
# TAB: TELEOP
# ══════════════════════════════════════════════════════════════════════
def build_teleop():
    with dpg.tab(label="  TELEOP  "):
        sec("ENABLE")
        dpg.add_checkbox(label=" TELEOP ENABLED", tag="t_en", default_value=False)
        dpg.add_spacer(height=2)
        dpg.add_text("⚠ Uncheck to stop publishing cmd_vel", color=(180, 140, 0, 255))

        sec("KEYBOARD CONTROLS")
        dpg.add_text("W  /  ↑     Forward",    color=(180, 210, 240, 255))
        dpg.add_text("S  /  ↓     Reverse",    color=(180, 210, 240, 255))
        dpg.add_text("A  /  ←     Turn Left",  color=(180, 210, 240, 255))
        dpg.add_text("D  /  →     Turn Right", color=(180, 210, 240, 255))
        dpg.add_text("Space       STOP",        color=(220,  80,  80, 255))

        sec("VELOCITY LIMITS")
        dpg.add_slider_float(label="Max Linear  (m/s)",   tag="t_maxlin",
                             default_value=0.4, min_value=0.0, max_value=1.0, width=-1)
        dpg.add_spacer(height=4)
        dpg.add_slider_float(label="Max Angular (rad/s)", tag="t_maxang",
                             default_value=1.0, min_value=0.0, max_value=2.0, width=-1)

        sec("LIVE OUTPUT")
        dpg.add_text("● DISABLED",           tag="t_status", color=(120, 120, 140, 255))
        dpg.add_text("Linear:  0.000 m/s",   tag="t_lin",    color=(160, 200, 220, 255))
        dpg.add_text("Angular: 0.000 rad/s", tag="t_ang",    color=(160, 200, 220, 255))

# ══════════════════════════════════════════════════════════════════════
# TAB: DRIVE
# ══════════════════════════════════════════════════════════════════════
SONIC_DIRS   = ["FL", "FC", "FR", " R", "RR", "RC", "RL", " L"]
SONIC_MAX_MM = 6000.0

def build_drive():
    with dpg.tab(label="  DRIVE  "):
        sec("MODE")
        dpg.add_text("● UNKNOWN", tag="dr_mode", color=(120, 120, 140, 255))

        sec("COMMANDS")
        dpg.add_text("Throttle   0", tag="dr_thr")
        dpg.add_progress_bar(default_value=0.5, width=-1, height=12,
                             tag="dr_thr_bar", overlay="0")
        dpg.add_spacer(height=4)
        dpg.add_text("Steering   0", tag="dr_str")
        dpg.add_progress_bar(default_value=0.5, width=-1, height=12,
                             tag="dr_str_bar", overlay="0")

        sec("MOTOR POWER  (-127 → 0 → +127)")
        dpg.add_text("LEFT    0", tag="dr_ml_lbl")
        dpg.add_progress_bar(default_value=0.5, width=-1, height=22,
                             tag="dr_ml_bar", overlay="0")
        dpg.add_spacer(height=6)
        dpg.add_text("RIGHT   0", tag="dr_mr_lbl")
        dpg.add_progress_bar(default_value=0.5, width=-1, height=22,
                             tag="dr_mr_bar", overlay="0")

        sec("PROXIMITY  (cm)")
        for i in range(8):
            dpg.add_text(f"{SONIC_DIRS[i]}  --", tag=f"dr_s{i}",
                         color=(120, 120, 140, 255))
            dpg.add_progress_bar(default_value=0.0, width=-1, height=8,
                                 tag=f"dr_sb{i}")
            dpg.add_spacer(height=1)

# ══════════════════════════════════════════════════════════════════════
# TAB: NODES
# ══════════════════════════════════════════════════════════════════════
def build_nodes():
    with dpg.tab(label="  NODES  "):

        sec("QUICK LAUNCH")
        with dpg.group(horizontal=True):
            dpg.add_button(
                label="▶ Full Stack", width=120,
                callback=lambda: launch_cmd_once(
                    "FullStack",
                    f"bash -c '{_SRC} && ros2 launch robot_bringup full_robot.launch.py'"))
            dpg.add_spacer(width=6)
            dpg.add_button(
                label="▶ SLAM Stack", width=120,
                callback=lambda: launch_cmd_once(
                    "SlamStack",
                    f"bash -c '{_SRC} && ros2 launch robot_bringup robot.launch.py'"))
        dpg.add_spacer(height=5)
        with dpg.group(horizontal=True):
            dpg.add_button(label="▶ RTABMap Viz", width=120,
                           callback=lambda: launch_proc("RTABMap Viz"))
            dpg.add_spacer(width=6)
            dpg.add_button(label="■ Kill Viz",    width=100,
                           callback=lambda: kill_proc("RTABMap Viz"))
        dpg.add_spacer(height=4)
        with dpg.group(horizontal=True):
            dpg.add_button(label="▶ RViz2",       width=120,
                           callback=lambda: launch_proc("RViz2"))
            dpg.add_spacer(width=6)
            dpg.add_button(label="■ Kill RViz2",  width=100,
                           callback=lambda: kill_proc("RViz2"))
        dpg.add_spacer(height=4)
        with dpg.group(horizontal=True):
            dpg.add_button(label="▶ Brain",       width=120,
                           callback=lambda: launch_proc("Brain"))
            dpg.add_spacer(width=6)
            dpg.add_button(label="■ Kill Brain",  width=100,
                           callback=lambda: kill_proc("Brain"))
        dpg.add_spacer(height=4)
        with dpg.group(horizontal=True):
            dpg.add_button(label="▶ Foxglove",    width=120,
                           callback=lambda: launch_proc("Foxglove"))
            dpg.add_spacer(width=6)
            dpg.add_button(label="■ Kill Foxglove", width=100,
                           callback=lambda: kill_proc("Foxglove"))

        sec("AUTO-RESTART")
        dpg.add_checkbox(label=" Auto-restart dead critical nodes",
                         tag="nd_auto", default_value=True)
        dpg.add_text(
            f"Grace period: {STARTUP_GRACE:.0f}s  |  Cooldown: {RESTART_COOLDOWN:.0f}s/node",
            color=(90, 120, 150, 255))
        dpg.add_text("ICP Odom has no solo cmd — RTABMap restart covers it.",
                     color=(110, 110, 90, 255))

        sec("CRITICAL NODES")
        dpg.add_text("  STATUS  NODE            ACTION",
                     color=(65, 95, 125, 255))
        dpg.add_separator()
        dpg.add_spacer(height=3)
        for label, nd in NODE_DEFS.items():
            if not nd['critical']: continue
            with dpg.group(horizontal=True):
                dpg.add_text("●", tag=f"nd_dot_{label}",
                             color=(120, 120, 140, 255))
                dpg.add_text(f"  {label:<14}", color=(180, 200, 220, 255))
                dpg.add_button(
                    label="   ···   ",
                    tag=f"nd_btn_{label}",
                    callback=lambda s, a, u=label: _node_btn_cb(u),
                    width=136)
            dpg.add_spacer(height=3)

        sec("OPTIONAL NODES")
        dpg.add_text("  STATUS  NODE            ACTION",
                     color=(65, 95, 125, 255))
        dpg.add_separator()
        dpg.add_spacer(height=3)
        for label, nd in NODE_DEFS.items():
            if nd['critical']: continue
            with dpg.group(horizontal=True):
                dpg.add_text("●", tag=f"nd_dot_{label}",
                             color=(120, 120, 140, 255))
                dpg.add_text(f"  {label:<14}", color=(180, 200, 220, 255))
                dpg.add_button(
                    label="   ···   ",
                    tag=f"nd_btn_{label}",
                    callback=lambda s, a, u=label: _node_btn_cb(u),
                    width=136)
            dpg.add_spacer(height=3)

        sec("RESTART LOG")
        dpg.add_input_text(tag="nd_restart_log", multiline=True, readonly=True,
                           width=-1, height=80, default_value="(none)")

# ══════════════════════════════════════════════════════════════════════
# TAB: CAMERA
# ══════════════════════════════════════════════════════════════════════
def build_camera():
    with dpg.tab(label="  CAMERA  "):
        sec("KINECT RGB  /kinect2/qhd/image_color")
        with dpg.group(horizontal=True):
            dpg.add_text("● Waiting…", tag="cam_status", color=(120, 120, 140, 255))
            dpg.add_spacer(width=20)
            dpg.add_text("FPS: –",     tag="cam_fps",    color=(160, 200, 220, 255))
        dpg.add_spacer(height=6)
        dpg.add_image("cam_tex", width=CAM_DISP_W, height=CAM_DISP_H)

# ══════════════════════════════════════════════════════════════════════
# TAB: MAP
# ══════════════════════════════════════════════════════════════════════
def build_map():
    with dpg.tab(label="  MAP  "):
        sec("2D MAP  (RTABMap → /map)")
        with dpg.group(horizontal=True):
            dpg.add_text("● Waiting…", tag="map_status", color=(120, 120, 140, 255))
            dpg.add_spacer(width=16)
            dpg.add_text("–",          tag="map_info",   color=(160, 200, 220, 255))
        dpg.add_spacer(height=4)

        with dpg.plot(height=480, width=-1, tag="map_plot",
                      no_mouse_pos=False, equal_aspects=True):
            dpg.add_plot_legend()
            dpg.add_plot_axis(dpg.mvXAxis, label="X (m)", tag="map_px")
            with dpg.plot_axis(dpg.mvYAxis, label="Y (m)", tag="map_py"):
                dpg.add_image_series("map_tex",
                                     bounds_min=[0.0, 0.0],
                                     bounds_max=[1.0, 1.0],
                                     tag="map_series")
                dpg.add_line_series([],    [],    label="Path",    tag="map_path")
                dpg.add_line_series([],    [],    label="Heading", tag="map_heading")
                dpg.add_scatter_series([0.0], [0.0], label="Robot", tag="map_robot")

        sec("NAV2")
        dpg.add_text("● Inactive", tag="nav2_status", color=(120, 120, 140, 255))
        dpg.add_text("Nodes:  --",  tag="nav2_nodes",  color=(160, 200, 220, 255))

# ══════════════════════════════════════════════════════════════════════
# TAB: SYSTEM
# ══════════════════════════════════════════════════════════════════════
_NUM_CORES = psutil.cpu_count() if HAS_PSUTIL else 20

def build_system():
    with dpg.tab(label="  SYSTEM  "):
        sec("CPU")
        dpg.add_text("Package  – °C", tag="sy_temp",       color=(160, 200, 220, 255))
        dpg.add_text("Overall  0%",    tag="sy_overall")
        dpg.add_progress_bar(default_value=0.0, width=-1, height=14,
                             tag="sy_overall_bar", overlay="0%")
        dpg.add_spacer(height=4)
        p_end = min(12, _NUM_CORES)
        if p_end > 0:
            dpg.add_text("P-cores (0–11)", color=(100, 160, 200, 255))
            for i in range(p_end):
                with dpg.group(horizontal=True):
                    dpg.add_text(f"C{i:02d}", color=(100, 130, 160, 255))
                    dpg.add_progress_bar(default_value=0.0, width=-1,
                                         height=7, tag=f"sy_c{i}")
        if p_end < _NUM_CORES:
            dpg.add_spacer(height=4)
            dpg.add_text(f"E-cores ({p_end}–{_NUM_CORES-1})", color=(100, 160, 200, 255))
            for i in range(p_end, _NUM_CORES):
                with dpg.group(horizontal=True):
                    dpg.add_text(f"C{i:02d}", color=(80, 110, 140, 255))
                    dpg.add_progress_bar(default_value=0.0, width=-1,
                                         height=7, tag=f"sy_c{i}")

        sec("MEMORY")
        dpg.add_text("RAM   0 / 0 GB",  tag="sy_ram_lbl")
        dpg.add_progress_bar(default_value=0.0, width=-1, height=14,
                             tag="sy_ram_bar", overlay="0%")
        dpg.add_spacer(height=4)
        dpg.add_text("Disk  0 / 0 GB",  tag="sy_disk_lbl")
        dpg.add_progress_bar(default_value=0.0, width=-1, height=14,
                             tag="sy_disk_bar", overlay="0%")

        sec("NETWORK")
        dpg.add_text("RX  --", tag="sy_rx", color=(  0, 200, 160, 255))
        dpg.add_text("TX  --", tag="sy_tx", color=(200, 160,   0, 255))

        sec("PROCESSES")
        for lbl in WATCH_PROCS:
            dpg.add_text(f"● {lbl}", tag=f"sy_p_{lbl}", color=(120, 120, 140, 255))

        sec("ROS2 NODES")
        dpg.add_input_text(tag="sy_nodes", multiline=True, readonly=True,
                           width=-1, height=120, default_value="(none)")

# ══════════════════════════════════════════════════════════════════════
# TAB: LOG
# ══════════════════════════════════════════════════════════════════════
def build_log():
    with dpg.tab(label="  LOG  "):
        sec("VOICE / SYSTEM LOG")
        dpg.add_input_text(tag="log_txt", multiline=True, readonly=True,
                           width=-1, height=-1, default_value="")

# ══════════════════════════════════════════════════════════════════════
# UPDATE: STATUS
# ══════════════════════════════════════════════════════════════════════
def upd_status(s):
    now       = time.time()
    bat_live  = s["bat_last"]  > 0 and now - s["bat_last"]  < STALE
    odom_live = s["odom_last"] > 0 and now - s["odom_last"] < STALE
    gps_live  = s["gps_last"]  > 0 and now - s["gps_last"]  < STALE
    drv_live  = s["m_last"]    > 0 and now - s["m_last"]    < STALE

    pct = s["battery_pct"]
    dpg.set_value("s_bat",     f"Battery    {pct:.0f}%")
    dpg.set_value("s_bat_bar",  pct / 100.0)
    dpg.configure_item("s_bat_bar", overlay=f"{pct:.0f}%")
    dpg.set_value("s_volt",    f"Voltage    {s['voltage']:.2f} V")
    dpg.set_value("s_curr",    f"Current    {s['current']:.2f} A")
    dpg.set_value("s_pwr",     f"Power      {s['power']:.1f} W")
    dpg.set_value("s_runtime", f"Runtime    {fmt_runtime(pct, s['current'])}")

    vh = snap_buf(_vh)
    if len(vh) > 1:
        xs, ys = zip(*vh)
        dpg.set_value("vs", [list(xs), list(ys)])
        dpg.set_axis_limits("vx", xs[0], xs[-1])

    ch = snap_buf(_ch)
    if len(ch) > 1:
        xs, ys = zip(*ch)
        dpg.set_value("cs", [list(xs), list(ys)])
        dpg.set_axis_limits("cx", xs[0], xs[-1])

    # ── GPS basic fix / position ──────────────────────────────────────
    fix = s["gps_fix"]
    dpg.set_value("s_gfix", f"Fix     {fix_label(fix)}")
    dpg.configure_item("s_gfix", color=fix_color(fix, gps_live))
    if gps_live and fix >= 0:
        dpg.set_value("s_glat", f"Lat     {s['gps_lat']:.7f}°")
        dpg.set_value("s_glon", f"Lon     {s['gps_lon']:.7f}°")
        dpg.set_value("s_galt", f"Alt     {s['gps_alt']:.2f} m")
    else:
        dpg.set_value("s_glat", "Lat     --")
        dpg.set_value("s_glon", "Lon     --")
        dpg.set_value("s_galt", "Alt     -- m")

    # ── GPS signal metrics from /gps/info ─────────────────────────────
    gi = s.get("gps_info", {})
    if gi:
        su  = gi.get("satellites_used",    0)
        sv  = gi.get("satellites_in_view", 0)
        dpg.set_value("s_gsats", f"Sats    {su} used / {sv} in view")
        dpg.set_value("s_ghdop", f"HDOP    {gi.get('hdop', 0):.2f}")
        dpg.set_value("s_gvdop", f"VDOP    {gi.get('vdop', 0):.2f}")
        a   = gi.get("avg_snr_db", 0)
        mn  = gi.get("min_snr_db", 0)
        mx  = gi.get("max_snr_db", 0)
        dpg.set_value("s_gsnr",
                      f"SNR     {a:.0f} dBHz  ({mn:.0f}–{mx:.0f})")
        dpg.set_value("s_gspd",
                      f"Speed   {gi.get('speed_kph', 0):.1f} km/h  "
                      f"{gi.get('course_deg', 0):.0f}°")
        last_ts = gi.get("last_fix_utc") or "never"
        age     = gi.get("fix_age_sec")
        age_str = f"{age:.0f}s ago" if (age is not None and age >= 0) else "–"
        col = ((  0, 220, 120, 255) if age is not None and age <  5 else
               (220, 180,   0, 255) if age is not None and age < 30 else
               (120, 120, 140, 255))
        dpg.set_value("s_glast", f"Updated {last_ts}  ({age_str})")
        dpg.configure_item("s_glast", color=col)
    else:
        for tag, val in [("s_gsats", "Sats    --"),
                         ("s_ghdop", "HDOP    --"),
                         ("s_gvdop", "VDOP    --"),
                         ("s_gsnr",  "SNR     --"),
                         ("s_gspd",  "Speed   --"),
                         ("s_glast", "Updated --")]:
            dpg.set_value(tag, val)

    # ── GPS track plot ────────────────────────────────────────────────
    tr = snap_buf(_gtrack)
    if len(tr) > 1:
        xs, ys = zip(*tr)
        xs, ys = list(xs), list(ys)
        dpg.set_value("gtrack", [xs, ys])
        dpg.set_value("gpos",   [[xs[-1]], [ys[-1]]])
        pad = max(max(xs) - min(xs), max(ys) - min(ys), 4.0) * 0.15
        dpg.set_axis_limits("gx", min(xs) - pad, max(xs) + pad)
        dpg.set_axis_limits("gy", min(ys) - pad, max(ys) + pad)

    dpg.set_value("s_ox", f"X        {s['odom_x']:.3f} m")
    dpg.set_value("s_oy", f"Y        {s['odom_y']:.3f} m")
    dpg.set_value("s_lv", f"Linear   {s['lin_vel']:.3f} m/s")
    dpg.set_value("s_av", f"Angular  {s['ang_vel']:.3f} rad/s")

    dot_upd("d_ros",  s["ros_ok"],           "ROS2")
    dot_upd("d_bat",  bat_live,               "Battery")
    dot_upd("d_odom", odom_live,              "Odom")
    dot_upd("d_gps",  gps_live and fix >= 0, "GPS")
    dot_upd("d_drv",  drv_live,               "Drive")
    dot_upd("d_cam",  _cam_connected,         "Camera")
    dot_upd("d_map",  _map_connected,         "Map")

# ══════════════════════════════════════════════════════════════════════
# UPDATE: DRIVE
# ══════════════════════════════════════════════════════════════════════
def upd_drive(s):
    ok = s["m_mode"]
    dpg.set_value("dr_mode", "● RC ACTIVE" if ok else "● STANDBY")
    dpg.configure_item("dr_mode", color=(0, 220, 120, 255) if ok else (120, 120, 140, 255))

    thr, st = s["m_throttle"], s["m_steering"]
    dpg.set_value("dr_thr",     f"Throttle   {thr:+d}")
    dpg.set_value("dr_thr_bar", (thr + 100) / 200.0)
    dpg.configure_item("dr_thr_bar", overlay=f"{thr:+d}")
    dpg.set_value("dr_str",     f"Steering   {st:+d}")
    dpg.set_value("dr_str_bar", (st + 100) / 200.0)
    dpg.configure_item("dr_str_bar", overlay=f"{st:+d}")

    ml, mr = s["m_left"], s["m_right"]
    dpg.set_value("dr_ml_lbl",  f"LEFT    {ml:+d}")
    dpg.set_value("dr_ml_bar",  (ml + 127) / 254.0)
    dpg.configure_item("dr_ml_bar", overlay=f"{ml:+d}")
    dpg.set_value("dr_mr_lbl",  f"RIGHT   {mr:+d}")
    dpg.set_value("dr_mr_bar",  (mr + 127) / 254.0)
    dpg.configure_item("dr_mr_bar", overlay=f"{mr:+d}")

    for i, mm in enumerate(s["m_sonics"]):
        if mm > 0:
            cm = mm / 10.0
            dpg.set_value(f"dr_s{i}", f"{SONIC_DIRS[i]}  {cm:.0f}")
            dpg.configure_item(f"dr_s{i}",
                               color=(  0, 220, 120, 255) if cm > 100 else
                                     (220, 180,   0, 255) if cm >  30 else
                                     (220,  60,  60, 255))
        else:
            dpg.set_value(f"dr_s{i}", f"{SONIC_DIRS[i]}  --")
            dpg.configure_item(f"dr_s{i}", color=(120, 120, 140, 255))
        dpg.set_value(f"dr_sb{i}",
                      min(mm, SONIC_MAX_MM) / SONIC_MAX_MM if mm > 0 else 0.0)

# ══════════════════════════════════════════════════════════════════════
# UPDATE: NODES
# ══════════════════════════════════════════════════════════════════════
def upd_nodes():
    global _auto_restart

    _auto_restart = dpg.get_value("nd_auto")

    now      = time.time()
    grace_ok = (now - _startup_time) > STARTUP_GRACE

    with _sl:
        nodes = list(_sm.get('ros_nodes', []))

    for label, nd in NODE_DEFS.items():
        alive = any(nd['ros_name'] in n for n in nodes)
        btn   = f"nd_btn_{label}"
        dot   = f"nd_dot_{label}"

        if alive:
            dpg.configure_item(dot, color=(0, 220, 120, 255))
        elif nd['critical'] and grace_ok:
            dpg.configure_item(dot, color=(220, 60, 60, 255))
        else:
            dpg.configure_item(dot, color=(120, 120, 140, 255))

        if alive:
            if nd['critical']:
                dpg.configure_item(btn, label="   ✓ OK   ")
                dpg.bind_item_theme(btn, _theme_dim)
            else:
                dpg.configure_item(btn, label="■ KILL")
                dpg.bind_item_theme(btn, _theme_amber)
        else:
            if nd['critical'] and grace_ok:
                lbl_txt = ("✗ RESTART ↺" if nd['cmd'] else "✗ NO CMD")
                dpg.configure_item(btn, label=lbl_txt)
                dpg.bind_item_theme(btn, _theme_red)
            elif nd['critical']:
                dpg.configure_item(btn, label="  STARTING…")
                dpg.bind_item_theme(btn, _theme_dim)
            else:
                dpg.configure_item(btn, label="▶ LAUNCH")
                dpg.bind_item_theme(btn, _theme_grn)

    lines = []
    for lbl, t in sorted(_last_restart.items(), key=lambda x: -x[1]):
        ago = now - t
        lines.append(f"{lbl:<15}  {time.strftime('%H:%M:%S', time.localtime(t))}"
                     f"  ({ago:.0f}s ago)")
    dpg.set_value("nd_restart_log", "\n".join(lines) if lines else "(none)")

# ══════════════════════════════════════════════════════════════════════
# UPDATE: CAMERA
# ══════════════════════════════════════════════════════════════════════
def upd_camera():
    global _cam_dirty
    if _cam_dirty:
        with _cam_lock:
            data       = _cam_rgba
            _cam_dirty = False
        try: dpg.set_value("cam_tex", data)
        except Exception: pass
        dpg.set_value("cam_status", f"● Live  {CAM_W}×{CAM_H}")
        dpg.configure_item("cam_status", color=(0, 220, 120, 255))
        dpg.set_value("cam_fps", f"FPS: {_cam_fps_val:.1f}")

# ══════════════════════════════════════════════════════════════════════
# UPDATE: MAP
# ══════════════════════════════════════════════════════════════════════
def upd_map(s):
    global _map_dirty
    if _map_dirty:
        with _map_lock:
            data       = _map_rgba
            bounds     = list(_map_bounds)
            info       = _map_info_cache
            _map_dirty = False
        try:
            dpg.set_value("map_tex", data)
            dpg.configure_item("map_series",
                               bounds_min=bounds[0], bounds_max=bounds[1])
        except Exception: pass
        if info:
            dpg.set_value("map_status",
                          f"● Live  {info.width}×{info.height} cells")
            dpg.configure_item("map_status", color=(0, 220, 120, 255))
            dpg.set_value("map_info",
                          f"{info.resolution:.3f} m/cell  |  "
                          f"{info.width * info.resolution:.0f}×"
                          f"{info.height * info.resolution:.0f} m")

    x, y, yaw = s["odom_x"], s["odom_y"], s["odom_yaw"]
    dpg.set_value("map_robot", [[x], [y]])
    alen = 0.4
    dpg.set_value("map_heading",
                  [[x, x + alen * math.cos(yaw)],
                   [y, y + alen * math.sin(yaw)]])

    with _path_lock: pxs, pys = list(_path_xs), list(_path_ys)
    if pxs: dpg.set_value("map_path", [pxs, pys])

    with _sl: nodes = list(_sm.get("ros_nodes", []))
    nav_nodes = [n for n in nodes if "nav2" in n or "bt_navigator" in n
                 or "controller" in n or "planner" in n]
    if nav_nodes:
        dpg.set_value("nav2_status", "● Active")
        dpg.configure_item("nav2_status", color=(0, 220, 120, 255))
        dpg.set_value("nav2_nodes",
                      "Nodes:  " + ", ".join(n.lstrip("/") for n in nav_nodes[:3]))
    else:
        dpg.set_value("nav2_status", "● Inactive")
        dpg.configure_item("nav2_status", color=(120, 120, 140, 255))
        dpg.set_value("nav2_nodes", "Nodes:  --")

# ══════════════════════════════════════════════════════════════════════
# UPDATE: SYSTEM
# ══════════════════════════════════════════════════════════════════════
def upd_system(sy):
    cpus = sy["cpu_pct"]
    if cpus:
        ov = sum(cpus) / len(cpus)
        dpg.set_value("sy_overall",     f"Overall  {ov:.0f}%")
        dpg.set_value("sy_overall_bar",  ov / 100.0)
        dpg.configure_item("sy_overall_bar", overlay=f"{ov:.0f}%")
        for i, pct in enumerate(cpus[:_NUM_CORES]):
            if dpg.does_item_exist(f"sy_c{i}"):
                dpg.set_value(f"sy_c{i}", pct / 100.0)

    temp  = sy["cpu_temp"]
    t_col = (  0, 220, 120, 255) if temp < 70 else \
            (220, 180,   0, 255) if temp < 85 else \
            (220,  60,  60, 255)
    dpg.set_value("sy_temp", f"Package  {temp:.1f} °C")
    dpg.configure_item("sy_temp", color=t_col)

    ru, rt = sy["ram_used"], sy["ram_total"]
    rp     = ru / rt if rt else 0
    dpg.set_value("sy_ram_lbl", f"RAM   {ru / 1e9:.1f} / {rt / 1e9:.1f} GB")
    dpg.set_value("sy_ram_bar",  rp)
    dpg.configure_item("sy_ram_bar", overlay=f"{rp * 100:.0f}%")

    du, dt = sy["disk_used"], sy["disk_total"]
    dp_    = du / dt if dt else 0
    dpg.set_value("sy_disk_lbl", f"Disk  {du / 1e9:.1f} / {dt / 1e9:.1f} GB")
    dpg.set_value("sy_disk_bar",  dp_)
    dpg.configure_item("sy_disk_bar", overlay=f"{dp_ * 100:.0f}%")

    dpg.set_value("sy_rx", f"RX  {fmt_bytes(sy['rx_bps'])}")
    dpg.set_value("sy_tx", f"TX  {fmt_bytes(sy['tx_bps'])}")

    for lbl in WATCH_PROCS:
        ok = sy["procs"].get(lbl, False)
        dpg.set_value(f"sy_p_{lbl}", f"● {lbl}")
        dpg.configure_item(f"sy_p_{lbl}",
                           color=(0, 220, 120, 255) if ok else (120, 120, 140, 255))

    nodes = sy.get("ros_nodes", [])
    dpg.set_value("sy_nodes", "\n".join(nodes) if nodes else "(none)")

# ══════════════════════════════════════════════════════════════════════
# CMD_VEL
# ══════════════════════════════════════════════════════════════════════
def compute_cmd_vel():
    en      = dpg.get_value("t_en")
    max_lin = dpg.get_value("t_maxlin")
    max_ang = dpg.get_value("t_maxang")
    lin = ang = 0.0
    if en:
        sp = held(dpg.mvKey_Spacebar)
        if not sp:
            if held(dpg.mvKey_W) or held(dpg.mvKey_Up):    lin =  max_lin
            if held(dpg.mvKey_S) or held(dpg.mvKey_Down):  lin = -max_lin
            if held(dpg.mvKey_A) or held(dpg.mvKey_Left):  ang =  max_ang
            if held(dpg.mvKey_D) or held(dpg.mvKey_Right): ang = -max_ang
        if _node: _node.pub(lin, ang)
        if sp or (lin == 0.0 and ang == 0.0):
            dpg.set_value("t_status", "● IDLE")
            dpg.configure_item("t_status", color=(120, 120, 140, 255))
        else:
            dpg.set_value("t_status", "● PUBLISHING")
            dpg.configure_item("t_status", color=(0, 220, 120, 255))
    else:
        if _node: _node.pub(0.0, 0.0)
        dpg.set_value("t_status", "● DISABLED")
        dpg.configure_item("t_status", color=(120, 120, 140, 255))
    dpg.set_value("t_lin", f"Linear:  {lin:.3f} m/s")
    dpg.set_value("t_ang", f"Angular: {ang:.3f} rad/s")

# ══════════════════════════════════════════════════════════════════════
# MAIN
# ══════════════════════════════════════════════════════════════════════
def main():
    threading.Thread(target=_ros_thread, daemon=True).start()
    threading.Thread(target=_sys_thread, daemon=True).start()

    dpg.create_context()
    apply_theme()
    make_btn_themes()

    with dpg.texture_registry():
        dpg.add_raw_texture(CAM_W, CAM_H, _cam_rgba.tolist(),
                            tag="cam_tex", format=dpg.mvFormat_Float_rgba)
        dpg.add_raw_texture(MAP_TEX, MAP_TEX, _map_rgba.tolist(),
                            tag="map_tex", format=dpg.mvFormat_Float_rgba)

    with dpg.handler_registry():
        dpg.add_key_down_handler(dpg.mvKey_W,        callback=lambda: on_key(dpg.mvKey_W))
        dpg.add_key_down_handler(dpg.mvKey_S,        callback=lambda: on_key(dpg.mvKey_S))
        dpg.add_key_down_handler(dpg.mvKey_A,        callback=lambda: on_key(dpg.mvKey_A))
        dpg.add_key_down_handler(dpg.mvKey_D,        callback=lambda: on_key(dpg.mvKey_D))
        dpg.add_key_down_handler(dpg.mvKey_Up,       callback=lambda: on_key(dpg.mvKey_Up))
        dpg.add_key_down_handler(dpg.mvKey_Down,     callback=lambda: on_key(dpg.mvKey_Down))
        dpg.add_key_down_handler(dpg.mvKey_Left,     callback=lambda: on_key(dpg.mvKey_Left))
        dpg.add_key_down_handler(dpg.mvKey_Right,    callback=lambda: on_key(dpg.mvKey_Right))
        dpg.add_key_down_handler(dpg.mvKey_Spacebar, callback=lambda: on_key(dpg.mvKey_Spacebar))

    with dpg.window(tag="main"):
        dpg.add_text("ROBOSNAILBOB", color=(0, 200, 255, 255))
        dpg.add_text("Teleop & Status Dashboard",
                     color=(100, 130, 160, 255), indent=2)
        dpg.add_separator()
        dpg.add_spacer(height=4)
        with dpg.tab_bar():
            build_status()
            build_teleop()
            build_drive()
            build_nodes()
            build_camera()
            build_map()
            build_system()
            build_log()

    dpg.create_viewport(title="RoboSnailBob", width=600, height=860,
                        min_width=480, min_height=600)
    dpg.setup_dearpygui()
    dpg.show_viewport()
    dpg.set_primary_window("main", True)

    while dpg.is_dearpygui_running():
        s  = GS()
        sy = get_sys()
        upd_status(s)
        upd_drive(s)
        upd_camera()
        upd_map(s)
        upd_nodes()
        _auto_restart_tick()
        upd_system(sy)
        dpg.set_value("log_txt", "\n".join(snap_log()))
        compute_cmd_vel()
        dpg.render_dearpygui_frame()

    dpg.destroy_context()


if __name__ == "__main__":
    main()
