#!/usr/bin/env python3
"""
RoboSnailBob Server Control GUI

Single-screen curses TUI for SSH/headless use.
Consolidates STATUS + TELEOP + DRIVE + SYSTEM + LOG onto one dense screen.

Layout
------
  Header  : robot name  timestamp  ROS/connection dots  model/personality
  Row A   : POWER | NAVIGATION | SYSTEM   (3 equal columns)
  Row B   : DRIVE  (full width)
  Row C   : TELEOP (full width)
  Row D   : VOICE LOG  (remaining rows)
  Footer  : key help

Subscriptions
-------------
  /battery/state      sensor_msgs/BatteryState
  /odom               nav_msgs/Odometry
  /imu/data           sensor_msgs/Imu
  /fix                sensor_msgs/NavSatFix
  /voice/input        std_msgs/String
  /voice/output       std_msgs/String
  /mega/mode          std_msgs/Bool
  /mega/motor_left    std_msgs/Int32
  /mega/motor_right   std_msgs/Int32
  /mega/throttle      std_msgs/Int32
  /mega/steering      std_msgs/Int32
  /mega/ultrasonics   std_msgs/Float32MultiArray

Publishes
---------
  /cmd_vel            geometry_msgs/Twist   (teleop)

Keys
----
  t          Toggle teleop enable / disable
  W/S        Forward / Reverse
  A/D        Turn left / right
  Space      Emergency stop (zero cmd_vel)
  q / Q      Quit
"""

import curses
import math
import os
import threading
import time
from collections import deque

try:
    import psutil
    HAS_PSUTIL = True
except ImportError:
    HAS_PSUTIL = False

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Imu, NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32MultiArray, Int32, String


# ── Constants ─────────────────────────────────────────────────────────────────

STALE = 6.0       # seconds before a topic value is considered stale
KEY_HOLD = 0.12   # seconds a key remains "held" for smooth teleop

MAX_LIN = 0.4     # m/s  default max linear velocity
MAX_ANG = 1.0     # rad/s default max angular velocity

SONIC_DIRS = ['FL', 'FC', 'FR', ' R', 'RR', 'RC', 'RL', ' L']

WATCH_PROCS = {
    'Ollama':    'ollama',
    'VoiceIO':   'voice_io_node',
    'LLMBrain':  'llm_brain_node',
    'MegaBrdg':  'mega_bridge_node',
    'Battery':   'pzem_battery',
    'GPS':       'gps_node',
    'Kinect2':   'kinect2_bridge',
    'RTABMap':   'rtabmap',
    'microROS':  'micro_ros_agent',
}

MIN_ROWS = 26
MIN_COLS = 80


# ── Shared State ──────────────────────────────────────────────────────────────

_lock = threading.Lock()
_state = {
    'ros_ok':     False,
    'bat_pct':    0.0,    'voltage':   0.0,  'current':  0.0,
    'power':      0.0,    'bat_last':  0.0,
    'odom_x':     0.0,    'odom_y':    0.0,
    'lin_vel':    0.0,    'ang_vel':   0.0,  'odom_last': 0.0,
    'heading':    None,
    'gps_lat':    0.0,    'gps_lon':   0.0,  'gps_alt':   0.0,
    'gps_fix':   -1,      'gps_last':  0.0,
    'm_mode':     False,  'm_throttle': 0,   'm_steering': 0,
    'm_left':     0,      'm_right':    0,
    'm_sonics':   [0.0] * 8,               'm_last': 0.0,
    'teleop_en':  False,
    'cmd_lin':    0.0,    'cmd_ang':   0.0,
}
_start = time.time()


def _gs():
    with _lock:
        return dict(_state)


def _ss(**kw):
    with _lock:
        _state.update(kw)


# ── Voice Log ─────────────────────────────────────────────────────────────────

_log = deque(maxlen=200)
_ll = threading.Lock()


def _push_log(src, txt):
    ts = time.strftime('%H:%M:%S')
    with _ll:
        _log.append(f'[{ts}][{src:4s}] {txt[:130]}')


def _snap_log():
    with _ll:
        return list(_log)


# ── System Metrics ────────────────────────────────────────────────────────────

_sl = threading.Lock()
_sm = {
    'cpu_pct': [],  'cpu_temp': 0.0,
    'ram_used': 0,  'ram_total': 1,
    'dsk_used': 0,  'dsk_total': 1,
    'rx_bps':  0.0, 'tx_bps':   0.0,
    'procs':   {},  'ros_nodes': [],
}


def _sys_loop():
    if not HAS_PSUTIL:
        return
    psutil.cpu_percent(percpu=True)
    net0 = psutil.net_io_counters()
    while True:
        time.sleep(2.0)
        try:
            cpu = psutil.cpu_percent(percpu=True, interval=None)
            temp = 0.0
            try:
                ts = psutil.sensors_temperatures()
                src = ts.get('coretemp') or ts.get('acpitz') or []
                for e in src:
                    if 'Package' in getattr(e, 'label', '') or src.index(e) == 0:
                        temp = e.current
                        break
            except Exception:
                pass
            ram = psutil.virtual_memory()
            dsk = psutil.disk_usage('/')
            net1 = psutil.net_io_counters()
            rx = (net1.bytes_recv - net0.bytes_recv) / 2.0
            tx = (net1.bytes_sent - net0.bytes_sent) / 2.0
            net0 = net1
            procs = {}
            try:
                rows = []
                for p in psutil.process_iter(['name', 'cmdline']):
                    try:
                        rows.append(
                            ((p.info['name'] or '') + ' ' +
                             ' '.join(p.info['cmdline'] or [])).lower()
                        )
                    except (psutil.NoSuchProcess, psutil.AccessDenied):
                        pass
                for lbl, kw in WATCH_PROCS.items():
                    procs[lbl] = any(kw in r for r in rows)
            except Exception:
                pass
            with _sl:
                _sm.update(
                    cpu_pct=cpu, cpu_temp=temp,
                    ram_used=ram.used, ram_total=ram.total,
                    dsk_used=dsk.used, dsk_total=dsk.total,
                    rx_bps=rx, tx_bps=tx, procs=procs,
                )
        except Exception:
            pass


def _get_sys():
    with _sl:
        return dict(_sm)


# ── ROS2 Node ─────────────────────────────────────────────────────────────────

_node = None


class _ServerGUINode(Node):

    def __init__(self):
        super().__init__('server_gui_node')
        sub = self.create_subscription
        sub(BatteryState,     '/battery/state',    self._bat,    10)
        sub(Odometry,         '/odom',             self._odom,   10)
        sub(Imu,              '/imu/data',         self._imu,    10)
        sub(NavSatFix,        '/fix',              self._gps,    10)
        sub(String,           '/voice/input',
            lambda m: _push_log('hear', m.data), 10)
        sub(String,           '/voice/output',
            lambda m: _push_log('say ', m.data), 10)
        sub(Bool,    '/mega/mode',
            lambda m: _ss(m_mode=bool(m.data),       m_last=time.time()), 10)
        sub(Int32,   '/mega/motor_left',
            lambda m: _ss(m_left=int(m.data),        m_last=time.time()), 10)
        sub(Int32,   '/mega/motor_right',
            lambda m: _ss(m_right=int(m.data),       m_last=time.time()), 10)
        sub(Int32,   '/mega/throttle',
            lambda m: _ss(m_throttle=int(m.data),    m_last=time.time()), 10)
        sub(Int32,   '/mega/steering',
            lambda m: _ss(m_steering=int(m.data),    m_last=time.time()), 10)
        sub(Float32MultiArray, '/mega/ultrasonics', self._sonic, 10)
        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(5.0, self._poll_nodes)
        _ss(ros_ok=True)
        _push_log('boot', 'server_gui_node ready')

    def _bat(self, m):
        pct = m.percentage * 100.0 if m.percentage <= 1.0 else m.percentage
        v, c = float(m.voltage), abs(float(m.current))
        _ss(bat_pct=float(pct), voltage=v, current=c,
            power=v * c, bat_last=time.time())

    def _odom(self, m):
        _ss(odom_x=float(m.pose.pose.position.x),
            odom_y=float(m.pose.pose.position.y),
            lin_vel=float(m.twist.twist.linear.x),
            ang_vel=float(m.twist.twist.angular.z),
            odom_last=time.time())

    def _imu(self, m):
        q = m.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        _ss(heading=(90.0 - math.degrees(yaw)) % 360.0)

    def _gps(self, m):
        lat = float(m.latitude)
        lon = float(m.longitude)
        fix = int(m.status.status)
        _ss(gps_lat=lat, gps_lon=lon, gps_alt=float(m.altitude),
            gps_fix=fix, gps_last=time.time())

    def _sonic(self, m):
        v = list(m.data[:8]) + [0.0] * (8 - len(m.data))
        _ss(m_sonics=v, m_last=time.time())

    def _poll_nodes(self):
        try:
            with _sl:
                _sm['ros_nodes'] = sorted(self.get_node_names())
        except Exception:
            pass

    def pub_twist(self, lin, ang):
        t = Twist()
        t.linear.x = float(lin)
        t.angular.z = float(ang)
        self._pub.publish(t)


def _ros_loop():
    global _node
    rclpy.init()
    _node = _ServerGUINode()
    try:
        rclpy.spin(_node)
    except Exception:
        pass
    finally:
        _ss(ros_ok=False)
        _push_log('warn', 'ROS2 node stopped')
        try:
            _node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
        _node = None


# ── Helpers ───────────────────────────────────────────────────────────────────

def _runtime(pct, amps):
    if amps < 0.2:
        return '--'
    h = 50.0 * (pct / 100.0) / amps
    hh, mm = int(h), int((h % 1) * 60)
    return f'~{hh}h{mm:02d}m' if hh > 0 else f'~{mm}m'


def _fmt_bps(n):
    for u in ('B', 'K', 'M', 'G'):
        if n < 1024:
            return f'{n:.0f}{u}/s'
        n /= 1024
    return f'{n:.1f}G/s'


def _fix_label(f):
    return {-1: 'NO FIX', 0: 'GPS', 1: 'SBAS', 2: 'RTK'}.get(f, 'UNK')


def _cardinal(deg):
    dirs = ['N', 'NNE', 'NE', 'ENE', 'E', 'ESE', 'SE', 'SSE',
            'S', 'SSW', 'SW', 'WSW', 'W', 'WNW', 'NW', 'NNW']
    return dirs[round(deg / 22.5) % 16]


def _uptime():
    s = int(time.time() - _start)
    h, rem = divmod(s, 3600)
    m, sec = divmod(rem, 60)
    return f'{h}h{m:02d}m{sec:02d}s'


def _bar(frac, width):
    """Return a filled/empty block bar string of exactly `width` chars."""
    if width <= 0:
        return ''
    n = int(max(0.0, min(1.0, frac)) * width)
    return '█' * n + '░' * (width - n)


def _clamp(s, w):
    return s[:w] if len(s) > w else s


# ── Curses Color Pairs ────────────────────────────────────────────────────────

_CP_TITLE  = 1
_CP_GOOD   = 2
_CP_WARN   = 3
_CP_BAD    = 4
_CP_DIM    = 5
_CP_ACCENT = 6


def _init_colors():
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(_CP_TITLE,  curses.COLOR_CYAN,   -1)
    curses.init_pair(_CP_GOOD,   curses.COLOR_GREEN,  -1)
    curses.init_pair(_CP_WARN,   curses.COLOR_YELLOW, -1)
    curses.init_pair(_CP_BAD,    curses.COLOR_RED,    -1)
    curses.init_pair(_CP_DIM,    curses.COLOR_WHITE,  -1)
    curses.init_pair(_CP_ACCENT, curses.COLOR_CYAN,   -1)


def _cp(pair, bold=False):
    a = curses.color_pair(pair)
    if bold:
        a |= curses.A_BOLD
    return a


def _put(win, y, x, s, attr=0):
    """Safe addstr that silently ignores out-of-bounds writes."""
    try:
        h, w = win.getmaxyx()
        if y < 0 or y >= h or x < 0 or x >= w:
            return
        avail = w - x - 1
        if avail <= 0:
            return
        win.addstr(y, x, s[:avail], attr)
    except curses.error:
        pass


def _hline(win, y, x, w):
    try:
        win.hline(y, x, curses.ACS_HLINE, w)
    except curses.error:
        pass


def _vline(win, y, x, h):
    try:
        win.vline(y, x, curses.ACS_VLINE, h)
    except curses.error:
        pass


# ── Draw Sections ─────────────────────────────────────────────────────────────

def _draw_header(win, s, cols):
    now = time.strftime('%Y-%m-%d %H:%M:%S')
    title = ' ROBOSNAILBOB CONTROL CENTER '
    _put(win, 0, 0, '═' * cols, _cp(_CP_TITLE))
    _put(win, 0, (cols - len(title)) // 2, title, _cp(_CP_TITLE, True))
    _put(win, 0, cols - len(now) - 2, now, _cp(_CP_DIM))

    now_t = time.time()
    bat_live  = s['bat_last']  > 0 and now_t - s['bat_last']  < STALE
    odom_live = s['odom_last'] > 0 and now_t - s['odom_last'] < STALE
    gps_live  = s['gps_last']  > 0 and now_t - s['gps_last']  < STALE
    drv_live  = s['m_last']    > 0 and now_t - s['m_last']    < STALE

    def dot(ok, label):
        attr = _cp(_CP_GOOD, True) if ok else _cp(_CP_DIM)
        return (f'● {label}', attr)

    ros_attr = _cp(_CP_GOOD, True) if s['ros_ok'] else _cp(_CP_BAD, True)
    _put(win, 1, 1, '● ROS2', ros_attr)
    col = 8
    for ok, lbl in [(bat_live, 'BAT'), (odom_live, 'ODOM'),
                    (gps_live and s['gps_fix'] >= 0, 'GPS'), (drv_live, 'DRIVE')]:
        txt, attr = dot(ok, lbl)
        _put(win, 1, col, txt, attr)
        col += len(txt) + 2

    model = os.environ.get('ROBOSNAILBOB_MODEL', 'llama3.2:1b')
    pers  = os.environ.get('ROBOSNAILBOB_PERSONALITY', 'default')
    info  = f'│ UP:{_uptime()}  │  {model} / {pers}'
    _put(win, 1, col + 2, info, _cp(_CP_DIM))
    _hline(win, 2, 0, cols)


def _draw_power(win, s, y0, x0, col_w):
    now = time.time()
    live = s['bat_last'] > 0 and now - s['bat_last'] < STALE
    pct  = s['bat_pct']
    v, c, p = s['voltage'], s['current'], s['power']
    dim  = _cp(_CP_DIM)

    if not live:
        bat_a = dim
    elif pct > 50:
        bat_a = _cp(_CP_GOOD, True)
    elif pct > 20:
        bat_a = _cp(_CP_WARN, True)
    else:
        bat_a = _cp(_CP_BAD, True)

    _put(win, y0,     x0, 'POWER', _cp(_CP_ACCENT, True))
    bw = max(0, col_w - 12)
    _put(win, y0 + 1, x0, f'Bat {pct:3.0f}%', bat_a)
    if bw > 2 and live:
        _put(win, y0 + 1, x0 + 8, f'[{_bar(pct / 100.0, bw)}]', bat_a)
    _put(win, y0 + 2, x0, f'V   {v:5.2f} V', dim if not live else 0)
    _put(win, y0 + 3, x0, f'A   {c:5.2f} A', dim if not live else 0)
    _put(win, y0 + 4, x0, f'W   {p:5.1f} W', dim if not live else 0)
    rt = _runtime(pct, c)
    _put(win, y0 + 5, x0, f'Est {rt}', _cp(_CP_GOOD) if live else dim)


def _draw_nav(win, s, y0, x0, col_w):
    now = time.time()
    gps_live  = s['gps_last']  > 0 and now - s['gps_last']  < STALE
    odom_live = s['odom_last'] > 0 and now - s['odom_last'] < STALE
    fix = s['gps_fix']
    dim = _cp(_CP_DIM)

    _put(win, y0, x0, 'NAVIGATION', _cp(_CP_ACCENT, True))

    if not gps_live:
        fix_a = dim
    elif fix < 0:
        fix_a = _cp(_CP_BAD)
    elif fix >= 2:
        fix_a = _cp(_CP_GOOD, True)
    else:
        fix_a = _cp(_CP_WARN)

    _put(win, y0 + 1, x0, f'GPS  {_fix_label(fix)}', fix_a)

    if gps_live and fix >= 0:
        _put(win, y0 + 2, x0, f'Lat  {s["gps_lat"]:11.7f}°', 0)
        _put(win, y0 + 3, x0, f'Lon  {s["gps_lon"]:11.7f}°', 0)
        _put(win, y0 + 4, x0, f'Alt  {s["gps_alt"]:7.1f} m', 0)
    else:
        _put(win, y0 + 2, x0, 'Lat  --', dim)
        _put(win, y0 + 3, x0, 'Lon  --', dim)
        _put(win, y0 + 4, x0, 'Alt  -- m', dim)

    _hline(win, y0 + 5, x0, col_w - 1)

    odom_a = 0 if odom_live else dim
    _put(win, y0 + 5, x0, 'ODOM', _cp(_CP_DIM, True))
    _put(win, y0 + 6, x0, f'X    {s["odom_x"]:7.3f} m', odom_a)
    _put(win, y0 + 7, x0, f'Y    {s["odom_y"]:7.3f} m', odom_a)
    lv = abs(s['lin_vel'])
    spd_a = _cp(_CP_GOOD) if (odom_live and lv > 0.02) else odom_a
    _put(win, y0 + 8, x0, f'Spd  {lv:5.3f} m/s', spd_a)
    _put(win, y0 + 9, x0, f'Turn {s["ang_vel"]:+6.3f} r/s', odom_a)

    hd = s.get('heading')
    if hd is not None:
        _put(win, y0 + 10, x0,
             f'Hdg  {hd:5.1f}° {_cardinal(hd)}',
             _cp(_CP_ACCENT))
    else:
        _put(win, y0 + 10, x0, 'Hdg  --', dim)


def _draw_system(win, sy, y0, x0, col_w):
    dim = _cp(_CP_DIM)
    _put(win, y0, x0, 'SYSTEM', _cp(_CP_ACCENT, True))

    cpus    = sy['cpu_pct']
    overall = sum(cpus) / len(cpus) if cpus else 0.0
    temp    = sy['cpu_temp']
    t_a = (_cp(_CP_GOOD) if temp < 70 else
           _cp(_CP_WARN) if temp < 85 else _cp(_CP_BAD))
    cpu_a = (_cp(_CP_GOOD) if overall < 60 else
             _cp(_CP_WARN) if overall < 85 else _cp(_CP_BAD))
    bw = max(0, col_w - 2)
    _put(win, y0 + 1, x0, f'CPU {overall:3.0f}%  {temp:.0f}°C', cpu_a)
    if bw > 2:
        _put(win, y0 + 2, x0, f'[{_bar(overall / 100.0, bw - 2)}]', cpu_a)
    _put(win, y0 + 2, x0 + bw - 6, f'{temp:.0f}°', t_a)

    ru, rt = sy['ram_used'], sy['ram_total']
    rp = ru / rt if rt else 0
    ram_a = (_cp(_CP_GOOD) if rp < 0.7 else
             _cp(_CP_WARN) if rp < 0.9 else _cp(_CP_BAD))
    _put(win, y0 + 3, x0,
         f'RAM {ru / 1e9:.1f}/{rt / 1e9:.1f}G {rp * 100:.0f}%', ram_a)
    if bw > 2:
        _put(win, y0 + 4, x0, f'[{_bar(rp, bw - 2)}]', ram_a)

    du, dt = sy['dsk_used'], sy['dsk_total']
    dp = du / dt if dt else 0
    dsk_a = (_cp(_CP_GOOD) if dp < 0.8 else
             _cp(_CP_WARN) if dp < 0.95 else _cp(_CP_BAD))
    _put(win, y0 + 5, x0,
         f'Dsk {du / 1e9:.0f}/{dt / 1e9:.0f}G  {dp * 100:.0f}%', dsk_a)
    _put(win, y0 + 6, x0,
         f'Net↓{_fmt_bps(sy["rx_bps"])} ↑{_fmt_bps(sy["tx_bps"])}',
         dim)

    _put(win, y0 + 7, x0, 'Procs:', dim)
    labels = list(WATCH_PROCS.keys())
    cw = max(1, (col_w - 1) // 2)
    for i, lbl in enumerate(labels):
        ok   = sy['procs'].get(lbl, False)
        attr = _cp(_CP_GOOD) if ok else dim
        dot  = '●' if ok else '○'
        r, c = divmod(i, 2)
        _put(win, y0 + 8 + r, x0 + c * cw,
             _clamp(f'{dot}{lbl}', cw - 1), attr)


def _draw_drive(win, s, y0, x0, cols):
    now = time.time()
    live  = s['m_last'] > 0 and now - s['m_last'] < STALE
    dim   = _cp(_CP_DIM)
    da    = 0 if live else dim

    _put(win, y0, x0, 'DRIVE', _cp(_CP_ACCENT, True))
    mode_ok = s['m_mode']
    m_a = _cp(_CP_GOOD, True) if mode_ok else dim
    _put(win, y0, x0 + 8,
         'RC ACTIVE' if mode_ok else 'STANDBY', m_a)

    thr, str_ = s['m_throttle'], s['m_steering']
    half = max(0, (cols - x0 - 2) // 2)
    bw   = max(0, half - 16)
    _put(win, y0 + 1, x0,
         f'Thr {thr:+4d} [{_bar((thr + 127) / 254.0, bw)}]  '
         f'Str {str_:+4d} [{_bar((str_ + 127) / 254.0, bw)}]', da)

    ml, mr = s['m_left'], s['m_right']
    mbw = max(0, half - 10)
    ml_a = (_cp(_CP_GOOD) if ml > 20 else _cp(_CP_BAD) if ml < -20 else dim)
    mr_a = (_cp(_CP_GOOD) if mr > 20 else _cp(_CP_BAD) if mr < -20 else dim)
    _put(win, y0 + 2, x0,
         f'L {ml:+4d} [{_bar((ml + 127) / 254.0, mbw)}]', ml_a)
    _put(win, y0 + 2, x0 + half,
         f'R {mr:+4d} [{_bar((mr + 127) / 254.0, mbw)}]', mr_a)

    parts = []
    for i, mm in enumerate(s['m_sonics']):
        if mm > 0:
            cm = mm / 10.0
            c = (_cp(_CP_GOOD) if cm > 100 else
                 _cp(_CP_WARN) if cm > 30 else _cp(_CP_BAD))
            parts.append((f'{SONIC_DIRS[i]}:{cm:3.0f}', c))
        else:
            parts.append((f'{SONIC_DIRS[i]}:---', dim))

    x = x0
    _put(win, y0 + 3, x, 'Prox ', dim)
    x += 5
    for txt, attr in parts:
        _put(win, y0 + 3, x, txt, attr)
        x += len(txt) + 2


def _draw_teleop(win, s, y0, x0, cols):
    en  = s['teleop_en']
    lin = s['cmd_lin']
    ang = s['cmd_ang']
    en_a = _cp(_CP_GOOD, True) if en else _cp(_CP_DIM)
    _put(win, y0, x0, 'TELEOP', _cp(_CP_ACCENT, True))
    en_txt = '[ENABLED ]' if en else '[DISABLED]'
    _put(win, y0, x0 + 8, en_txt, en_a)
    _put(win, y0, x0 + 20,
         't=Toggle  W/S=Fwd/Rev  A/D=L/R  Space=Stop',
         _cp(_CP_DIM))
    cmd_a = _cp(_CP_ACCENT) if en else _cp(_CP_DIM)
    _put(win, y0 + 1, x0,
         f'Cmd  Lin={lin:+6.3f} m/s   Ang={ang:+6.3f} r/s', cmd_a)


def _draw_log(win, logs, y0, x0, h_avail, cols):
    _put(win, y0, x0, 'VOICE LOG', _cp(_CP_ACCENT, True))
    visible = logs[-(h_avail):]
    for i, line in enumerate(visible):
        if i >= h_avail:
            break
        if '[hear]' in line:
            attr = _cp(_CP_ACCENT)
        elif '[say ]' in line:
            attr = _cp(_CP_GOOD)
        elif '[warn]' in line or '[boot]' in line:
            attr = _cp(_CP_WARN)
        else:
            attr = _cp(_CP_DIM)
        _put(win, y0 + 1 + i, x0, _clamp(line, cols - x0 - 1), attr)


def _draw_footer(win, rows, cols):
    help_txt = (
        ' q=Quit  t=Teleop  W/S=Fwd/Rev  A/D=L/R  Space=Stop '
    )
    _hline(win, rows - 1, 0, cols)
    _put(win, rows - 1, 1, help_txt, _cp(_CP_DIM))


# ── Key State ─────────────────────────────────────────────────────────────────

_kt = {}
_kl = threading.Lock()


def _on_key(k):
    with _kl:
        _kt[k] = time.time()


def _held(k):
    with _kl:
        t = _kt.get(k, 0.0)
    return (time.time() - t) < KEY_HOLD


# ── cmd_vel Publisher ─────────────────────────────────────────────────────────

def _compute_cmdvel():
    s = _gs()
    if not s['teleop_en']:
        if _node:
            _node.pub_twist(0.0, 0.0)
        _ss(cmd_lin=0.0, cmd_ang=0.0)
        return
    stop = _held(ord(' '))
    lin = ang = 0.0
    if not stop:
        if _held(ord('w')) or _held(ord('W')):
            lin = MAX_LIN
        if _held(ord('s')) or _held(ord('S')):
            lin = -MAX_LIN
        if _held(ord('a')) or _held(ord('A')):
            ang = MAX_ANG
        if _held(ord('d')) or _held(ord('D')):
            ang = -MAX_ANG
    if _node:
        _node.pub_twist(lin, ang)
    _ss(cmd_lin=lin, cmd_ang=ang)


# ── Main Draw Loop ────────────────────────────────────────────────────────────

MID_H = 12    # rows used by the 3-column section


def _curses_main(stdscr):
    _init_colors()
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(50)

    while True:
        try:
            key = stdscr.getch()
        except Exception:
            key = -1

        if key in (ord('q'), ord('Q')):
            break
        elif key in (ord('t'), ord('T')):
            s = _gs()
            new_en = not s['teleop_en']
            _ss(teleop_en=new_en)
            if not new_en and _node:
                _node.pub_twist(0.0, 0.0)
        elif key in (ord('w'), ord('W'), ord('s'), ord('S'),
                     ord('a'), ord('A'), ord('d'), ord('D'), ord(' ')):
            _on_key(key)

        _compute_cmdvel()

        s   = _gs()
        sy  = _get_sys()
        logs = _snap_log()

        try:
            rows, cols = stdscr.getmaxyx()
            stdscr.erase()

            if rows < MIN_ROWS or cols < MIN_COLS:
                _put(stdscr, rows // 2, max(0, (cols - 36) // 2),
                     f'Terminal too small ({cols}x{rows}). Need {MIN_COLS}x{MIN_ROWS}.',
                     _cp(_CP_BAD, True))
                stdscr.refresh()
                continue

            # ── Header  (rows 0–2) ────────────────────────────────────────────
            _draw_header(stdscr, s, cols)

            # ── 3-column section  (rows 3 … 3+MID_H-1) ───────────────────────
            mid_y = 3
            cw = cols // 3
            for r in range(mid_y, mid_y + MID_H):
                _vline(stdscr, r, cw,         1)
                _vline(stdscr, r, cw * 2,     1)

            _draw_power(stdscr, s,  mid_y, 1,       cw - 2)
            _draw_nav(  stdscr, s,  mid_y, cw + 1,  cw - 2)
            _draw_system(stdscr, sy, mid_y, cw * 2 + 1, cols - cw * 2 - 2)

            # ── Drive  (4 rows) ───────────────────────────────────────────────
            drive_y = mid_y + MID_H
            _hline(stdscr, drive_y, 0, cols)
            _draw_drive(stdscr, s, drive_y + 1, 1, cols)

            # ── Teleop  (2 rows) ──────────────────────────────────────────────
            teleop_y = drive_y + 5
            _hline(stdscr, teleop_y, 0, cols)
            _draw_teleop(stdscr, s, teleop_y + 1, 1, cols)

            # ── Voice log  (remaining rows) ───────────────────────────────────
            log_y   = teleop_y + 3
            _hline(stdscr, log_y, 0, cols)
            log_h   = max(1, rows - log_y - 2)
            _draw_log(stdscr, logs, log_y + 1, 1, log_h, cols)

            # ── Footer ────────────────────────────────────────────────────────
            _draw_footer(stdscr, rows, cols)

            stdscr.refresh()

        except curses.error:
            pass


# ── Entry Point ───────────────────────────────────────────────────────────────

def main(args=None):
    threading.Thread(target=_ros_loop, daemon=True).start()
    threading.Thread(target=_sys_loop, daemon=True).start()
    time.sleep(0.4)   # let ROS init before first frame

    try:
        curses.wrapper(_curses_main)
    except KeyboardInterrupt:
        pass
    finally:
        _ss(ros_ok=False)
        if _node:
            try:
                _node.pub_twist(0.0, 0.0)
                _node.destroy_node()
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()
