#!/usr/bin/env python3
"""RoboSnailBob Teleop & Status Dashboard
Tabs: STATUS | TELEOP | DRIVE | SYSTEM | LOG
"""
import math
import threading
import time
from collections import deque

try:
    import psutil
    HAS_PSUTIL = True
except ImportError:
    HAS_PSUTIL = False
    print("[warn] psutil missing — pip install psutil --break-system-packages")

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool, Int32, Float32MultiArray
import dearpygui.dearpygui as dpg

# ═══════════════════════════════════════════════════════════════════════════════
# SHARED STATE
# ═══════════════════════════════════════════════════════════════════════════════
_lock  = threading.Lock()
_state = {
    "battery_pct": 0.0, "voltage":    0.0,  "current":    0.0,
    "power":       0.0, "bat_last":   0.0,
    "odom_x":      0.0, "odom_y":     0.0,  "lin_vel":    0.0,
    "ang_vel":     0.0, "odom_last":  0.0,
    "gps_lat":     0.0, "gps_lon":    0.0,  "gps_alt":    0.0,
    "gps_fix":    -1,   "gps_last":   0.0,
    "m_mode":    False, "m_throttle": 0,    "m_steering": 0,
    "m_left":      0,   "m_right":    0,    "m_sonics":   [0.0]*8,
    "m_last":      0.0, "ros_ok":    False,
}
STALE = 6.0

def GS():
    with _lock: return dict(_state)

def SS(**kw):
    with _lock: _state.update(kw)

# ═══════════════════════════════════════════════════════════════════════════════
# HISTORY BUFFERS
# ═══════════════════════════════════════════════════════════════════════════════
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

# ═══════════════════════════════════════════════════════════════════════════════
# SYSTEM METRICS
# ═══════════════════════════════════════════════════════════════════════════════
_sl = threading.Lock()
_sm = {
    "cpu_pct": [], "cpu_temp": 0.0,
    "ram_used": 0, "ram_total": 1,
    "disk_used": 0, "disk_total": 1,
    "rx_bps": 0.0, "tx_bps": 0.0,
    "procs": {}, "ros_nodes": [],
}

WATCH_PROCS = {
    "Ollama":     "ollama",
    "Kinect2":    "kinect2_bridge",
    "RTABMap":    "rtabmap",
    "MegaBridge": "mega_bridge_node",
    "Battery":    "pzem_battery",
    "GPS":        "gps_node",
    "microROS":   "micro_ros_agent",
    "TeleopUI":   "teleop_ui",
}

def _sys_thread():
    if not HAS_PSUTIL: return
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
                        temp = e.current; break
            except Exception: pass
            ram  = psutil.virtual_memory()
            dsk  = psutil.disk_usage('/')
            net1 = psutil.net_io_counters()
            rx = (net1.bytes_recv - net0.bytes_recv) / 2.0
            tx = (net1.bytes_sent - net0.bytes_sent) / 2.0
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
                _sm.update(cpu_pct=cpu, cpu_temp=temp,
                           ram_used=ram.used, ram_total=ram.total,
                           disk_used=dsk.used, disk_total=dsk.total,
                           rx_bps=rx, tx_bps=tx, procs=procs)
        except Exception: pass

def get_sys():
    with _sl: return dict(_sm)

# ═══════════════════════════════════════════════════════════════════════════════
# VOICE LOG
# ═══════════════════════════════════════════════════════════════════════════════
_log = deque(maxlen=60)
_ll  = threading.Lock()

def push_log(src, txt):
    ts = time.strftime("%H:%M:%S")
    with _ll: _log.append(f"[{ts}][{src}] {txt}")

def snap_log():
    with _ll: return list(_log)

# ═══════════════════════════════════════════════════════════════════════════════
# KEY STATE
# ═══════════════════════════════════════════════════════════════════════════════
KEY_HOLD = 0.15
_kt = {}
_kl = threading.Lock()

def on_key(k):
    with _kl: _kt[k] = time.time()

def held(k):
    with _kl: t = _kt.get(k, 0.0)
    return (time.time() - t) < KEY_HOLD

# ═══════════════════════════════════════════════════════════════════════════════
# ROS2 NODE
# ═══════════════════════════════════════════════════════════════════════════════
_node = None

class UINode(Node):
    def __init__(self):
        super().__init__("teleop_ui_node")
        sub = self.create_subscription
        sub(BatteryState,      "/battery/state",    self._bat,   10)
        sub(Odometry,          "/odom",             self._odom,  10)
        sub(NavSatFix,         "/fix",              self._gps,   10)
        sub(String,            "/voice/input",      lambda m: push_log("hear", m.data), 10)
        sub(String,            "/voice/output",     lambda m: push_log("say",  m.data), 10)
        sub(Bool,              "/mega/mode",        lambda m: SS(m_mode=bool(m.data),    m_last=time.time()), 10)
        sub(Int32,             "/mega/motor_left",  lambda m: SS(m_left=int(m.data),     m_last=time.time()), 10)
        sub(Int32,             "/mega/motor_right", lambda m: SS(m_right=int(m.data),    m_last=time.time()), 10)
        sub(Int32,             "/mega/throttle",    lambda m: SS(m_throttle=int(m.data), m_last=time.time()), 10)
        sub(Int32,             "/mega/steering",    lambda m: SS(m_steering=int(m.data), m_last=time.time()), 10)
        sub(Float32MultiArray, "/mega/ultrasonics", self._sonic, 10)
        self._pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_timer(5.0, self._nodes)
        SS(ros_ok=True); push_log("boot", "teleop_ui_node ready")

    def _bat(self, m):
        pct = m.percentage * 100.0 if m.percentage <= 1.0 else m.percentage
        v, c = float(m.voltage), abs(float(m.current))
        SS(battery_pct=float(pct), voltage=v, current=c, power=v*c, bat_last=time.time())
        push_power(v, c)

    def _odom(self, m):
        SS(odom_x=float(m.pose.pose.position.x),
           odom_y=float(m.pose.pose.position.y),
           lin_vel=float(m.twist.twist.linear.x),
           ang_vel=float(m.twist.twist.angular.z),
           odom_last=time.time())

    def _gps(self, m):
        lat, lon, fix = float(m.latitude), float(m.longitude), int(m.status.status)
        SS(gps_lat=lat, gps_lon=lon, gps_alt=float(m.altitude),
           gps_fix=fix, gps_last=time.time())
        if fix >= 0: push_gps_pos(lat, lon)

    def _sonic(self, m):
        v = list(m.data[:8]) + [0.0] * (8 - len(m.data))
        SS(m_sonics=v, m_last=time.time())

    def _nodes(self):
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

# ═══════════════════════════════════════════════════════════════════════════════
# THEME
# ═══════════════════════════════════════════════════════════════════════════════
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

# ═══════════════════════════════════════════════════════════════════════════════
# HELPERS
# ═══════════════════════════════════════════════════════════════════════════════
def sec(label):
    dpg.add_spacer(height=8)
    dpg.add_text(label, color=(0, 160, 220, 255))
    dpg.add_separator()
    dpg.add_spacer(height=3)

def fmt_bytes(n):
    for u in ('B/s','KB/s','MB/s','GB/s'):
        if n < 1024: return f"{n:.0f} {u}"
        n /= 1024
    return f"{n:.1f} TB/s"

def fmt_runtime(pct, amps):
    if amps < 0.2: return "--"
    h = 50.0 * (pct / 100.0) / amps
    hh, mm = int(h), int((h % 1) * 60)
    return f"~{hh}h {mm:02d}m" if hh > 0 else f"~{mm}m"

def fix_label(f):
    return {-1:"NO FIX", 0:"GPS", 1:"SBAS", 2:"RTK/DGPS"}.get(f, "UNKNOWN")

def fix_color(f, live):
    if not live:     return (120, 120, 140, 255)
    if f == 2:       return (  0, 220, 120, 255)
    if f >= 0:       return (220, 180,   0, 255)
    return                  (220,  60,  60, 255)

def dot_upd(tag, ok, label):
    dpg.set_value(tag, f"● {label}")
    dpg.configure_item(tag, color=(0,220,120,255) if ok else (120,120,140,255))

# ═══════════════════════════════════════════════════════════════════════════════
# TAB: STATUS
# ═══════════════════════════════════════════════════════════════════════════════
def build_status():
    with dpg.tab(label="  STATUS  "):
        sec("POWER")
        dpg.add_text("Battery    0%",    tag="s_bat")
        dpg.add_progress_bar(default_value=0.0, width=-1, height=16,
                             tag="s_bat_bar", overlay="0%")
        dpg.add_spacer(height=2)
        dpg.add_text("Voltage    0.00 V",  tag="s_volt")
        dpg.add_text("Current    0.00 A",  tag="s_curr")
        dpg.add_text("Power      0.0 W",   tag="s_pwr")
        dpg.add_text("Runtime    --",       tag="s_runtime",
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
        dpg.add_text("Fix     NO FIX",  tag="s_gfix",
                     color=(120, 120, 140, 255))
        dpg.add_text("Lat     --",       tag="s_glat")
        dpg.add_text("Lon     --",       tag="s_glon")
        dpg.add_text("Alt     -- m",     tag="s_galt")

        sec("GPS TRACK")
        with dpg.plot(height=140, width=-1, no_mouse_pos=True):
            dpg.add_plot_axis(dpg.mvXAxis, label="E (m)", tag="gx")
            with dpg.plot_axis(dpg.mvYAxis, label="N (m)", tag="gy"):
                dpg.add_line_series([],  [],  tag="gtrack")
                dpg.add_scatter_series([], [], tag="gpos")

        sec("ODOMETRY")
        dpg.add_text("X        0.000 m",    tag="s_ox")
        dpg.add_text("Y        0.000 m",    tag="s_oy")
        dpg.add_text("Linear   0.000 m/s",  tag="s_lv")
        dpg.add_text("Angular  0.000 rad/s",tag="s_av")

        sec("CONNECTIONS")
        dpg.add_text("● ROS2",    tag="d_ros",  color=(120, 120, 140, 255))
        dpg.add_text("● Battery", tag="d_bat",  color=(120, 120, 140, 255))
        dpg.add_text("● Odom",    tag="d_odom", color=(120, 120, 140, 255))
        dpg.add_text("● GPS",     tag="d_gps",  color=(120, 120, 140, 255))
        dpg.add_text("● Drive",   tag="d_drv",  color=(120, 120, 140, 255))

# ═══════════════════════════════════════════════════════════════════════════════
# TAB: TELEOP
# ═══════════════════════════════════════════════════════════════════════════════
def build_teleop():
    with dpg.tab(label="  TELEOP  "):
        sec("ENABLE")
        dpg.add_checkbox(label=" TELEOP ENABLED", tag="t_en",
                         default_value=False)
        dpg.add_spacer(height=2)
        dpg.add_text("⚠ Uncheck to stop publishing cmd_vel",
                     color=(180, 140, 0, 255))

        sec("KEYBOARD CONTROLS")
        dpg.add_text("W  /  ↑     Forward",    color=(180, 210, 240, 255))
        dpg.add_text("S  /  ↓     Reverse",    color=(180, 210, 240, 255))
        dpg.add_text("A  /  ←     Turn Left",  color=(180, 210, 240, 255))
        dpg.add_text("D  /  →     Turn Right", color=(180, 210, 240, 255))
        dpg.add_text("Space       STOP",        color=(220,  80,  80, 255))

        sec("VELOCITY LIMITS")
        dpg.add_slider_float(label="Max Linear  (m/s)",
                             tag="t_maxlin", default_value=0.4,
                             min_value=0.0, max_value=1.0, width=-1)
        dpg.add_spacer(height=4)
        dpg.add_slider_float(label="Max Angular (rad/s)",
                             tag="t_maxang", default_value=1.0,
                             min_value=0.0, max_value=2.0, width=-1)

        sec("LIVE OUTPUT")
        dpg.add_text("● DISABLED",          tag="t_status",
                     color=(120, 120, 140, 255))
        dpg.add_text("Linear:  0.000 m/s",  tag="t_lin",
                     color=(160, 200, 220, 255))
        dpg.add_text("Angular: 0.000 rad/s",tag="t_ang",
                     color=(160, 200, 220, 255))

# ═══════════════════════════════════════════════════════════════════════════════
# TAB: DRIVE
# ═══════════════════════════════════════════════════════════════════════════════
SONIC_DIRS   = ["FL", "FC", "FR", " R", "RR", "RC", "RL", " L"]
SONIC_MAX_MM = 6000.0

def build_drive():
    with dpg.tab(label="  DRIVE  "):
        sec("MODE")
        dpg.add_text("● UNKNOWN",  tag="dr_mode", color=(120, 120, 140, 255))

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

        sec("PROXIMITY  (cm)  [stubs until wired]")
        for i in range(8):
            dpg.add_text(f"{SONIC_DIRS[i]}  --",
                         tag=f"dr_s{i}", color=(120, 120, 140, 255))
            dpg.add_progress_bar(default_value=0.0, width=-1, height=8,
                                 tag=f"dr_sb{i}")
            dpg.add_spacer(height=1)

# ═══════════════════════════════════════════════════════════════════════════════
# TAB: SYSTEM
# ═══════════════════════════════════════════════════════════════════════════════
_NUM_CORES = psutil.cpu_count() if HAS_PSUTIL else 20

def build_system():
    with dpg.tab(label="  SYSTEM  "):
        sec("CPU")
        dpg.add_text("Package  -- °C",  tag="sy_temp",
                     color=(160, 200, 220, 255))
        dpg.add_text("Overall  0%",      tag="sy_overall")
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
        e_start = p_end
        if e_start < _NUM_CORES:
            dpg.add_spacer(height=4)
            dpg.add_text(f"E-cores ({e_start}–{_NUM_CORES-1})",
                         color=(100, 160, 200, 255))
            for i in range(e_start, _NUM_CORES):
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
            tag = f"sy_p_{lbl}"
            dpg.add_text(f"● {lbl}", tag=tag, color=(120, 120, 140, 255))

        sec("ROS2 NODES")
        dpg.add_input_text(tag="sy_nodes", multiline=True, readonly=True,
                           width=-1, height=120, default_value="(none)")

# ═══════════════════════════════════════════════════════════════════════════════
# TAB: LOG
# ═══════════════════════════════════════════════════════════════════════════════
def build_log():
    with dpg.tab(label="  LOG  "):
        sec("VOICE / SYSTEM LOG")
        dpg.add_input_text(tag="log_txt", multiline=True, readonly=True,
                           width=-1, height=-1, default_value="")

# ═══════════════════════════════════════════════════════════════════════════════
# PER-FRAME UPDATES
# ═══════════════════════════════════════════════════════════════════════════════
def upd_status(s):
    now = time.time()
    bat_live  = s["bat_last"]  > 0 and now - s["bat_last"]  < STALE
    odom_live = s["odom_last"] > 0 and now - s["odom_last"] < STALE
    gps_live  = s["gps_last"]  > 0 and now - s["gps_last"]  < STALE
    drv_live  = s["m_last"]    > 0 and now - s["m_last"]    < STALE

    pct = s["battery_pct"]
    dpg.set_value("s_bat",      f"Battery    {pct:.0f}%")
    dpg.set_value("s_bat_bar",   pct / 100.0)
    dpg.configure_item("s_bat_bar", overlay=f"{pct:.0f}%")
    dpg.set_value("s_volt",     f"Voltage    {s['voltage']:.2f} V")
    dpg.set_value("s_curr",     f"Current    {s['current']:.2f} A")
    dpg.set_value("s_pwr",      f"Power      {s['power']:.1f} W")
    dpg.set_value("s_runtime",  f"Runtime    {fmt_runtime(pct, s['current'])}")

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

    tr = snap_buf(_gtrack)
    if len(tr) > 1:
        xs, ys = zip(*tr)
        xs, ys = list(xs), list(ys)
        dpg.set_value("gtrack", [xs, ys])
        dpg.set_value("gpos",   [[xs[-1]], [ys[-1]]])
        pad = max(max(xs)-min(xs), max(ys)-min(ys), 4.0) * 0.15
        dpg.set_axis_limits("gx", min(xs)-pad, max(xs)+pad)
        dpg.set_axis_limits("gy", min(ys)-pad, max(ys)+pad)

    dpg.set_value("s_ox", f"X        {s['odom_x']:.3f} m")
    dpg.set_value("s_oy", f"Y        {s['odom_y']:.3f} m")
    dpg.set_value("s_lv", f"Linear   {s['lin_vel']:.3f} m/s")
    dpg.set_value("s_av", f"Angular  {s['ang_vel']:.3f} rad/s")

    dot_upd("d_ros",  s["ros_ok"],           "ROS2")
    dot_upd("d_bat",  bat_live,               "Battery")
    dot_upd("d_odom", odom_live,              "Odom")
    dot_upd("d_gps",  gps_live and fix >= 0, "GPS")
    dot_upd("d_drv",  drv_live,               "Drive")

def upd_drive(s):
    mode_ok = s["m_mode"]
    dpg.set_value("dr_mode",
                  "● RC ACTIVE" if mode_ok else "● STANDBY")
    dpg.configure_item("dr_mode",
                       color=(0,220,120,255) if mode_ok else (120,120,140,255))

    thr, str_ = s["m_throttle"], s["m_steering"]
    dpg.set_value("dr_thr",     f"Throttle   {thr:+d}")
    dpg.set_value("dr_thr_bar", (thr + 100) / 200.0)
    dpg.configure_item("dr_thr_bar", overlay=f"{thr:+d}")
    dpg.set_value("dr_str",     f"Steering   {str_:+d}")
    dpg.set_value("dr_str_bar", (str_ + 100) / 200.0)
    dpg.configure_item("dr_str_bar", overlay=f"{str_:+d}")

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
                               color=(  0,220,120,255) if cm > 100 else
                                     (220,180,  0,255) if cm >  30 else
                                     (220, 60, 60,255))
        else:
            dpg.set_value(f"dr_s{i}", f"{SONIC_DIRS[i]}  --")
            dpg.configure_item(f"dr_s{i}", color=(120,120,140,255))
        dpg.set_value(f"dr_sb{i}",
                      min(mm, SONIC_MAX_MM) / SONIC_MAX_MM if mm > 0 else 0.0)

def upd_system(sy):
    cpus = sy["cpu_pct"]
    if cpus:
        overall = sum(cpus) / len(cpus)
        dpg.set_value("sy_overall",     f"Overall  {overall:.0f}%")
        dpg.set_value("sy_overall_bar",  overall / 100.0)
        dpg.configure_item("sy_overall_bar", overlay=f"{overall:.0f}%")
        for i, pct in enumerate(cpus[:_NUM_CORES]):
            if dpg.does_item_exist(f"sy_c{i}"):
                dpg.set_value(f"sy_c{i}", pct / 100.0)

    temp = sy["cpu_temp"]
    t_col = (  0,220,120,255) if temp < 70 else \
            (220,180,  0,255) if temp < 85 else \
            (220, 60, 60,255)
    dpg.set_value("sy_temp", f"Package  {temp:.1f} °C")
    dpg.configure_item("sy_temp", color=t_col)

    ru, rt = sy["ram_used"], sy["ram_total"]
    rp = ru / rt if rt else 0
    dpg.set_value("sy_ram_lbl", f"RAM   {ru/1e9:.1f} / {rt/1e9:.1f} GB")
    dpg.set_value("sy_ram_bar",  rp)
    dpg.configure_item("sy_ram_bar", overlay=f"{rp*100:.0f}%")

    du, dt = sy["disk_used"], sy["disk_total"]
    dp_ = du / dt if dt else 0
    dpg.set_value("sy_disk_lbl", f"Disk  {du/1e9:.1f} / {dt/1e9:.1f} GB")
    dpg.set_value("sy_disk_bar",  dp_)
    dpg.configure_item("sy_disk_bar", overlay=f"{dp_*100:.0f}%")

    dpg.set_value("sy_rx", f"RX  {fmt_bytes(sy['rx_bps'])}")
    dpg.set_value("sy_tx", f"TX  {fmt_bytes(sy['tx_bps'])}")

    for lbl in WATCH_PROCS:
        ok  = sy["procs"].get(lbl, False)
        tag = f"sy_p_{lbl}"
        dpg.set_value(tag, f"● {lbl}")
        dpg.configure_item(tag, color=(0,220,120,255) if ok else (120,120,140,255))

    nodes = sy.get("ros_nodes", [])
    dpg.set_value("sy_nodes", "\n".join(nodes) if nodes else "(none)")

# ═══════════════════════════════════════════════════════════════════════════════
# CMD_VEL
# ═══════════════════════════════════════════════════════════════════════════════
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
            dpg.configure_item("t_status", color=(120,120,140,255))
        else:
            dpg.set_value("t_status", "● PUBLISHING")
            dpg.configure_item("t_status", color=(0,220,120,255))
    else:
        if _node: _node.pub(0.0, 0.0)
        dpg.set_value("t_status", "● DISABLED")
        dpg.configure_item("t_status", color=(120,120,140,255))
    dpg.set_value("t_lin", f"Linear:  {lin:.3f} m/s")
    dpg.set_value("t_ang", f"Angular: {ang:.3f} rad/s")

# ═══════════════════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════════════════
def main():
    threading.Thread(target=_ros_thread, daemon=True).start()
    threading.Thread(target=_sys_thread, daemon=True).start()

    dpg.create_context()
    apply_theme()

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
        dpg.add_separator(); dpg.add_spacer(height=4)
        with dpg.tab_bar():
            build_status()
            build_teleop()
            build_drive()
            build_system()
            build_log()

    dpg.create_viewport(title="RoboSnailBob", width=560, height=820,
                        min_width=460, min_height=600)
    dpg.setup_dearpygui()
    dpg.show_viewport()
    dpg.set_primary_window("main", True)

    while dpg.is_dearpygui_running():
        s  = GS()
        sy = get_sys()
        upd_status(s)
        upd_drive(s)
        upd_system(sy)
        dpg.set_value("log_txt", "\n".join(snap_log()))
        compute_cmd_vel()
        dpg.render_dearpygui_frame()

    dpg.destroy_context()

if __name__ == "__main__":
    main()
