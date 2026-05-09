#!/usr/bin/env python3
import threading
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import dearpygui.dearpygui as dpg

# ── Shared robot state ────────────────────────────────────────────────────────
_lock = threading.Lock()
_state = {
    "battery_pct":  0.0,
    "voltage":      0.0,
    "current":      0.0,
    "power":        0.0,
    "linear_vel":   0.0,
    "angular_vel":  0.0,
    "odom_x":       0.0,
    "odom_y":       0.0,
    "battery_last": 0.0,
    "odom_last":    0.0,
    "ros_ok":       False,
}
STALE_SEC = 6.0

def get_state():
    with _lock:
        return dict(_state)

def set_state(**kwargs):
    with _lock:
        _state.update(kwargs)

# ── Key state via timestamps (VNC-safe) ───────────────────────────────────────
KEY_HOLD_MS = 0.15

_key_times      = {}
_key_times_lock = threading.Lock()

def on_key_down(key):
    with _key_times_lock:
        _key_times[key] = time.time()

def is_held(key):
    with _key_times_lock:
        t = _key_times.get(key, 0.0)
    return (time.time() - t) < KEY_HOLD_MS

# ── ROS2 node ─────────────────────────────────────────────────────────────────
_node = None

class TeleopUINode(Node):
    def __init__(self):
        super().__init__("teleop_ui_node")
        self.create_subscription(BatteryState, "/battery/state", self._battery_cb, 10)
        self.create_subscription(Odometry,     "/odom",          self._odom_cb,    10)
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        set_state(ros_ok=True)

    def _battery_cb(self, msg):
        pct = msg.percentage
        if pct <= 1.0:
            pct *= 100.0
        set_state(
            battery_pct  = float(pct),
            voltage      = float(msg.voltage),
            current      = float(msg.current),
            power        = float(msg.voltage * msg.current),
            battery_last = time.time(),
        )

    def _odom_cb(self, msg):
        set_state(
            odom_x       = float(msg.pose.pose.position.x),
            odom_y       = float(msg.pose.pose.position.y),
            linear_vel   = float(msg.twist.twist.linear.x),
            angular_vel  = float(msg.twist.twist.angular.z),
            odom_last    = time.time(),
        )

    def publish_cmd_vel(self, linear, angular):
        msg = Twist()
        msg.linear.x  = float(linear)
        msg.angular.z = float(angular)
        self._cmd_pub.publish(msg)

def _ros_thread():
    global _node
    rclpy.init()
    _node = TeleopUINode()
    try:
        rclpy.spin(_node)
    except Exception:
        pass
    finally:
        set_state(ros_ok=False)
        try:
            _node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
        _node = None

# ── Theme ─────────────────────────────────────────────────────────────────────
def apply_theme():
    with dpg.theme() as global_theme:
        with dpg.theme_component(dpg.mvAll):
            dpg.add_theme_color(dpg.mvThemeCol_WindowBg,        ( 13,  17,  23, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ChildBg,         ( 20,  26,  35, 255))
            dpg.add_theme_color(dpg.mvThemeCol_PopupBg,         ( 20,  26,  35, 255))
            dpg.add_theme_color(dpg.mvThemeCol_Border,          ( 48,  68,  95, 180))
            dpg.add_theme_color(dpg.mvThemeCol_FrameBg,         ( 30,  40,  55, 255))
            dpg.add_theme_color(dpg.mvThemeCol_FrameBgHovered,  ( 45,  62,  85, 255))
            dpg.add_theme_color(dpg.mvThemeCol_TitleBg,         ( 10,  14,  20, 255))
            dpg.add_theme_color(dpg.mvThemeCol_TitleBgActive,   (  0, 140, 200, 255))
            dpg.add_theme_color(dpg.mvThemeCol_Tab,             ( 20,  26,  35, 255))
            dpg.add_theme_color(dpg.mvThemeCol_TabHovered,      (  0, 140, 200, 180))
            dpg.add_theme_color(dpg.mvThemeCol_TabActive,       (  0, 100, 160, 255))
            dpg.add_theme_color(dpg.mvThemeCol_Header,          (  0, 100, 160, 180))
            dpg.add_theme_color(dpg.mvThemeCol_HeaderHovered,   (  0, 140, 200, 200))
            dpg.add_theme_color(dpg.mvThemeCol_Button,          (  0, 100, 160, 220))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered,   (  0, 140, 200, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive,    (  0,  80, 130, 255))
            dpg.add_theme_color(dpg.mvThemeCol_Text,            (200, 220, 240, 255))
            dpg.add_theme_color(dpg.mvThemeCol_CheckMark,       (  0, 200, 255, 255))
            dpg.add_theme_color(dpg.mvThemeCol_SliderGrab,      (  0, 140, 200, 255))
            dpg.add_theme_color(dpg.mvThemeCol_SliderGrabActive,(  0, 200, 255, 255))
            dpg.add_theme_color(dpg.mvThemeCol_PlotHistogram,   (  0, 160, 220, 255))
            dpg.add_theme_style(dpg.mvStyleVar_WindowRounding,  6)
            dpg.add_theme_style(dpg.mvStyleVar_FrameRounding,   4)
            dpg.add_theme_style(dpg.mvStyleVar_TabRounding,     4)
            dpg.add_theme_style(dpg.mvStyleVar_WindowPadding,   12, 12)
            dpg.add_theme_style(dpg.mvStyleVar_ItemSpacing,      8,  6)
            dpg.add_theme_style(dpg.mvStyleVar_FramePadding,     6,  4)
    dpg.bind_theme(global_theme)

# ── Helpers ───────────────────────────────────────────────────────────────────
def section(label):
    dpg.add_spacer(height=10)
    dpg.add_text(label, color=(0, 160, 220, 255))
    dpg.add_separator()
    dpg.add_spacer(height=4)

# ── Tab: STATUS ───────────────────────────────────────────────────────────────
def build_status_tab():
    with dpg.tab(label="  STATUS  ", tag="tab_status"):

        section("POWER")
        dpg.add_text("Battery    0%", tag="lbl_battery_pct")
        dpg.add_progress_bar(default_value=0.0, width=-1, height=18,
                             tag="bar_battery", overlay="0%")
        dpg.add_spacer(height=2)
        dpg.add_text("Voltage    0.00 V", tag="lbl_voltage")
        dpg.add_text("Current    0.00 A", tag="lbl_current")
        dpg.add_text("Power      0.0 W",  tag="lbl_power")

        section("ODOMETRY")
        dpg.add_text("X           0.000 m",    tag="lbl_odom_x")
        dpg.add_text("Y           0.000 m",    tag="lbl_odom_y")
        dpg.add_text("Linear      0.000 m/s",  tag="lbl_linear")
        dpg.add_text("Angular     0.000 rad/s",tag="lbl_angular")

        section("CONNECTIONS")
        dpg.add_text("● ROS2",    tag="dot_ros",     color=(120, 120, 140, 255))
        dpg.add_text("● Battery", tag="dot_battery", color=(120, 120, 140, 255))
        dpg.add_text("● Odom",    tag="dot_odom",    color=(120, 120, 140, 255))

# ── Tab: TELEOP ───────────────────────────────────────────────────────────────
def build_teleop_tab():
    with dpg.tab(label="  TELEOP  ", tag="tab_teleop"):

        section("ENABLE")
        dpg.add_checkbox(label=" TELEOP ENABLED", tag="chk_enabled",
                         default_value=False)
        dpg.add_spacer(height=2)
        dpg.add_text("⚠ Uncheck to stop publishing cmd_vel",
                     color=(180, 140, 0, 255))

        section("KEYBOARD CONTROLS")
        dpg.add_text("W  /  ↑     Forward",    color=(180, 210, 240, 255))
        dpg.add_text("S  /  ↓     Reverse",    color=(180, 210, 240, 255))
        dpg.add_text("A  /  ←     Turn Left",  color=(180, 210, 240, 255))
        dpg.add_text("D  /  →     Turn Right", color=(180, 210, 240, 255))
        dpg.add_text("Space       STOP",        color=(220,  80,  80, 255))

        section("VELOCITY LIMITS")
        dpg.add_slider_float(label="Max Linear  (m/s)",
                             tag="slider_linear",  default_value=0.4,
                             min_value=0.0, max_value=1.0, width=-1)
        dpg.add_spacer(height=4)
        dpg.add_slider_float(label="Max Angular (rad/s)",
                             tag="slider_angular", default_value=1.0,
                             min_value=0.0, max_value=2.0, width=-1)

        section("LIVE OUTPUT")
        dpg.add_text("● DISABLED",          tag="lbl_teleop_status",
                     color=(120, 120, 140, 255))
        dpg.add_text("Linear:  0.000 m/s",   tag="lbl_cmd_linear",
                     color=(160, 200, 220, 255))
        dpg.add_text("Angular: 0.000 rad/s", tag="lbl_cmd_angular",
                     color=(160, 200, 220, 255))

# ── Tab: LOG ──────────────────────────────────────────────────────────────────
def build_log_tab():
    with dpg.tab(label="  LOG  ", tag="tab_log"):
        section("SYSTEM LOG")
        dpg.add_text("[boot]  teleop_ui starting...",      color=(120, 160, 120, 255))
        dpg.add_text("[info]  waiting for ROS2 topics...", color=(160, 160, 180, 255),
                     tag="lbl_log_ros")

# ── Teleop logic ──────────────────────────────────────────────────────────────
def compute_and_publish_cmd_vel():
    enabled = dpg.get_value("chk_enabled")
    max_lin = dpg.get_value("slider_linear")
    max_ang = dpg.get_value("slider_angular")

    linear  = 0.0
    angular = 0.0

    if enabled:
        space = is_held(dpg.mvKey_Spacebar)
        if not space:
            if is_held(dpg.mvKey_W) or is_held(dpg.mvKey_Up):    linear  =  max_lin
            if is_held(dpg.mvKey_S) or is_held(dpg.mvKey_Down):  linear  = -max_lin
            if is_held(dpg.mvKey_A) or is_held(dpg.mvKey_Left):  angular =  max_ang
            if is_held(dpg.mvKey_D) or is_held(dpg.mvKey_Right): angular = -max_ang

        if _node is not None:
            _node.publish_cmd_vel(linear, angular)

        if space or (linear == 0.0 and angular == 0.0):
            dpg.set_value("lbl_teleop_status", "● IDLE")
            dpg.configure_item("lbl_teleop_status", color=(120, 120, 140, 255))
        else:
            dpg.set_value("lbl_teleop_status", "● PUBLISHING")
            dpg.configure_item("lbl_teleop_status", color=(0, 220, 120, 255))
    else:
        if _node is not None:
            _node.publish_cmd_vel(0.0, 0.0)
        dpg.set_value("lbl_teleop_status", "● DISABLED")
        dpg.configure_item("lbl_teleop_status", color=(120, 120, 140, 255))

    dpg.set_value("lbl_cmd_linear",  f"Linear:  {linear:.3f} m/s")
    dpg.set_value("lbl_cmd_angular", f"Angular: {angular:.3f} rad/s")

# ── Per-frame UI update ───────────────────────────────────────────────────────
def update_ui(s):
    now = time.time()
    battery_live = (now - s["battery_last"]) < STALE_SEC and s["battery_last"] > 0
    odom_live    = (now - s["odom_last"])    < STALE_SEC and s["odom_last"]    > 0

    pct = s["battery_pct"]
    dpg.set_value("lbl_battery_pct", f"Battery    {pct:.0f}%")
    dpg.set_value("bar_battery", pct / 100.0)
    dpg.configure_item("bar_battery", overlay=f"{pct:.0f}%")
    dpg.set_value("lbl_voltage", f"Voltage    {s['voltage']:.2f} V")
    dpg.set_value("lbl_current", f"Current    {s['current']:.2f} A")
    dpg.set_value("lbl_power",   f"Power      {s['power']:.1f} W")

    dpg.set_value("lbl_odom_x",  f"X           {s['odom_x']:.3f} m")
    dpg.set_value("lbl_odom_y",  f"Y           {s['odom_y']:.3f} m")
    dpg.set_value("lbl_linear",  f"Linear      {s['linear_vel']:.3f} m/s")
    dpg.set_value("lbl_angular", f"Angular     {s['angular_vel']:.3f} rad/s")

    def dot(tag, ok, label):
        color = (0, 220, 120, 255) if ok else (120, 120, 140, 255)
        dpg.set_value(tag, f"● {label}")
        dpg.configure_item(tag, color=color)

    dot("dot_ros",     s["ros_ok"],  "ROS2")
    dot("dot_battery", battery_live, "Battery")
    dot("dot_odom",    odom_live,    "Odom")

    if s["ros_ok"]:
        dpg.set_value("lbl_log_ros", "[info]  ROS2 node running")
        dpg.configure_item("lbl_log_ros", color=(0, 200, 120, 255))

# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    t = threading.Thread(target=_ros_thread, daemon=True)
    t.start()

    dpg.create_context()
    apply_theme()

    # Explicit zero-param lambdas — avoids DearPyGui overriding captured key arg
    with dpg.handler_registry():
        dpg.add_key_down_handler(dpg.mvKey_W,        callback=lambda: on_key_down(dpg.mvKey_W))
        dpg.add_key_down_handler(dpg.mvKey_S,        callback=lambda: on_key_down(dpg.mvKey_S))
        dpg.add_key_down_handler(dpg.mvKey_A,        callback=lambda: on_key_down(dpg.mvKey_A))
        dpg.add_key_down_handler(dpg.mvKey_D,        callback=lambda: on_key_down(dpg.mvKey_D))
        dpg.add_key_down_handler(dpg.mvKey_Up,       callback=lambda: on_key_down(dpg.mvKey_Up))
        dpg.add_key_down_handler(dpg.mvKey_Down,     callback=lambda: on_key_down(dpg.mvKey_Down))
        dpg.add_key_down_handler(dpg.mvKey_Left,     callback=lambda: on_key_down(dpg.mvKey_Left))
        dpg.add_key_down_handler(dpg.mvKey_Right,    callback=lambda: on_key_down(dpg.mvKey_Right))
        dpg.add_key_down_handler(dpg.mvKey_Spacebar, callback=lambda: on_key_down(dpg.mvKey_Spacebar))

    with dpg.window(tag="main_window"):
        dpg.add_text("ROBOSNAILBOB", color=(0, 200, 255, 255))
        dpg.add_text("Teleop & Status Dashboard",
                     color=(100, 130, 160, 255), indent=2)
        dpg.add_separator()
        dpg.add_spacer(height=4)
        with dpg.tab_bar():
            build_status_tab()
            build_teleop_tab()
            build_log_tab()

    dpg.create_viewport(title="RoboSnailBob", width=480, height=640,
                        min_width=400, min_height=500)
    dpg.setup_dearpygui()
    dpg.show_viewport()
    dpg.set_primary_window("main_window", True)

    while dpg.is_dearpygui_running():
        s = get_state()
        update_ui(s)
        compute_and_publish_cmd_vel()
        dpg.render_dearpygui_frame()

    dpg.destroy_context()

if __name__ == "__main__":
    main()
