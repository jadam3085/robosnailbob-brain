#!/usr/bin/env bash
# robot_tuning.sh — system-level tuning for RoboSnailBob
#
# Fixes for camera drops, ping stalls, and USB disconnects:
#   1. Disable USB autosuspend (cameras lose power mid-stream)
#   2. Increase UDP socket buffers (camera bursts overflow defaults)
#   3. Set CPU governor to performance (avoids freq-scaling latency)
#   4. Lower Ollama process priority (prevents LLM from starving camera threads)
#   5. Optimize WiFi power management (latency spikes cause DDS heartbeat loss)
#
# Run at boot:  sudo bash robot_tuning.sh
# Or add to /etc/rc.local or a systemd service.

set -euo pipefail

log() { echo "[robot_tuning] $*"; }

# ── 1. USB autosuspend ────────────────────────────────────────────────────────
# USB cameras (including Kinect2) can enter low-power suspend and drop frames.
# Setting autosuspend_delay_ms to -1 disables it.
log "Disabling USB autosuspend..."
for f in /sys/bus/usb/devices/*/power/autosuspend_delay_ms; do
    echo -1 | sudo tee "$f" > /dev/null 2>&1 || true
done
for f in /sys/bus/usb/devices/*/power/control; do
    echo "on" | sudo tee "$f" > /dev/null 2>&1 || true
done
log "USB autosuspend disabled."

# ── 2. Network socket buffers ─────────────────────────────────────────────────
# Default rmem_max (~212 KB) is too small for camera image bursts over DDS/UDP.
# 64 MB allows the kernel to buffer many frames without dropping.
log "Setting socket receive/send buffers..."
sudo sysctl -w net.core.rmem_max=67108864     > /dev/null
sudo sysctl -w net.core.wmem_max=67108864     > /dev/null
sudo sysctl -w net.core.rmem_default=16777216 > /dev/null
sudo sysctl -w net.core.wmem_default=4194304  > /dev/null
sudo sysctl -w net.ipv4.udp_rmem_min=8192     > /dev/null
sudo sysctl -w net.ipv4.udp_wmem_min=8192     > /dev/null
log "Socket buffers set."

# ── 3. CPU governor ───────────────────────────────────────────────────────────
# Freq-scaling can cause millisecond latency spikes that delay DDS/USB processing.
log "Setting CPU governor to performance..."
for cpu in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
    [ -f "$cpu" ] && echo performance | sudo tee "$cpu" > /dev/null 2>&1 || true
done
log "CPU governor set."

# ── 4. WiFi power management ──────────────────────────────────────────────────
# WiFi power-save mode can add 50-200 ms latency per packet, causing DDS
# heartbeat timeouts and remote viewer STALL.
log "Disabling WiFi power management..."
WIFI_IFACE=$(iw dev 2>/dev/null | awk '/Interface/{print $2; exit}') || true
if [ -n "${WIFI_IFACE:-}" ]; then
    sudo iw dev "$WIFI_IFACE" set power_save off > /dev/null 2>&1 && \
        log "WiFi power-save off on $WIFI_IFACE." || \
        log "Could not set power-save on $WIFI_IFACE (may need root or different interface)."
else
    log "No WiFi interface found — skipping."
fi

# ── 5. Ollama CPU priority ────────────────────────────────────────────────────
# LLM inference can saturate all cores for several seconds, starving camera
# driver threads and delaying DDS heartbeats.  nice +10 gives camera/DDS
# threads priority when the scheduler has to choose.
log "Adjusting Ollama process priority..."
OLLAMA_PID=$(pgrep -x ollama 2>/dev/null | head -1) || true
if [ -n "${OLLAMA_PID:-}" ]; then
    sudo renice -n 10 -p "$OLLAMA_PID" > /dev/null 2>&1 && \
        log "Ollama (PID $OLLAMA_PID) reniced to +10." || \
        log "Could not renice Ollama."
else
    log "Ollama not running — will apply on next start if called from wrapper."
fi

# ── 6. IRQ affinity hint ──────────────────────────────────────────────────────
# Move USB IRQs to CPU 0, leave CPUs 1+ for ROS/Ollama so USB DMA doesn't
# compete with inference threads.  Best-effort — fails silently on systems
# without xhci IRQs exposed in /proc/irq.
log "Hinting USB IRQs to CPU 0..."
for irq_dir in /proc/irq/*/; do
    action_file="${irq_dir}actions"
    if [ -f "$action_file" ] && grep -q "xhci\|ehci\|ohci" "$action_file" 2>/dev/null; then
        irq_num=$(basename "$irq_dir")
        echo 1 | sudo tee "/proc/irq/${irq_num}/smp_affinity" > /dev/null 2>&1 || true
    fi
done
log "USB IRQ affinity set (best-effort)."

log "All tuning complete."
