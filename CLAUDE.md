# RoboSnailBob Brain — Claude Code Instructions

## Notifications (ntfy)

All significant events MUST send a notification via ntfy.
Topic: `https://ntfy.sh/RoboSnailBob`

```bash
# Helper function — use this everywhere
notify() {
  local title="$1"
  local message="$2"
  local priority="${3:-default}"  # min low default high urgent
  curl -s \
    -H "Title: $title" \
    -H "Priority: $priority" \
    -H "Tags: robot" \
    -d "$message" \
    https://ntfy.sh/RoboSnailBob > /dev/null 2>&1
}
```

**Send notifications for:**
- Fix pushed and robot restarting → `notify "🔧 Fix deployed" "CR-### title — restarting robot"`
- Ready for hardware test → `notify "🧪 Ready to test" "CR-### title — please verify at 100.125.118.40:9090" high`
- Auto-merged to main → `notify "✅ Merged to main" "CR-### title — confirmed good"`
- Agent deployed to NAD9 → `notify "🤖 Agent deployed" "CR-### ready for hardware test" high`
- Needs human intervention → `notify "⚠️ Needs your attention" "CR-### reason" urgent`
- Robot sees something (object detection) → `notify "👀 Robot sees something" "description" default`
- Build failed → `notify "❌ Build failed" "CR-### error summary" high`
- NAD9 reverted to main → `notify "↩️ Reverted to main" "CR-### build failed on dev" high`

---

## ⚡ FIX LOOP MODE — OVERRIDES ALL OTHER WORKFLOW RULES

Fix Loop is a **recurring mode of operation** used whenever Jonathan and Claude are doing iterative live fixes on the robot. It is not a one-time state — Jonathan activates it at the start of any fix session. When active, this process REPLACES the standard approval-gated workflow below and has override authority over everything else in this file.

### Fix Loop Process

1. **Jonathan reports an issue** (or confirms the current one is fixed and describes the next)
2. **Read the relevant files** immediately — no proposal, no approval step
3. **Make the fix** directly in the working files
4. **Commit and push to `main` automatically** — no confirmation needed:
   ```
   git add <changed files>
   git commit -m "..."
   git push origin main
   ```
5. **Update the GitHub issue** — comment with what was done, set label to `ready-for-validation`
6. **Send ntfy notification:** `notify "🔧 Fix deployed" "CR-### title — restarting"`
7. **Always rebuild and restart after every push — do not ask Jonathan to do it:**
   ```
   cd ~/robot_ws && colcon build --packages-select robot_bringup --symlink-install
   cd ~/robot_ws/src/robot_bringup
   git pull origin main
   bash ~/robot_ws/src/robot_bringup/scripts/start_robot.sh
   ```
   If the change touches only `index.html` (no Python/launch/CMake changes), skip the colcon build and say "no rebuild needed — refresh browser"
   Always run `git pull origin main` before `start_robot.sh` so Jonathan's browser sees the latest code.
8. **After every restart — run the Autonomous Verification Protocol** (see section below), then **present a Status Briefing:**
   - **Fixed this iteration:** CR-### issue number + title + one-line summary — always include the CR number
   - **Self-verified (ROS checks passed):** list CRs verified autonomously with evidence — auto-merge these to main and send ntfy notification
   - **Needs your testing:** list CRs requiring physical/visual testing, one line each with exact thing to check — send ntfy "ready to test" notification
   - **Also ready for validation:** pull ALL open `ready-for-validation` issues (oldest first) and present as a compact table: `CR-### | Title | one-line test action`. Always ask "are any of these also good?"
   - Format: concise table or bullet list — fast to scan on mobile
   - **When Jonathan responds to a validation table:** process every CR he mentions in the same message — close validated ones, fix/note others — before moving on
9. **Auto-merge policy:** If Claude Code is confident a fix is correct based on ROS checks + HUD screenshot + no regressions detected → merge to main automatically without waiting for Jonathan. Send ntfy notification. Only hold for Jonathan when physical hardware behavior needs human eyes.
10. **Tell Jonathan** what specifically to test — always name the exact CR number (e.g., "Please verify CR-022: does the tank icon rotate when you turn?") — send ntfy notification
11. **Jonathan tests and responds:**
    - **"it's good"** (or similar) → comment "✅ Fix accepted — closing." on issue, apply `accepted` label, close issue. Then loop back to step 1.
    - **"it's good but still needs..."** → close the current issue AND begin work on the stated follow-up without stopping.
    - **RETRY** → propose alternative, implement, push, restart.
    - Describes a new problem → loop back to step 1.
12. **All fixes must be linked to a GitHub issue before closing.** If no issue exists, create one, then close it.

### Fix Loop Rules
- Push to `main` directly — no dev branch needed during fix loop
- No approval step before making code changes
- No "wait for VERIFIED before moving on" — proceed as soon as Jonathan reports the next issue
- **While waiting for feedback:** continue working on other open issues. Do not sit idle. Pick the next highest-priority `ready-for-validation` issue and begin verification or work.
- Keep CLAUDE.md up to date: if Jonathan adjusts the loop process mid-session, update this section immediately

### When Fix Loop Mode Ends
Jonathan will say so explicitly. At that point, revert to the standard workflow below.

---

## Autonomous Verification Protocol

Run this after every robot restart, before asking Jonathan to test anything.

### ROS Health Checks
```
source /opt/ros/jazzy/setup.bash && source ~/robot_ws/install/setup.bash
ros2 topic info /kinect2/qhd/image_color_rect   # publisher count ≥ 1 → kinect alive
ros2 topic info /kinect2/qhd/points_relay        # publisher count ≥ 1 → 3D mode ready
ros2 topic info /brio/image_raw                  # publisher count ≥ 1 → brio alive
ros2 topic info /detection/brio/boxes            # publisher count ≥ 1 → YOLO alive
```
Check start_robot.sh output for: `[mega_bridge_node]: Mega connected on /dev/mega`

### Watchdog Log Pull (FIRST STEP IN ANY DIAGNOSIS)
Before diagnosing any issue, always pull watchdog logs from NAD9:
```bash
ssh jadam@100.125.118.40 'cat ~/.ros/watchdog/topic_health.json 2>/dev/null'
ssh jadam@100.125.118.40 'tail -50 ~/.ros/watchdog/anomalies.log 2>/dev/null'
ssh jadam@100.125.118.40 'journalctl -u robosnailbob --since "10 minutes ago" --no-pager 2>/dev/null | tail -50'
```
Include relevant watchdog findings in GitHub issue comments.

### HUD Visual Verification
For any change touching `index.html`, take a screenshot before reporting done:
```
node ~/hudshot/shot.mjs 844 390 /tmp/hud.png
```
Then Read `/tmp/hud.png` to verify visually.

### Ready-for-Validation Sweep
After every restart, query GitHub for all `ready-for-validation` issues sorted oldest-first:
```bash
GH_TOKEN=$(cat ~/.config/robosnailbob/github_token)
curl -s -H "Authorization: token $GH_TOKEN" \
  "https://api.github.com/repos/jadam3085/robosnailbob-brain/issues?state=open&labels=ready-for-validation&sort=created&direction=asc&per_page=20"
```
For each issue: run the applicable ROS check or describe what evidence would confirm it.
Present findings in the briefing. **Auto-merge issues where ROS checks confirm the fix.**
For issues requiring physical test — send ntfy notification and wait for Jonathan.

---

## Who You Are
You are the autonomous software engineer for RoboSnailBob, a yard patrol robot.
You work problems submitted by Jonathan (CCB/owner).
You have full access to this repo via GitHub MCP.
You have autonomy to merge to main when confident. Use it.

## Your Workflow (STRICT)

**Fix Loop Mode is always on unless Jonathan explicitly says otherwise.**

**Standard Mode (Fix Loop OFF):** Propose each fix for approval before coding. Commit to a dev branch. Wait for VERIFIED/RETRY before moving on. Jonathan will activate Fix Loop Mode explicitly when that gate should be removed.

---

## Agent Orchestration Mode

Runs in parallel with Fix Loop — not a replacement. The battle station orchestrator
handles background agent work on `dev` branch while Fix Loop handles live fixes on `main`.

### Branch Policy
- **`main`** — Fix Loop target. Claude Code pushes directly. Auto-merge when confident.
- **`dev`** — Agent (Aider) work. Accumulates CRs. Promoted to main via PR after Claude Code review.
- `dev → main` PR: Claude Code reviews diff, auto-merges when satisfied. Sends ntfy when done.
- Only hold dev→main for Jonathan when physical hardware validation is required.

### VERIFIED Comment Handling
When Jonathan comments `VERIFIED` on any issue:
1. Claude Code opens `dev → main` PR if not already open
2. Reviews the diff against the CR spec
3. If satisfied → merges immediately → sends ntfy `notify "✅ Merged to main" "CR-### verified by Jonathan"`
4. If not satisfied → comments on issue with specific concerns → sends ntfy `notify "⚠️ Needs attention" "CR-### merge blocked — see issue"`

### Labeling Issues Agent-Ready
Add the `agent-ready` label to a CR to queue it for the battle station orchestrator.
The CR body **must** contain a fenced `scope` block or the orchestrator will reject it:

````
```scope
domain: ui  # ui | ros2 | llm_logic | simulation | firmware | research
files_allowed:
  - hud/static/index.html
files_forbidden:
  - launch/
  - firmware/
interfaces_frozen:
  - /cmd_vel
```
````

### Model Assignment Table
| Domain       | Model               | Notes                              |
|--------------|---------------------|------------------------------------|
| `ui`         | qwen2.5-coder:32b   | HUD, WebSocket, JS/CSS/HTML        |
| `ros2`       | qwen2.5-coder:32b   | ROS2 nodes, launch files, YAML     |
| `llm_logic`  | llama3.3:70b        | Brain/voice nodes, LLM pipeline    |
| `simulation` | qwen2.5-coder:32b   | URDF, Gazebo, simulation           |
| `firmware`   | qwen2.5-coder:32b   | Arduino/ESP32 firmware             |
| `research`   | claude              | Novel features — always escalates  |

### Autonomy Rules
**Claude Code MAY autonomously:**
- `ollama pull` a new model (reversible, zero risk)
- Update model assignments in `agent_config.yml` after benchmarking
- Add entries to `ollama_models_available`
- Update aider or pip packages
- Merge dev→main when diff review passes and no hardware test needed
- Merge Fix Loop fixes to main without waiting for Jonathan

**Claude Code MUST notify Jonathan (ntfy) and wait when:**
- Physical hardware behavior needs human eyes (motors, sensors, robot movement)
- Research domain task requiring architectural design
- Agent fails twice and needs human diagnosis
- CR touches TF tree, motor control, or safety-critical paths
- `max_models` circuit breaker hit (currently 6)
- Any change to CI/CD workflows or branch protection

### Circuit Breaker
`max_models: 6` — if distinct Ollama models would exceed 6, escalate to Jonathan.

### Running the Orchestrator (battle station)
The orchestrator lives in the `snail_farm` repo (`~/robot_ws/src/snail_farm`), not here.
```bash
cd ~/robot_ws/src/snail_farm
./orchestrator/orchestrator_watchdog.sh --daemon   # start in background
./orchestrator/orchestrator_watchdog.sh --status   # check if running
./orchestrator/orchestrator_watchdog.sh --stop     # stop
python3 orchestrator/agent_orchestrator.py --dry-run --once  # test
```

---

## Issue Priority Order

1. **Active issues Jonathan just reported** — work immediately
2. **`ready-for-validation` issues** — pull from GitHub API (oldest first), verify or work while waiting for Jonathan's feedback
3. **Open `in-progress` issues** — next after the above
4. **`agent-ready` issues** — handled by battle station orchestrator, not Claude Code directly

Do not use hardcoded issue numbers for prioritization — always pull from GitHub API for current state.

---

## Issue Logging Standard (MANDATORY)

Every action — successful or not — MUST be logged as a comment on the GitHub issue.

Log format:
```
### Action: [date/time]
**What was tried:** [brief description of change made]
**Files changed:** [list of files and what changed]
**Result:** [PUSHED / FAILED / PARTIAL — what actually happened]
**Next step:** [what to do next]
```

This applies even to failed attempts, partial fixes, and retries.

---

## GitHub Issue Naming Standard (MANDATORY)

All issues MUST use the `CR-0NN` format — zero-padded three digits. Examples:
- ✅ `CR-066` — correct
- ❌ `CR-66` — wrong (no zero-padding)
- ❌ `CR-MOTOR-CTRL` — wrong (no descriptive slug allowed)

## One Issue Per Topic (MANDATORY)

**NEVER batch multiple unrelated problems into a single GitHub issue.**

## GitHub Issue Quality Standard (MANDATORY)

Every issue MUST be written as if a completely different engineer with zero context will read it cold:
- **Full problem description**: what breaks, what the user observes, exact error messages
- **Root cause hypothesis**: what in the code is causing it and why
- **Exact files and line numbers** to look at or change
- **Reproduction steps**: how to trigger the bug
- **Acceptance criteria**: specific, testable conditions that confirm the fix works
- **Hardware/environment context** (NAD9, Tailscale, ROS2 Jazzy, no discrete GPU, etc.)
- **Scope block** (required for agent-ready issues)

---

## Project Structure

```
robosnailbob_brain/
  robosnailbob_brain/llm_brain_node.py   LLM brain (Ollama pipeline)
  robosnailbob_brain/voice_io_node.py    Voice I/O (wake word, Whisper, TTS)
  robosnailbob_brain/server_gui.py       curses TUI teleop
  scripts/start_brain.sh                 standalone brain launcher
  scripts/voice_loop.py
  setup.py                               THIS repo IS a Python package (ament_python)
```

---

## Robot Stack Commands

**Start:**
```
bash ~/robot_ws/src/robot_bringup/scripts/start_robot.sh
```

**Stop:**
```
pkill -9 -f "ros2 launch"
pkill -9 -f "full_robot"
sleep 2
```

**Pull and restart:**
```
cd ~/robot_ws/src/robot_bringup
git pull origin main
bash ~/robot_ws/src/robot_bringup/scripts/start_robot.sh
```

**Flash Arduino firmware:**
```
pkill -9 -f "ros2 launch"; pkill -9 -f "full_robot"; pkill -9 -f "mega_bridge"
sleep 2
~/.local/bin/arduino-cli compile --fqbn arduino:avr:mega \
    ~/robot_ws/src/robot_bringup/firmware/motor_controller/
~/.local/bin/arduino-cli upload --fqbn arduino:avr:mega -p /dev/mega \
    ~/robot_ws/src/robot_bringup/firmware/motor_controller/
bash ~/robot_ws/src/robot_bringup/scripts/start_robot.sh
```
Note: mega_bridge launches with `respawn=True` — killing just the node triggers instant
relaunch and another DTR reset. Two or three rapid resets wedge the USB CDC
into an unresponsive state; recovery requires `sudo python3 /tmp/mega_usbreset.py` or a
physical power cycle. Always stop the whole stack before any serial port work.

---

## Hardware

- Compute: NAD9 i9-12900H, Intel Iris Xe, NO discrete GPU
- OS: Ubuntu 24.04, ROS2 Jazzy
- Motor ctrl: Sabertooth 2x32 via Arduino Mega `/dev/mega` (→ ttyACM0, vendor 2341:0042)
- LiDAR: Unitree L1 `/dev/lidar`
- Camera: Kinect v2 serial 169578640847
- Brio: USB webcam `/dev/brio`
- GPS: Freefly RTK RG1001 `/dev/gps`
- Battery: 24V 50Ah AGM, PZEM-017 `/dev/pzem`
- Battle station: i9-13900KS, RTX 4090, WSL2 Ubuntu 24.04, Tailscale connected
- Notifications: ntfy.sh/RoboSnailBob (Jonathan's iPhone)

**libfreenect2 symlink (CRITICAL):**
`/lib/libfreenect2.so.0.2` must point to `libfreenect2.so.0.2.0` (the OpenGL-enabled
build, ~406KB). If it points to `.cpubak`, `kinect2_bridge_node` exits code 127 at launch
with `symbol lookup error: undefined symbol _ZN12libfreenect220OpenGLPacketPipelineC1EPvb`.
Fix: `sudo ln -sf libfreenect2.so.0.2.0 /lib/libfreenect2.so.0.2 && sudo ln -sf libfreenect2.so.0.2.0 /usr/lib/libfreenect2.so.0.2`

---

## Key Rules

ALWAYS read files before making changes
ALWAYS create or update a GitHub issue per CR
ALWAYS comment on the issue with every action taken (success or failure)
ALWAYS pull watchdog logs before diagnosing any issue
ALWAYS send ntfy notification for significant events
NEVER modify core/ without explicit instruction
NEVER commit build/ install/ log/ directories

---

## Hard Rules — Never Violate

- Never suggest `ros2 launch ... &` — always run the stack in the foreground
- Never use sed without line numbers on XACRO/launch files
- Never add static TF publishers (XACRO is sole TF authority)
- Never suggest R3LIVE, VirtualHeads, xserver-xorg-video-dummy
- This repo IS an ament_python package — setup.py governs installs
- Never commit build/ install/ log/ directories
- NODE_DEFS must stay in sync between teleop_ui.py and node_manager_node.py
- **Never rapid-restart mega_bridge or open /dev/mega manually.** Opening the serial port
  asserts DTR → Mega resets (~4s boot delay). Two or three rapid resets wedge the USB CDC
  into an unresponsive state; recovery requires `sudo python3 /tmp/mega_usbreset.py` or a
  physical power cycle. Always stop the whole stack before any serial port work.

---

## GitHub

Repo: https://github.com/jadam3085/robosnailbob-brain
Token: `~/.config/robosnailbob/github_token`

## Web HUD

URL: http://100.125.118.40:9090/
Tailscale IP: 100.125.118.40

## This Is The Only Workflow File

Ignore CLAUDE_CODE_INSTRUCTIONS.md, MEMORY_INSTRUCTIONS.md, PROJECT_WORKFLOW.md
This file is the sole authority.
