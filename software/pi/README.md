# Pi runtime snapshot (live deployed state)

This directory is a **record of what is actually running on the Raspberry Pi** (`armpi` / 192.168.1.142 as of the pull date below).

It is the "as-deployed" / "as-running" view, not the development sources.

## When captured
- Date: 2026-06-08 (from Pi)
- Host: armpi (user armpi)
- Pulled from: armpi@192.168.1.142
- Method: rsync of the runtime locations that custom deploys target (`~/ros_nodes`, `~/vision`) plus the ROS packages in `~/moveo_ws/src` and selected helper scripts from `~`.

## Layout
- `ros_nodes/` — flat Python runtime for the arm (the files actually executed on the Pi for the controller + kinematics).
  - Includes `kinematics.py`, `moveo_publisher.py` (and its .bak), `foxglove_ee_to_joint_states.py`, `manual_joint_controller_node.py`, etc.
- `vision/` — all vision / approach / depth / hand-eye / goto nodes + run scripts + calibration files as they live on the Pi (`~/vision`).
- `moveo_ws/src/` — the three custom ROS 2 packages that were present on the Pi (`moveo_description`, `moveo_moveit_config`, `moveo_serial_bridge`). Build/install/log garbage has been pruned from this snapshot for repo sanity; meshes + URDF + source are kept as they were on the Pi.
- `home-scripts/` — Pi-side copies of various helper scripts that live directly in `~` (flash_*, reconnect_*, run_moveo_publisher.sh, setup_*, esp control scripts, etc.). These are often variants or the "production" versions of scripts also present in the Mac dev tree.

## Purpose
- Traceability: see exactly what code + configs + URDFs + meshes were on the hardware when something was tested or when a bug was observed.
- Diffing: compare against the "source" trees (`software/ros/`, `software/vision/pi/`, `software/mac-gui/`, root `urdf/`) to detect drift.
- Recovery / audit: if the Pi gets wiped or we want to reproduce a known-good (or known-bad) state.
- The recent kinematics unification work (single user frame, no more chain/user conversions) happened after this snapshot in the dev tree.

## How to refresh this snapshot later
From the repo root on a machine that can reach the Pi:

```bash
PI=armpi@192.168.1.142   # or armpi@armpi.local if mDNS works
TARGET=software/pi

mkdir -p $TARGET/ros_nodes $TARGET/vision $TARGET/moveo_ws/src $TARGET/home-scripts

rsync -avz --delete --exclude='__pycache__' --exclude='*.pyc' \
  $PI:~/ros_nodes/ $TARGET/ros_nodes/

rsync -avz --delete --exclude='__pycache__' --exclude='*.pyc' \
  $PI:~/vision/ $TARGET/vision/

rsync -avz --delete \
  $PI:~/moveo_ws/src/ $TARGET/moveo_ws/src/

# (optional) clean the snapshot after pull
find $TARGET/moveo_ws -type d \( -name build -o -name install -o -name log \) -exec rm -rf {} +
find $TARGET -type d -name __pycache__ -exec rm -rf {} +
find $TARGET -name '*.pyc' -delete

# loose home scripts
rsync -avz $PI:~/run_moveo_publisher.sh $PI:~/connect_esp.sh \
  $PI:~/reconnect_esp.sh $PI:~/flash*.sh $PI:~/setup*.sh \
  $PI:~/esp32_control.sh $TARGET/home-scripts/
```

Then `git add software/pi && git commit -m "chore: update Pi runtime snapshot from armpi (date)"`.

## Notes
- This snapshot includes the state of `kinematics.py` and `moveo_publisher.py` *before* the June 2026 single-frame axis unification was pushed.
- The root of this repo also contains development copies and the "source of truth" versions (see `software/mac-gui/kinematics.py` as the canonical one, `software/ros/moveo_publisher.py`, `software/vision/pi/deploy_to_pi.sh`, and the various `run_*.sh` launchers).
- Large binary assets (meshes) are present because they were part of the running `moveo_description` package on the Pi.
- For the absolute latest Pi state during active work, re-run the pull above and commit the diff.

This folder is intentionally tracked in the repo (unlike `.grok/` harness state).
