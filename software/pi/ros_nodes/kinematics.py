#!/usr/bin/env python3
"""
kinematics.py — Pure forward/inverse kinematics for the Moveo arm (no ROS dependencies).

This module is the SINGLE SOURCE OF TRUTH for Moveo 5-DOF arm link lengths
and the ikpy Chain definition used by both FK (GUI "Compute FK" button,
local checks) and IK (Pi-side cartesian solve_ik, reachability, collision).

Update ONLY here when you re-measure the physical arm.

Then copy the length constants + Chain(...) block into
software/ros/moveo_publisher.py (the Pi runtime version runs standalone).

Current measured values (user-provided):
    BASE=0.20, WAIST=0.140, UPPER=0.22, FORE=0.11, WRIST=0.11, EE=0.06
"""

try:
    import numpy as np
    from ikpy.chain import Chain
    from ikpy.link import OriginLink, URDFLink
    _IKPY_AVAILABLE = True
except ImportError:
    _IKPY_AVAILABLE = False
    print("[IK] ikpy not installed — FK and cartesian disabled. Run: pip3 install ikpy", flush=True)

# ── Moveo kinematic chain (SINGLE SOURCE OF TRUTH) ─────────────────────────────
# Link lengths in METRES. Measured values provided by user.
# Structure matches the segmented model (waist -> shoulder pitch -> elbow ->
# wrist roll -> wrist pitch -> EE) so that IK warm-starts, reach checks,
# and FK are identical on GUI and Pi.
L_BASE  = 0.20   # base plate to J1 (waist) rotation axis
L_WAIST = 0.140  # riser J1→J2 (shoulder)
L_UPPER = 0.22   # shoulder (J2) → elbow (J3)
L_FORE  = 0.11   # elbow (J3) → wrist-roll (J4)
L_WRIST = 0.11   # wrist-roll (J4) → wrist-pitch (J5)
L_EE    = 0.06   # wrist-pitch (J5) → EE tip

# Back-compat aliases (some solve_ik / check code still references the _L_ names)
_L_BASE  = L_BASE
_L_WAIST = L_WAIST
_L_UPPER = L_UPPER
_L_FORE  = L_FORE
_L_WRIST = L_WRIST
_L_EE    = L_EE

if _IKPY_AVAILABLE:
    MOVEO_CHAIN = Chain(
        name="moveo",
        active_links_mask=[False, True, True, True, True, True, False],
        links=[
            OriginLink(),
            # J1: waist — rotates about Z (yaw, swings arm left/right in X-Y plane)
            URDFLink("j1", origin_translation=[0, 0, L_BASE],  origin_orientation=[0,0,0], rotation=[0,0,1], bounds=(-2.00, 2.40)),
            # J2: shoulder — rotates about X (pitch, swings upper arm in Y-Z plane)
            #   At home (j2=0) upper arm points straight up along +Z
            URDFLink("j2", origin_translation=[0, 0, L_WAIST], origin_orientation=[0,0,0], rotation=[1,0,0], bounds=(-1.95, 1.95)),
            # J3: elbow — upper arm length along Z; rotates about X
            URDFLink("j3", origin_translation=[0, 0, L_UPPER], origin_orientation=[0,0,0], rotation=[1,0,0], bounds=(-2.20, 2.20)),
            # J4: wrist roll — forearm length along Z; rotates about Z (roll)
            URDFLink("j4", origin_translation=[0, 0, L_FORE],  origin_orientation=[0,0,0], rotation=[0,0,1], bounds=(-3.14, 3.14)),
            # J5: wrist pitch — rotates about X. WRIST segment before the pitch pivot.
            URDFLink("j5", origin_translation=[0, 0, L_WRIST], origin_orientation=[0,0,0], rotation=[1,0,0], bounds=(-1.75, 1.75)),
            # End effector (passive) — extends along previous Z
            URDFLink("ee", origin_translation=[0, 0, L_EE],    origin_orientation=[0,0,0], rotation=[0,0,0], bounds=(0, 0)),
        ],
    )
else:
    MOVEO_CHAIN = None

# Max reachable (approx) from the shoulder pivot. Used for early rejection.
_MAX_REACH = L_UPPER + L_FORE + L_WRIST + L_EE
MAX_REACH = _MAX_REACH  # public alias for importers


def forward_kinematics(joints):
    """Return (x, y, z) in metres for the end-effector given [j1..j5] in radians.

    The returned coordinates are in the *user command frame*:
        +X = forward, +Y = left, +Z = up  (REP-103 style, matching what you
        pass to cartesian commands and see in the controller "target x=..." logs).

    All higher-level code (position estimation, targets for IK, GUI display, etc.)
    should stay in this user frame.

    The internal ikpy MOVEO_CHAIN uses a 90° rotated convention (its primary
    reach plane is along +Y in chain coords when j1=0). The wrapper below
    (and the one inside solve_ik) handles the conversion so callers don't have to.
    """
    if not _IKPY_AVAILABLE or MOVEO_CHAIN is None:
        raise RuntimeError("ikpy is not installed")
    full = [0.0] + list(joints) + [0.0]
    fk = MOVEO_CHAIN.forward_kinematics(full)
    cx, cy, cz = fk[0, 3], fk[1, 3], fk[2, 3]
    return chain_to_user((cx, cy, cz))


def chain_to_user(p):
    """Convert point from internal ikpy chain frame → user frame."""
    cx, cy, cz = p
    # The IK side does: user(x,y) -> internal(-y, x)
    # This makes positive user-Y (left) map to the chain's positive reach direction
    # consistently with the j1 positive yaw direction.
    # Inverse of that:
    return cy, -cx, cz


def user_to_chain(p):
    """Convert point from user frame → internal ikpy chain frame."""
    x, y, z = p
    return -y, x, z


def forward_kinematics_matrix(joints):
    """Return the 4x4 homogeneous EE transform matrix given [j1..j5] radians.

    The *translation* part of this matrix is expressed in the internal chain frame.
    Use this (via mp.forward_kinematics_matrix) when transforming points that live
    in the model frame (camera offsets defined in URDF/chain coords, depth points
    from reprojectImageTo3D in left camera frame, etc.). After the 4x4 transform,
    convert the resulting position vector with chain_to_user if you need it in
    the user frame for targets/logs/etc.
    """
    if not _IKPY_AVAILABLE or MOVEO_CHAIN is None:
        raise RuntimeError("ikpy is not installed")
    full = [0.0] + list(joints) + [0.0]
    return MOVEO_CHAIN.forward_kinematics(full)
