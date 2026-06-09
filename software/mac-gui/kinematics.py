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
            # J1: waist yaw about +Z. Positive (RH rule, Z up) is CCW viewed from above.
            # At j1=0 the arm bend plane is aligned with +X_model; +j1 yaws the
            # reach direction toward +Y_model.
            URDFLink("j1", origin_translation=[0, 0, L_BASE],  origin_orientation=[0,0,0], rotation=[0,0,1], bounds=(-2.00, 2.40)),
            # J2: shoulder pitch about +Y. At home (j2=0) upper arm points straight up +Z.
            # Positive rotation (RH about +Y) bends the upper arm forward toward +X_model.
            URDFLink("j2", origin_translation=[0, 0, L_WAIST], origin_orientation=[0,0,0], rotation=[0,1,0], bounds=(-1.95, 1.95)),
            # J3: elbow pitch about +Y. Positive bends the forearm toward +X_model (in the plane set by j1).
            URDFLink("j3", origin_translation=[0, 0, L_UPPER], origin_orientation=[0,0,0], rotation=[0,1,0], bounds=(-2.20, 2.20)),
            # J4: wrist roll about local Z (forearm twist). Positive follows RH with local Z.
            URDFLink("j4", origin_translation=[0, 0, L_FORE],  origin_orientation=[0,0,0], rotation=[0,0,1], bounds=(-3.14, 3.14)),
            # J5: wrist pitch about +Y. Positive bends the tip toward +X_model in the local forearm frame.
            URDFLink("j5", origin_translation=[0, 0, L_WRIST], origin_orientation=[0,0,0], rotation=[0,1,0], bounds=(-1.75, 1.75)),
            # End effector (passive) — extends along previous Z at zero pose.
            URDFLink("ee", origin_translation=[0, 0, L_EE],    origin_orientation=[0,0,0], rotation=[0,0,0], bounds=(0, 0)),
        ],
    )
else:
    MOVEO_CHAIN = None

# Max reachable (approx) from the shoulder pivot. Used for early rejection.
_MAX_REACH = L_UPPER + L_FORE + L_WRIST + L_EE
MAX_REACH = _MAX_REACH  # public alias for importers


def chain_to_user(p):
    """Convert from internal ikpy *model* frame to final public user frame.

    The user frame is the model rotated 180° about Z:
        user = (-x_model, -y_model, z)
    This is the single coordinate system used by all higher-level code
    (GUI targets, IK, vision, logs, Compute FK, etc.).
    Matches physical description:
      - positive J2/J3/J5 bend toward +X in model → -X in user
      - positive J1 yaws toward +Y in model → -Y in user
    """
    x, y, z = p
    return -float(x), -float(y), float(z)


def user_to_chain(p):
    """Convert from final public user frame to internal ikpy *model* frame.

    (180° Z is self-inverse.)
    """
    x, y, z = p
    return -float(x), -float(y), float(z)


def forward_kinematics(joints):
    """Return (x, y, z) in metres for the end-effector given [j1..j5] in radians.

    Public coordinates are in the final *user frame* (REP-103 +X forward,
    +Y left, +Z up), which is the internal ikpy model frame rotated 180° about Z.

    In the *internal model* (how the Chain/joints are defined):
        positive J2/J3/J5 (J1=J4=0) bend toward +X_model
        positive J1 yaws CCW toward +Y_model
    After the 180° Z rotation the public user values are (-x_model, -y_model, z).

    All callers (GUI cartesian/IK targets, vision estimated/approaching, Compute FK,
    logs, depth points after T @ CAM_T) use/see the final user frame.
    """
    if not _IKPY_AVAILABLE or MOVEO_CHAIN is None:
        raise RuntimeError("ikpy is not installed")
    full = [0.0] + list(joints) + [0.0]
    fk = MOVEO_CHAIN.forward_kinematics(full)
    return chain_to_user((float(fk[0, 3]), float(fk[1, 3]), float(fk[2, 3])))


def forward_kinematics_matrix(joints):
    """Return the 4x4 homogeneous EE transform matrix given [j1..j5] radians.

    The matrix (translation + orientation) is in the *internal model frame*
    (the frame the ikpy Chain and joint axes are defined in).
    Use this for transforming points defined relative to the EE in the model
    (e.g. CAM_T, depth rays in camera->EE), then convert the resulting base point
    with chain_to_user() to get final user-frame coordinates.
    The public forward_kinematics() already returns user-frame points (model rotated 180° about Z).
    """
    if not _IKPY_AVAILABLE or MOVEO_CHAIN is None:
        raise RuntimeError("ikpy is not installed")
    full = [0.0] + list(joints) + [0.0]
    return MOVEO_CHAIN.forward_kinematics(full)
