# CLAUDE.md — Robotrol v7 Project Rules

## Critical: Robot Profile Protection

**NEVER modify `post_transform` in robot profile JSONs (EB300.json, EB15_red.json, Moveo.json) without explicit user approval.**

The `post_transform` section contains calibrated kinematic parameters that are essential for correct FK/IK:
- `sim_theta_offset_deg` — joint angle offsets (e.g. EB300: A:-90, X:-90, Z:-90)
- `sim_theta_scale` — joint angle scaling factors
- `mirror_x` — coordinate frame mirroring

Removing or changing these values silently breaks ALL kinematics (FK, IK, TCP sequences, gamepad control) with hard-to-diagnose 90-degree offsets.

### Mandatory FK verification after ANY profile change
After modifying a robot profile JSON, always verify FK at MPos=0:
```bash
python -c "
import json
from tcp_pose_module_v3 import set_dh_model_from_dict, fk6_forward_mm
with open('PROFILE.json') as f: p = json.load(f)
set_dh_model_from_dict(p['dh_model'])
from tcp_pose_module_v3 import GEOM_DH as G
X,Y,Z,R,P,Yaw,T = fk6_forward_mm(G, {'A':0,'X':0,'Y':0,'Z':0,'B':0,'C':0})
print('TCP=(%.1f, %.1f, %.1f) RPY=(%.1f, %.1f, %.1f)' % (X,Y,Z,R,P,Yaw))
"
```
Expected values at MPos=0:
- **EB300**: TCP=(179, 0, 860), Tool axis = +X
- **Moveo**: TCP=(-0, 0, 900), Tool axis = +Z (with mirror_x)

If FK output deviates significantly, the change is WRONG — revert immediately.

## Expected EB300 post_transform (reference)
```json
"post_transform": {
    "mirror_x": false,
    "sim_theta_offset_deg": {"A": -90.0, "X": -90.0, "Z": -90.0},
    "sim_theta_scale": {}
}
```

## Project Structure

- **Main entry**: `Robotrol_FluidNC_v7_3.py` (title "Robotrol V7.3")
- **Kinematics**: `tcp_world_kinematics_frame.py` (IK solver, TCP sequences)
- **FK/TCP panel**: `tcp_pose_module_v3.py` (forward kinematics, DH model loading)
- **Gamepad**: `gamepad_block_v3.py` (jogging modes 0/1/2)
- **Pick & Place**: `pickplace_ui.py` + `autocalib_v1.py`
- **Profiles**: `EB300.json`, `EB15_red.json`, `Moveo.json`
- **DH fallback**: `model/dh.json` (loaded at import, overridden by profile)

## Development Rules

1. **Syntax check after every edit**: `python -m py_compile <file>.py`
2. **Start command**: `start pythonw Robotrol_FluidNC_v7_3.py`
3. **All UI text in English**
4. **No auto-commit** — always ask before git operations
5. **Keep GUI compact** — mind existing window sizes
6. **Python path**: `/c/Users/Tobia/AppData/Local/Programs/Python/Python313/python`
7. **Profiles without endstops** (EB300, EB15_red): Homing ($H) is blocked by safety lock

## DH Model Architecture

The DH model flows through the system as follows:
1. `model/dh.json` loaded at module import time (fallback/default)
2. Profile JSON (`EB300.json` etc.) loaded at startup → `set_dh_model_from_dict()`
3. `tcp_pose_module_v3` stores global `_DH_ROWS`, `_POST_TRANSFORM`, `GEOM_DH`
4. All FK/IK calls use these globals — no separate copies
5. `apply_joint_angle_post_transform()` applies `sim_theta_offset_deg` + `sim_theta_scale`

**The `post_transform` is NOT optional** — it maps machine angles to DH model angles.
