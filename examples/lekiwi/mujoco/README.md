# LeKiwi MuJoCo simulation

A minimal MuJoCo scene for the LeKiwi mobile manipulator (SO101 arm + 3-omniwheel base).

## Files

- `lekiwi.xml` — the MuJoCo scene. Mounts the SO101 arm on a planar mobile base.
- `simulate.py` — interactive viewer: drive the base and move the arm joints.
- `../../../SO101/so101.mjcf` — the arm model (a body fragment), converted from
  `../../../SO101/so101_new_calib.urdf` (see below).

`lekiwi.xml`, the SO101 URDF and its STL meshes are excluded by `.gitignore`
(`*.xml`, `*.urdf`, `*.stl`), so they only exist on this machine. A fresh clone
has to bring its own copy before the scene can load.

## Run

```bash
python examples/lekiwi/mujoco/simulate.py
```

Controls:

| Key          | Action                        |
| ------------ | ----------------------------- |
| W / S        | drive forward / backward      |
| A / D        | drive left / right            |
| Z / X        | rotate left / right           |
| 1 .. 6       | select an arm joint           |
| Up / Down    | move the selected joint       |
| R            | reset base + arm to home      |
| Q / Esc      | quit                          |

## Notes on the model

- The base is approximated as a planar body (`slide x` + `slide y` + `hinge yaw`)
  driven by velocity actuators, matching the real LeKiwi action interface
  (`arm_*.pos` + `x.vel / y.vel / theta.vel`). The 3 wheels are visual only.
- Arm joints carry `armature="0.01"` to represent the servo gearbox rotor inertia;
  without it the very light SO101 links make PD position control unstable.
- The arm runs on `position` actuators (`kp=30`, `kv=2`); the base on `velocity`
  actuators (`kv=100` for x/y, `kv=5` for yaw — translation and rotation have
  different inertia).

## Regenerating the arm MJCF from the URDF

`so101.mjcf` is generated from the SO101 URDF. To regenerate it:

```python
import re

import mujoco

m = mujoco.MjModel.from_xml_path("SO101/so101_new_calib.urdf")
mujoco.mj_saveLastXML("/tmp/so101_full.xml", m)
s = open("/tmp/so101_full.xml").read()
wb = re.search(r"<worldbody>.*?</worldbody>", s, re.S).group(0)
inner = wb[len("<worldbody>") : -len("</worldbody>")].strip()
inner = re.sub(r'<joint name="([^"]+)"', r'<joint name="\1" armature="0.01"', inner)
open("SO101/so101.mjcf", "w").write(
    f'<body name="so101_arm" pos="0 0 0" quat="1 0 0 0">\n{inner}\n</body>\n'
)
```
