# pychrono-sim

PyChrono robotics/physics simulations. Windows 11, conda env `pychrono-sim`
(PyChrono **9.0.1**, Python 3.12). Recreate the env from `environment.lock.txt`,
not `environment.yml` (the yml is intent-only; the solver can't resolve it).

Run any sim: `conda run -n pychrono-sim python sims/robot/robosimian.py`
(or activate the env first). Env vars `RS_MODE` / `RS_TERRAIN` select gait and
terrain in the robosimian scripts.

## Layout

- `sims/simple_sims/` — pendulum, bouncing ball, etc. (`star_pendulum.py` uses
  `assets/star.obj`; concave mesh collision wedges the bob — convex hull is used
  for collision, real mesh for visuals)
- `sims/robot/` — the staged RoboSimian terrain pipeline (see below)
- `sims/urdf/` — URDF import diagnostics; `robosimian_urdf_diagnostic.py` proves
  why raw URDF import can't walk (no driver, no tuned collision)
- `sims/assets/` — .obj meshes (Blender exports; star/cube are flat cutouts in
  the X-Z plane extruded along Y — swap Y/Z to stand them upright)

## RoboSimian staged terrain pipeline

One file per stage; each stage copies the previous file and changes ONLY terrain.
Earlier files are never modified — they are the known-good regression baselines.

1. `robosimian.py` — flat ground, 4 gaits (walk/drive/scull/inchworm) — PASSED
2. `robosimian_ramp.py` — 5° ramp, drive mode — PASSED
3. `robosimian_obstacle.py` — low obstacle (bump/step), walk mode — PASSED
4. (next) repeated low bumps → random boxes → Blender visual mesh w/ simple collision

Every stage is validated **headlessly before windowed runs**: monkeypatch
`chronoirr.ChVisualSystemIrrlicht` with a no-op FakeVis class in a scratchpad
runner, `runpy.run_path` the sim, and read its end-of-run report (displacement,
speed, pitch range + max pitch rate, wheel clearance vs LOCAL terrain height,
contact count range, NaN check). Trust those numbers, not the render.

## Measured physics (drive/walk on flat, mu=0.8, SMC + Bullet + BB solver, dt=1e-3)

- touching wheel clearance = **+0.118 m** (wheel radius); +0.119 on a 5° slope
- drive: 0.048 m/s. walk: 19.16 s/gait-cycle, ~0.2 m/cycle (~0.010 m/s) —
  judging walk mode needs ≥1 full cycle or it looks broken
- **drive mode cannot climb ANY obstacle** — sharp steps (0.08/0.04/0.02 m) and
  smooth bumps (0.03/0.02 m crown) all stall at first contact, identically at
  mu=0.8 and mu=1.5. The constraint is the drive controller/wheel-torque path,
  not friction. Drive = flat ground only; obstacles are crossed with the walking
  gait (9 cm foot lift), matching the real JPL robot's usage.
- stall signature: displacement freezes at the wheel-vs-face contact geometry
  (x_edge − sqrt(r² − (r−h)²)), avg speed decays ~1/t, pitch stays flat

## PyChrono 9.0.1 API gotchas (all hit and verified this project)

- `pychrono.robot` has **no `ChRobotActuation`** — that's the Chrono ≥9.1 rename
  of `RS_Driver` (same class). Compat shim:
  `DriverClass = getattr(robosimian, "ChRobotActuation", None) or robosimian.RS_Driver`
- Drive mode needs BOTH the `driving_*` actuation files AND
  `driver.SetDrivingMode(True)`
- `RoboSimian.SetVisualizationType*` setters are **uncallable** (SWIG exposes the
  methods but not the enum they need). Use
  `vis.EnableCollisionShapeDrawing(True)` + `vis.EnableContactDrawing(...)` instead
- **`GetChassisPos()` returns a live reference** — copy components
  (`chrono.ChVector3d(p.x, p.y, p.z)`) before storing a start position, or every
  displacement computes to exactly zero
- `ChTriangleMeshConnected.GetBoundingBox()` raises TypeError in this build —
  compute bounds from `GetCoordsVertices()` instead
- Chrono 9 primitive cylinders run along **local Z** — rotate 90° about X to lay
  one across a path
- `chrono.GetChronoDataFile("")` on an empty string: don't — pass `""` through
  untouched for absent actuation files
- RoboSimian chassis local +Z points at the **belly**; use the rotated local −Z
  as "up" for tilt/pitch. `GetAxisZ()` reading −1 does NOT mean upside down
- Chassis stays fixed during the pose phase (t < 1.0 s); terrain is created under
  the settled feet (`GetWheelPos(FR).z − 0.15`), then the chassis is released.
  Verify the floor extends under the WHOLE planned path — a too-short floor made
  the robot drive off the edge and fall 1200 m (caught by the clearance log)

## Conventions

- Wheel IDs: FR=0, RR=1, RL=2, FL=3 (`robosimian.FR` etc.)
- Render at ~60 fps (`step % render_steps == 0`), physics at 1 kHz — never 1:1
- Chase camera: `vis.SetCameraPosition(p + offset)` / `vis.SetCameraTarget(p)`
  each render frame
- Long headless runs go through PowerShell background jobs writing UTF-8 output
  files (`| Out-File -Encoding utf8`; plain `>` produces UTF-16)
