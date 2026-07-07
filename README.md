# pychrono-sim

Multibody/multi-physics simulations built on [PyChrono](https://projectchrono.org/pychrono/).

PyChrono is a precompiled binary package, so it lives in its own conda environment
(`pychrono-sim`) rather than in this repo or a venv. This repo only tracks the
environment spec and simulation scripts.

## Setup

```
conda create -n pychrono-sim --file environment.lock.txt
conda install -n pychrono-sim -c conda-forge matplotlib ipykernel
conda activate pychrono-sim
```

`environment.lock.txt` is an explicit package list (exact URLs, no solving needed) —
it's the only reliable way to recreate this env. `environment.yml` documents the
intent but conda's solver can't resolve it from scratch: the `pychrono` build that
includes Irrlicht/VSG visualization (from the `bochengzou` channel) pins
`glfw>=3.5.0`, a build conda-forge's current index doesn't surface, plus real CUDA
runtime deps (Chrono's GPU module). The plain conda-forge `pychrono` package solves
fine but ships without any visualization module (no 3D window).

## Verify the install

```
python sims/hello_chrono.py
```

Should print the falling box's height at each checkpoint and end with
`PyChrono is working.`

## Layout

- `environment.yml` — human-readable dependency summary (see note above; not solvable from scratch)
- `environment.lock.txt` — explicit package spec that actually recreates the env (`conda create -n <env> --file environment.lock.txt`)
- `sims/` — simulation scripts; each is a standalone, runnable script (`python sims/<name>.py`)
- `sims/assets/` — mesh files used as visual shapes (e.g. `cube.obj`)
- `output/` — generated simulation data/renders (gitignored)

## Sims

- `hello_chrono.py` — headless smoke test: a box falling under gravity, no visualization (fast install check)
- `pendulum.py` — a bob on a rigid rod (a real welded body, not just a visual asset) swinging from a fixed pivot, striking a cube obstacle (`sims/assets/cube.obj`, exported from Blender) mid-swing and bouncing off it
- `bouncing_ball.py` — a sphere with restitution bouncing on a floor, rendered live in an Irrlicht window (requires the Bullet collision system, enabled via `SetCollisionSystemType`)
- `r2d2.py` — parses the R2D2 URDF bundled with PyChrono itself (`robot/r2d2/r2d2.urdf` under the conda env's Chrono data dir, loaded via `chrono.GetChronoDataFile`, no repo assets needed) and drives it forward on a linear motor, wheels spinning in sync, until it stops short of the floor edge
- `robosimian.py` — RoboSimian actually walking on rigid terrain, using Chrono's own compiled model and driver (`pychrono.robot.RoboSimian` + `RS_Driver`), reproducing Chrono's `demo_ROBOT_RoboSimian_Rigid.py`. The robot assumes its initial pose (chassis fixed, no terrain yet), settles briefly once terrain is created under its feet, then the bundled `actuation/walking_cycle.txt` drives it. Verified headlessly over 15s: no NaN/exploding velocities, chassis height stays tightly bounded (doesn't hover or sink), wheels stay below the chassis and resting on the terrain, contacts are continuously present, and the chassis makes real net forward progress with the oscillating advance-then-plateau pattern of an actual stepping gait — not random drift. (`GetChassisRot().GetAxisZ()` reads as "upside down" throughout even though the robot is genuinely right-side up; that axis is just this model's local body-frame convention, not a reliable upright indicator — wheel-height-vs-chassis-height is what actually confirms orientation.)
- `robosimian_urdf_diagnostic.py` — **diagnostic tool, not a terrain simulator.** A raw `ChParserURDF` import of the same robot's URDF (`robot/robosimian/rs.urdf`, 43 bodies / 32 revolute joints). Useful for inspecting body/joint names, geometry, and import correctness, but a URDF alone gives Chrono the body/joint connectivity and nothing else — no tuned collision model, no contact materials, no driver that knows how to make the thing walk. Confirmed by testing: even with a real stance applied smoothly and collision properly enabled (both need fixing manually — see the file's docstring for exactly what's wrong and why), it settles into a crouch and drifts rather than standing or walking cleanly. Has three selectable modes (`MODE = "A"|"B"|"C"` at the top of the file) for assembly/collision/stance checks.

Visualized sims open an interactive window (`vis.Run()` loop) — close the window to end the script.
