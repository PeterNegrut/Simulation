"""Gator lapping the Blender racetrack (mesh-trainer/blender/gator-track.blend).

Terrain is the exported collision mesh `sims/assets/gator_track_collision.obj`
loaded as a RigidTerrain mesh patch; `gator_track_visual.obj` (track + cones/
rocks/tirewalls/trees, all visual-only) is attached as the display mesh when
windowed. Both OBJs are exported from Blender at 3x SCALE with Z-up preserved
(up_axis='Z', forward_axis='Y'):

  - at scale 1 the track is 20x10 m and its turns are ~2.5 m radius; the
    Gator's minimum turning radius is ~6 m (2.78 m wheelbase, 25 deg steer),
    so the loop is geometrically undrivable. At 3x: 60x30 m, 7.5 m turns.
  - track layout (scaled coords): flat road at z~0, a raised bump ridge along
    y=0 for x in [-13.5, 15] (the rock infield), curbs along the x edges, and
    ~0.9 m berms at the four corners. Drivable corridor: |y| in [2, 12.6].

The driver is the same ChPathFollowerDriver as gator_flat.py, on a CLOSED
ChBezierCurve stadium loop around the infield: straights at y=+-7.5, semi-
circle turns of radius 7.5 centered at (+-16.5, 0). Laps are counted at the
x=START_X line on the south straight.

Run windowed:  conda run -n pychrono-sim python sims/vehicle/gator_track.py
Run headless:  conda run -n pychrono-sim python sims/vehicle/gator_track.py --headless
Options:       --speed 5.0   --tend 60   --laps 2
Exit code 0/1 = PASS/FAIL from the end-of-run report.
"""

import argparse
import math
import os
import sys

import pychrono as chrono
import pychrono.vehicle as veh

veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

ASSETS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "assets")
COLLISION_OBJ = os.path.normpath(os.path.join(ASSETS, "gator_track_collision.obj"))
VISUAL_OBJ = os.path.normpath(os.path.join(ASSETS, "gator_track_visual.obj"))

STEP_SIZE = 1e-3          # physics at 1 kHz
RENDER_FPS = 50.0         # render decoupled from physics
SAMPLE_PERIOD = 0.1       # diagnostics sampling
PROGRESS_PERIOD = 2.0     # progress print cadence

# Stadium centerline (scaled coords): straights y=+-7.5, turn radius 7.5
STRAIGHT_HALF = 16.5      # turn centers at (+-16.5, 0)
LANE_R = 7.5
PATH_Z = 0.4
START_X = -12.0           # start pose / lap line on the south straight
INIT_LOC = chrono.ChVector3d(START_X, -LANE_R, 0.4)


def quat_to_rpy(q):
    """ZYX yaw-pitch-roll from a ChQuaterniond, in radians."""
    w, x, y, z = q.e0, q.e1, q.e2, q.e3
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def stadium_waypoints():
    """CCW loop: south straight -> east turn -> north straight -> west turn."""
    wp = []
    n_str, n_arc = 7, 12
    # south straight, heading +x
    for i in range(n_str):
        x = -STRAIGHT_HALF + i * (2 * STRAIGHT_HALF) / n_str
        wp.append((x, -LANE_R))
    # east turn: around (STRAIGHT_HALF, 0), angle -90deg -> +90deg
    for i in range(n_arc):
        a = -0.5 * math.pi + i * math.pi / n_arc
        wp.append((STRAIGHT_HALF + LANE_R * math.cos(a), LANE_R * math.sin(a)))
    # north straight, heading -x
    for i in range(n_str):
        x = STRAIGHT_HALF - i * (2 * STRAIGHT_HALF) / n_str
        wp.append((x, LANE_R))
    # west turn: around (-STRAIGHT_HALF, 0), angle +90deg -> +270deg
    for i in range(n_arc):
        a = 0.5 * math.pi + i * math.pi / n_arc
        wp.append((-STRAIGHT_HALF + LANE_R * math.cos(a), LANE_R * math.sin(a)))
    return wp


def build_loop_path():
    pts = chrono.vector_ChVector3d()
    for x, y in stadium_waypoints():
        pts.append(chrono.ChVector3d(x, y, PATH_Z))
    return chrono.ChBezierCurve(pts, True)  # closed


def cross_track_error(x, y):
    """Distance from the stadium centerline (analytic, not the bezier)."""
    if x > STRAIGHT_HALF:
        return abs(math.hypot(x - STRAIGHT_HALF, y) - LANE_R)
    if x < -STRAIGHT_HALF:
        return abs(math.hypot(x + STRAIGHT_HALF, y) - LANE_R)
    return abs(abs(y) - LANE_R)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--headless", action="store_true", help="no window; diagnostics only")
    ap.add_argument("--speed", type=float, default=5.0, help="target speed m/s")
    ap.add_argument("--tend", type=float, default=60.0, help="simulation end time s")
    ap.add_argument("--laps", type=int, default=2, help="laps required to PASS")
    args = ap.parse_args()

    for f in (COLLISION_OBJ, VISUAL_OBJ):
        if not os.path.isfile(f):
            print(f"missing mesh: {f} (re-export from gator-track.blend)")
            return 1

    # --- Vehicle (same pattern as gator_flat.py) ---
    gator = veh.Gator()
    gator.SetContactMethod(chrono.ChContactMethod_NSC)
    gator.SetChassisFixed(False)
    gator.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, chrono.QUNIT))
    gator.SetBrakeType(veh.BrakeType_SHAFTS)
    gator.SetTireType(veh.TireModelType_TMEASY)
    gator.SetTireStepSize(STEP_SIZE)
    gator.SetInitFwdVel(0.0)
    gator.Initialize()

    # VisualizationType_* is in pychrono core in this build, not pychrono.vehicle
    vt = chrono.VisualizationType_NONE if args.headless else chrono.VisualizationType_PRIMITIVES
    gator.SetChassisVisualizationType(chrono.VisualizationType_MESH if not args.headless else vt)
    gator.SetSuspensionVisualizationType(vt)
    gator.SetSteeringVisualizationType(vt)
    gator.SetWheelVisualizationType(chrono.VisualizationType_NONE)
    gator.SetTireVisualizationType(chrono.VisualizationType_MESH if not args.headless else vt)

    system = gator.GetSystem()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # --- Terrain: track collision mesh as a RigidTerrain mesh patch ---
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, COLLISION_OBJ,
                             True,      # connected mesh for collision
                             0.01,      # sweep sphere radius
                             False)     # visuals come from VISUAL_OBJ instead
    terrain.Initialize()

    ground = patch.GetGroundBody()
    h0 = terrain.GetHeight(chrono.ChVector3d(INIT_LOC.x, INIT_LOC.y, 2.0))
    print(f"terrain: mesh patch, ground body fixed={ground.IsFixed()}, "
          f"height at start=({h0:.3f})")

    # --- Scripted driver on the closed stadium loop ---
    path = build_loop_path()
    driver = veh.ChPathFollowerDriver(gator.GetVehicle(), path, "loop", args.speed)
    driver.GetSteeringController().SetLookAheadDistance(5)
    driver.GetSteeringController().SetGains(0.8, 0, 0)
    driver.GetSpeedController().SetGains(0.4, 0, 0)
    driver.Initialize()

    # --- Optional visualization ---
    vis = None
    if not args.headless:
        import pychrono.irrlicht  # noqa: F401
        vmesh = chrono.ChTriangleMeshConnected().CreateFromWavefrontFile(
            VISUAL_OBJ, True, True)
        vshape = chrono.ChVisualShapeTriangleMesh()
        vshape.SetMesh(vmesh)  # keep vmesh referenced: live-view gotcha
        vshape.SetMutable(False)
        ground.AddVisualShape(vshape)

        line = chrono.ChVisualShapeLine()
        line.SetLineGeometry(chrono.ChLineBezier(path))
        line.SetNumRenderPoints(400)
        ground.AddVisualShape(line)

        vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
        vis.SetWindowTitle("Gator track loop")
        vis.SetWindowSize(1280, 800)
        vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 8.0, 0.8)
        vis.Initialize()
        vis.AddLightDirectional()
        vis.AddSkyBox()
        vis.AttachVehicle(gator.GetVehicle())
        gator.GetVehicle().EnableRealtime(True)

    # --- Diagnostics accumulators ---
    p0 = gator.GetVehicle().GetPos()
    start_pos = chrono.ChVector3d(p0.x, p0.y, p0.z)  # copy: live-reference gotcha
    max_speed = 0.0
    roll_max = pitch_max = 0.0
    z_min, z_max = start_pos.z, start_pos.z
    xte_max = 0.0
    dist = 0.0
    prev_x, prev_y = start_pos.x, start_pos.y
    contacts_min, contacts_max = None, 0
    nan_found = False
    laps = 0
    lap_times = []
    last_lap_t = 0.0
    next_sample = 0.0
    next_progress = 0.0
    render_steps = max(1, int(round(1.0 / (RENDER_FPS * STEP_SIZE))))
    step = 0

    while True:
        time = system.GetChTime()
        if time >= args.tend:
            break
        if vis is not None and not vis.Run():
            break

        if vis is not None and step % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()

        driver.Synchronize(time)
        terrain.Synchronize(time)
        gator.Synchronize(time, driver_inputs, terrain)
        if vis is not None:
            vis.Synchronize(time, driver_inputs)

        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        gator.Advance(STEP_SIZE)
        if vis is not None:
            vis.Advance(STEP_SIZE)
        step += 1

        if time >= next_sample:
            next_sample += SAMPLE_PERIOD
            p = gator.GetVehicle().GetPos()
            speed = gator.GetVehicle().GetSpeed()
            roll, pitch, yaw = quat_to_rpy(gator.GetVehicle().GetRot())
            if any(math.isnan(v) for v in (p.x, p.y, p.z, speed, roll, pitch, yaw)):
                nan_found = True
                print(f"[{time:6.2f}s] NaN detected — aborting")
                break
            max_speed = max(max_speed, speed)
            roll_max = max(roll_max, abs(roll))
            pitch_max = max(pitch_max, abs(pitch))
            z_min = min(z_min, p.z)
            z_max = max(z_max, p.z)
            xte = cross_track_error(p.x, p.y)
            xte_max = max(xte_max, xte)
            dist += math.hypot(p.x - prev_x, p.y - prev_y)
            # lap line: crossing x=START_X eastbound on the south straight.
            # The run starts ON the line, so every crossing completes a lap
            # (lap 1 includes the standing start).
            if prev_x < START_X <= p.x and p.y < -2.0 and time - last_lap_t > 5.0:
                laps += 1
                lap_times.append(time - last_lap_t)
                print(f"[{time:6.2f}s] LAP {laps} complete "
                      f"({lap_times[-1]:.2f} s)")
                last_lap_t = time
            prev_x, prev_y = p.x, p.y
            nc = system.GetContactContainer().GetNumContacts()
            contacts_min = nc if contacts_min is None else min(contacts_min, nc)
            contacts_max = max(contacts_max, nc)
            if time >= next_progress:
                next_progress += PROGRESS_PERIOD
                print(f"[{time:6.2f}s] x={p.x:7.2f}  y={p.y:6.2f}  z={p.z:5.2f}  "
                      f"v={speed:5.2f} m/s  yaw={math.degrees(yaw):7.1f}  "
                      f"xte={xte:4.2f} m")

    # --- Final report ---
    p = gator.GetVehicle().GetPos()
    end_pos = chrono.ChVector3d(p.x, p.y, p.z)

    checks = [
        ("no NaN", not nan_found),
        (f"laps {laps} >= {args.laps}", laps >= args.laps),
        (f"max cross-track {xte_max:.2f} m < 3.5 m", xte_max < 3.5),
        (f"|roll| max {math.degrees(roll_max):.1f} deg < 20", math.degrees(roll_max) < 20.0),
        (f"|pitch| max {math.degrees(pitch_max):.1f} deg < 20", math.degrees(pitch_max) < 20.0),
        (f"chassis z in [{z_min:.2f}, {z_max:.2f}] within [0.1, 0.9]",
         0.1 <= z_min and z_max <= 0.9),
    ]
    passed = all(ok for _, ok in checks)

    print("\n===== FINAL REPORT (gator_track) =====")
    print(f"target speed: {args.speed} m/s   t_end: {args.tend} s   "
          f"required laps: {args.laps}")
    print(f"end: ({end_pos.x:.2f}, {end_pos.y:.2f}, {end_pos.z:.2f})   "
          f"distance driven: {dist:.1f} m")
    print(f"laps: {laps}" +
          (f"   lap times: {', '.join(f'{t:.2f} s' for t in lap_times)}"
           if lap_times else ""))
    print(f"speed max: {max_speed:.2f} m/s   cross-track max: {xte_max:.2f} m")
    print(f"attitude: |roll| max {math.degrees(roll_max):.2f} deg, "
          f"|pitch| max {math.degrees(pitch_max):.2f} deg")
    print(f"contacts: [{contacts_min}, {contacts_max}] "
          "(TMEASY tire forces bypass the collision system; 0 is normal)")
    for label, ok in checks:
        print(f"  {'PASS' if ok else 'FAIL'}  {label}")
    print(f"RESULT: {'PASS' if passed else 'FAIL'}")
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
