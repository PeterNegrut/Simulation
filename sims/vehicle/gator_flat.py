"""Gator on flat rigid terrain with a scripted path-follower driver.

Pattern copied from the installed PyChrono 9.0.1 demos (source of truth):
  demos/vehicle/demo_VEH_Gator.py              — vehicle + RigidTerrain setup
  demos/vehicle/demo_VEH_SteeringController.py — ChPathFollowerDriver (PID
                                                 steering + cruise control)

No URDF parsing: the Gator is a built-in pychrono.vehicle model (NSC contact,
TMEASY tires, shafts brakes). The driver follows either an ISO double lane
change or a straight line at a target speed — no keyboard input needed.

Run windowed:  conda run -n pychrono-sim python sims/vehicle/gator_flat.py
Run headless:  conda run -n pychrono-sim python sims/vehicle/gator_flat.py --headless
Options:       --path dlc|straight   --speed 8.0   --tend 20
Exit code 0/1 = PASS/FAIL from the end-of-run report.
"""

import argparse
import math
import sys

import pychrono as chrono
import pychrono.vehicle as veh

veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

STEP_SIZE = 1e-3          # physics at 1 kHz
RENDER_FPS = 50.0         # render decoupled from physics
SAMPLE_PERIOD = 0.1       # diagnostics sampling
PROGRESS_PERIOD = 2.0     # progress print cadence

INIT_LOC = chrono.ChVector3d(-120.0, 0.0, 0.4)
PATCH_LENGTH = 300.0      # x in [-150, 150]
PATCH_WIDTH = 50.0        # y in [-25, 25]


def quat_to_rpy(q):
    """ZYX yaw-pitch-roll from a ChQuaterniond, in radians."""
    w, x, y, z = q.e0, q.e1, q.e2, q.e3
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def build_path(kind):
    if kind == "dlc":
        # Parameterized ISO double lane change (to left), values from
        # demo_VEH_SteeringController.py
        return veh.DoubleLaneChangePath(INIT_LOC, 13.5, 4.0, 11.0, 50.0, True)
    start = chrono.ChVector3d(INIT_LOC.x, INIT_LOC.y, 0.5)
    end = chrono.ChVector3d(PATCH_LENGTH / 2 - 10.0, INIT_LOC.y, 0.5)  # stay on the patch
    return veh.StraightLinePath(start, end, 1)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--headless", action="store_true", help="no window; diagnostics only")
    ap.add_argument("--path", choices=("dlc", "straight"), default="dlc")
    ap.add_argument("--speed", type=float, default=8.0, help="target speed m/s")
    ap.add_argument("--tend", type=float, default=20.0, help="simulation end time s")
    args = ap.parse_args()

    # --- Vehicle (pattern: demo_VEH_Gator.py) ---
    gator = veh.Gator()
    gator.SetContactMethod(chrono.ChContactMethod_NSC)
    gator.SetChassisFixed(False)
    gator.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, chrono.QUNIT))
    gator.SetBrakeType(veh.BrakeType_SHAFTS)
    gator.SetTireType(veh.TireModelType_TMEASY)
    gator.SetTireStepSize(STEP_SIZE)
    gator.SetInitFwdVel(0.0)
    gator.Initialize()

    # NOTE: in this PyChrono 9.0.1 build VisualizationType_* lives in
    # pychrono core, not pychrono.vehicle (the shipped demos say veh.*)
    vt = chrono.VisualizationType_NONE if args.headless else chrono.VisualizationType_PRIMITIVES
    gator.SetChassisVisualizationType(chrono.VisualizationType_MESH if not args.headless else vt)
    gator.SetSuspensionVisualizationType(vt)
    gator.SetSteeringVisualizationType(vt)
    gator.SetWheelVisualizationType(chrono.VisualizationType_NONE)
    gator.SetTireVisualizationType(chrono.VisualizationType_MESH if not args.headless else vt)

    system = gator.GetSystem()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    print(f"Vehicle mass: {gator.GetVehicle().GetMass():.1f} kg")
    print(f"Driveline:    {gator.GetVehicle().GetDriveline().GetTemplateName()}")
    print(f"Tire:         {gator.GetVehicle().GetTire(1, veh.LEFT).GetTemplateName()}")

    # --- Terrain (pattern: demo_VEH_Gator.py) ---
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, PATCH_LENGTH, PATCH_WIDTH)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    terrain.Initialize()

    # --- Scripted driver (pattern: demo_VEH_SteeringController.py) ---
    path = build_path(args.path)
    # Past the end of an open path the steering target freezes and the vehicle
    # circles back toward it — end the run just before the last path point.
    path_end_x = path.GetPoint(path.GetNumPoints() - 1).x - 2.0
    driver = veh.ChPathFollowerDriver(gator.GetVehicle(), path, "path", args.speed)
    driver.GetSteeringController().SetLookAheadDistance(5)
    driver.GetSteeringController().SetGains(0.8, 0, 0)
    driver.GetSpeedController().SetGains(0.4, 0, 0)
    driver.Initialize()

    # --- Optional visualization ---
    vis = None
    if not args.headless:
        import pychrono.irrlicht  # noqa: F401  (needed for the Irrlicht runtime)
        vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
        vis.SetWindowTitle("Gator path follower — flat terrain")
        vis.SetWindowSize(1280, 800)
        vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
        vis.Initialize()
        vis.AddLightDirectional()
        vis.AddSkyBox()
        vis.AttachVehicle(gator.GetVehicle())
        gator.GetVehicle().EnableRealtime(True)

    # --- Diagnostics accumulators ---
    p0 = gator.GetVehicle().GetPos()
    start_pos = chrono.ChVector3d(p0.x, p0.y, p0.z)  # copy: live-reference gotcha
    max_speed = 0.0
    final_speed = 0.0
    roll_max = pitch_max = 0.0
    yaw_min = yaw_max = 0.0
    z_min, z_max = start_pos.z, start_pos.z
    contacts_min, contacts_max = None, 0
    nan_found = False
    reached_end = False
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
            final_speed = speed
            max_speed = max(max_speed, speed)
            roll_max = max(roll_max, abs(roll))
            pitch_max = max(pitch_max, abs(pitch))
            yaw_min = min(yaw_min, yaw)
            yaw_max = max(yaw_max, yaw)
            z_min = min(z_min, p.z)
            z_max = max(z_max, p.z)
            nc = system.GetContactContainer().GetNumContacts()
            contacts_min = nc if contacts_min is None else min(contacts_min, nc)
            contacts_max = max(contacts_max, nc)
            if p.x >= path_end_x:
                reached_end = True
                print(f"[{time:6.2f}s] reached path end (x={p.x:.2f}) — stopping")
                break
            if time >= next_progress:
                next_progress += PROGRESS_PERIOD
                print(f"[{time:6.2f}s] x={p.x:8.2f}  y={p.y:6.2f}  z={p.z:5.2f}  "
                      f"v={speed:5.2f} m/s  yaw={math.degrees(yaw):6.1f} deg")

    # --- Final report ---
    p = gator.GetVehicle().GetPos()
    end_pos = chrono.ChVector3d(p.x, p.y, p.z)
    disp = (end_pos - start_pos).Length()
    lateral = end_pos.y - start_pos.y

    # If we stopped early at the path end, expect the path length instead of
    # target_speed * tend
    disp_expected = (path_end_x - start_pos.x - 5.0) if reached_end \
        else 0.5 * args.speed * args.tend
    checks = [
        ("no NaN", not nan_found),
        (f"displacement {disp:.1f} m >= {disp_expected:.1f} m", disp >= disp_expected),
        (f"max speed {max_speed:.2f} >= {0.7 * args.speed:.2f} m/s",
         max_speed >= 0.7 * args.speed),
        (f"|roll| max {math.degrees(roll_max):.1f} deg < 15", math.degrees(roll_max) < 15.0),
        (f"|pitch| max {math.degrees(pitch_max):.1f} deg < 15", math.degrees(pitch_max) < 15.0),
        (f"chassis z in [{z_min:.2f}, {z_max:.2f}] within [0.1, 1.5]",
         0.1 <= z_min and z_max <= 1.5),
    ]
    passed = all(ok for _, ok in checks)

    print("\n===== FINAL REPORT (gator_flat) =====")
    print(f"path: {args.path}   target speed: {args.speed} m/s   t_end: {args.tend} s"
          f"   reached path end: {reached_end}")
    print(f"start:  ({start_pos.x:.2f}, {start_pos.y:.2f}, {start_pos.z:.2f})")
    print(f"end:    ({end_pos.x:.2f}, {end_pos.y:.2f}, {end_pos.z:.2f})")
    print(f"displacement: {disp:.2f} m   lateral offset: {lateral:+.2f} m")
    print(f"speed: final {final_speed:.2f} m/s, max {max_speed:.2f} m/s")
    print(f"attitude: |roll| max {math.degrees(roll_max):.2f} deg, "
          f"|pitch| max {math.degrees(pitch_max):.2f} deg, "
          f"yaw range [{math.degrees(yaw_min):.1f}, {math.degrees(yaw_max):.1f}] deg")
    print(f"contacts: [{contacts_min}, {contacts_max}] "
          "(TMEASY tire forces bypass the collision system; 0 is normal)")
    for label, ok in checks:
        print(f"  {'PASS' if ok else 'FAIL'}  {label}")
    print(f"RESULT: {'PASS' if passed else 'FAIL'}")
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
