"""HMMWV gauntlet: full Chrono vehicle stack on the Blender obstacle course.

Vehicle pattern from the installed demos (source of truth):
  demos/vehicle/demo_VEH_HMMWV.py — HMMWV_Full, SMC, SHAFTS engine,
  AUTOMATIC_SHAFTS transmission, AWD, Pitman arm, chassis collision NONE
No URDFs, no sensors. Terrain is a veh.RigidTerrain MESH PATCH built from the
collision-only OBJ (never the full visual OBJ): tire forces need real
terrain height/normal queries, not a decorative ChBody.

Course (exported from mesh-trainer/blender/hmmwv-course.blend at scale 1,
Z-up preserved): 64 x 16 m, west-to-east, x=[-32,+32], y=[-8,+8].
Zones (measured from the collision mesh, west to east):
  start_flat  x<-28    | moguls     -28..-17  0.35 m staggered bumps
  trench      -17..-12 | 0.55 m deep, crossed diagonally (30 deg)
  off_camber  -12..-4  | 12 deg side slope
  hill_ledge  -4..+8   | 31 deg grade, 0.38 m ledge mid-face, crest z=2.30
  boulders    +8..+15.5| 0.25-0.45 m embedded humps
  mud_basin   +15.5..22| 0.5 m fording depression
  finish_flat x>+22

Expected regression behavior (headless, MEASURED):
  --throttle 0.20               -> crosses moguls/trench/camber, stalls at the
       hill ledge (x=-2.8), rolls back, brakes to a stop  => EXPECTED_STALL
  --throttle 0.30 (default)     -> carries ~20 km/h into the ledge and rears
       over backward (pitch -84 deg)                      => EXPECTED_FLIP
  --throttle 0.50               -> hits the ledge at 30 km/h and flips
       (roll -59, pitch -86)                              => EXPECTED_FLIP
  --throttle 0.30 --tire rigid  -> slows to 1.8 m/s, CLIMBS the ledge,
       finishes the course in 16 s                        => PASS_COURSE
The TMEASY/RIGID split on the ledge is the force-element artifact this file's
--tire flag exists to expose: TMEASY reads the near-vertical ledge face as
sudden contact-patch penetration and launches; RIGID tires resolve real
Bullet contacts against the mesh and climb it. Rollover or stall in any zone
other than hill_ledge is reported as FAIL_* (regression).
PASS_COURSE only if it reaches x >= +30 with no NaN and no rollover.
Exit code 0 = PASS_COURSE / EXPECTED_STALL / EXPECTED_FLIP; 1 = anything else.

Run windowed (W/S throttle/brake, A/D steer, chase cam; --throttle ignored):
  conda run -n pychrono-sim python sims/vehicle/simulate_hmmwv.py
Watch the scripted run in the window (--throttle drives, keys don't):
  conda run -n pychrono-sim python sims/vehicle/simulate_hmmwv.py --scripted
  --throttle 0.35 --tire rigid
Run headless scripted:
  conda run -n pychrono-sim python sims/vehicle/simulate_hmmwv.py --headless
  [--throttle 0.30] [--duration 60] [--print-dt 2.0] [--tire tmeasy|rigid]
"""

import argparse
import math
import os
import sys

import pychrono as chrono
import pychrono.vehicle as veh

veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

ASSETS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "assets")
COLLISION_OBJ = os.path.normpath(os.path.join(ASSETS, "hmmwv_course_collision.obj"))
VISUAL_OBJ = os.path.normpath(os.path.join(ASSETS, "hmmwv_course_visual.obj"))

STEP_SIZE = 1e-3
TIRE_STEP_SIZE = STEP_SIZE
RENDER_FPS = 50.0
SAMPLE_PERIOD = 0.1

# Tire model kept as a constant (with --tire override) so RIGID / RIGID_MESH
# can be compared if the force-element (TMEASY) tires behave oddly on mesh
# ledges. TMEASY queries terrain height under the contact patch, so a sharp
# ledge face reads as sudden penetration -> force spike; RIGID tires use real
# Bullet collision against the mesh.
TIRE_MODELS = {
    "tmeasy": veh.TireModelType_TMEASY,
    "rigid": veh.TireModelType_RIGID,
    "rigid_mesh": veh.TireModelType_RIGID_MESH,
}
TIRE_MODEL_DEFAULT = "tmeasy"   # parity with demo_VEH_HMMWV.py

INIT_LOC = chrono.ChVector3d(-30.0, 0.0, 1.6)   # matches demo drop-in height

SETTLE_TIME = 0.5        # zero inputs while suspension settles
RAMP_TIME = 1.0          # throttle 0 -> target over this window
STALL_WINDOW = 4.0       # no new forward progress for this long => stall
STALL_PROGRESS = 0.2     # "new progress" means x_peak advanced by this much
ROLLOVER_DEG = 75.0
FINISH_X = 30.0

ZONES = [
    (-28.0, "start_flat"),
    (-17.0, "moguls"),
    (-12.0, "trench"),
    (-4.0, "off_camber"),
    (8.0, "hill_ledge"),
    (15.5, "boulders"),
    (22.0, "mud_basin"),
    (float("inf"), "finish_flat"),
]


def zone_of(x):
    for xmax, name in ZONES:
        if x < xmax:
            return name
    return "finish_flat"


def quat_to_rpy(q):
    """ZYX yaw-pitch-roll from a ChQuaterniond, in radians."""
    w, x, y, z = q.e0, q.e1, q.e2, q.e3
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--headless", action="store_true",
                    help="scripted driver, no Irrlicht")
    ap.add_argument("--throttle", type=float, default=0.30,
                    help="headless throttle after settle/ramp (0..1)")
    ap.add_argument("--duration", type=float, default=60.0,
                    help="max simulation time s")
    ap.add_argument("--print-dt", type=float, default=2.0,
                    help="diagnostics print period s")
    ap.add_argument("--tire", choices=sorted(TIRE_MODELS), default=TIRE_MODEL_DEFAULT,
                    help="tire model (compare rigid vs tmeasy on mesh ledges)")
    ap.add_argument("--scripted", action="store_true",
                    help="drive with the scripted --throttle even in windowed "
                         "mode (watch the autonomous run instead of W/S/A/D)")
    args = ap.parse_args()
    interactive = not args.headless and not args.scripted

    if not os.path.isfile(COLLISION_OBJ):
        print(f"missing mesh: {COLLISION_OBJ} (export from hmmwv-course.blend)")
        return 1

    # --- Vehicle (demo_VEH_HMMWV.py pattern) ---
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, chrono.QUNIT))
    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
    hmmwv.SetTireType(TIRE_MODELS[args.tire])
    hmmwv.SetTireStepSize(TIRE_STEP_SIZE)
    hmmwv.Initialize()

    # VisualizationType_* lives in pychrono core in this build
    vt = chrono.VisualizationType_NONE if args.headless else chrono.VisualizationType_PRIMITIVES
    mesh_vt = chrono.VisualizationType_NONE if args.headless else chrono.VisualizationType_MESH
    hmmwv.SetChassisVisualizationType(mesh_vt)
    hmmwv.SetSuspensionVisualizationType(vt)
    hmmwv.SetSteeringVisualizationType(vt)
    hmmwv.SetWheelVisualizationType(chrono.VisualizationType_NONE)
    hmmwv.SetTireVisualizationType(mesh_vt)

    system = hmmwv.GetSystem()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    print(f"vehicle mass: {hmmwv.GetVehicle().GetMass():.1f} kg   "
          f"tire: {hmmwv.GetVehicle().GetTire(1, veh.LEFT).GetTemplateName()}")

    # --- Terrain: collision-only OBJ as a RigidTerrain mesh patch ---
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChContactMaterialSMC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch_mat.SetYoungModulus(2e7)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, COLLISION_OBJ,
                             True,      # connected_mesh
                             0.002,     # sweep_sphere_radius
                             True)      # patch renders the terrain mesh
    terrain.Initialize()
    ground = patch.GetGroundBody()
    h0 = terrain.GetHeight(chrono.ChVector3d(INIT_LOC.x, INIT_LOC.y, 5.0))
    print(f"terrain: mesh patch fixed={ground.IsFixed()}, "
          f"height at spawn: {h0:.3f} m")

    # --- Driver + optional visualization ---
    vis = None
    driver = None
    if not args.headless:
        import pychrono.irrlicht  # noqa: F401

        # visual-only scenery (bushes/cones/rocks); no collision model at all
        if os.path.isfile(VISUAL_OBJ):
            vmesh = chrono.ChTriangleMeshConnected().CreateFromWavefrontFile(
                VISUAL_OBJ, True, True)
            vshape = chrono.ChVisualShapeTriangleMesh()
            vshape.SetMesh(vmesh)  # keep vmesh referenced: live-view gotcha
            vshape.SetMutable(False)
            ground.AddVisualShape(vshape)

        if interactive:
            # this build ships ChInteractiveDriver (+ AttachDriver); older
            # builds called it ChInteractiveDriverIRR
            driver_cls = getattr(veh, "ChInteractiveDriverIRR", None) \
                or veh.ChInteractiveDriver
            driver = driver_cls(hmmwv.GetVehicle())
            render_step = 1.0 / RENDER_FPS
            driver.SetSteeringDelta(render_step / 1.0)   # A/D
            driver.SetThrottleDelta(render_step / 1.0)   # W
            driver.SetBrakingDelta(render_step / 0.3)    # S
            driver.Initialize()

        vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
        vis.SetWindowTitle("HMMWV gauntlet")
        vis.SetWindowSize(1280, 800)
        vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
        vis.Initialize()
        vis.AddLightDirectional()
        vis.AddSkyBox()
        vis.AttachVehicle(hmmwv.GetVehicle())
        if driver is not None:
            vis.AttachDriver(driver)
        hmmwv.GetVehicle().EnableRealtime(True)

    # --- Diagnostics state ---
    max_speed = 0.0
    roll_max = pitch_max = 0.0
    z_min = z_max = INIT_LOC.z
    x_peak = INIT_LOC.x
    x_peak_time = 0.0
    contacts_min, contacts_max = None, 0
    nan_found = False
    outcome = None            # PASS_COURSE / EXPECTED_STALL / ...
    event_zone = event_x = None
    braking_until = None      # set when stall triggers the controlled stop
    next_sample = 0.0
    next_print = 0.0
    render_steps = max(1, int(round(1.0 / (RENDER_FPS * STEP_SIZE))))
    step = 0

    exit_reason = "event"
    while True:
        time = system.GetChTime()
        if time >= args.duration:
            exit_reason = "duration reached"
            break
        if vis is not None and not vis.Run():
            exit_reason = f"window closed at t={time:.2f}s"
            break

        if vis is not None and step % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        # driver inputs: interactive (visual) or scripted (headless)
        if driver is not None:
            driver_inputs = driver.GetInputs()
            driver.Synchronize(time)
        else:
            driver_inputs = veh.DriverInputs()
            driver_inputs.m_steering = 0.0
            if braking_until is not None:
                driver_inputs.m_throttle = 0.0
                driver_inputs.m_braking = 0.5
            elif time < SETTLE_TIME:
                driver_inputs.m_throttle = 0.0
                driver_inputs.m_braking = 0.0
            else:
                ramp = min(1.0, (time - SETTLE_TIME) / RAMP_TIME)
                driver_inputs.m_throttle = args.throttle * ramp
                driver_inputs.m_braking = 0.0

        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        if vis is not None:
            vis.Synchronize(time, driver_inputs)

        if driver is not None:
            driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        hmmwv.Advance(STEP_SIZE)
        if vis is not None:
            vis.Advance(STEP_SIZE)
        step += 1

        if time < next_sample:
            continue
        next_sample += SAMPLE_PERIOD

        p = hmmwv.GetVehicle().GetPos()
        speed = hmmwv.GetVehicle().GetSpeed()
        roll, pitch, yaw = quat_to_rpy(hmmwv.GetVehicle().GetRot())
        if any(math.isnan(v) for v in (p.x, p.y, p.z, speed, roll, pitch, yaw)):
            nan_found = True
            outcome = "FAIL_NAN"
            event_zone, event_x = zone_of(x_peak), x_peak
            print(f"[{time:6.2f}s] NaN detected — aborting")
            break

        max_speed = max(max_speed, speed)
        roll_max = max(roll_max, abs(roll))
        pitch_max = max(pitch_max, abs(pitch))
        z_min, z_max = min(z_min, p.z), max(z_max, p.z)
        nc = system.GetContactContainer().GetNumContacts()
        contacts_min = nc if contacts_min is None else min(contacts_min, nc)
        contacts_max = max(contacts_max, nc)

        if p.x > x_peak + STALL_PROGRESS:
            x_peak, x_peak_time = p.x, time

        if time >= next_print:
            next_print += args.print_dt
            print(f"[{time:6.2f}s] x={p.x:7.2f} y={p.y:5.2f} z={p.z:5.2f}  "
                  f"v={speed:5.2f} m/s  "
                  f"r/p/y={math.degrees(roll):6.1f}/{math.degrees(pitch):6.1f}/"
                  f"{math.degrees(yaw):6.1f}  {zone_of(p.x):<11s}  "
                  f"thr={driver_inputs.m_throttle:.2f} "
                  f"str={driver_inputs.m_steering:+.2f} "
                  f"brk={driver_inputs.m_braking:.2f}  nc={nc}")

        # rollover? At the ledge it is the designed outcome; anywhere else
        # it is a regression.
        if math.degrees(abs(roll)) > ROLLOVER_DEG or \
                math.degrees(abs(pitch)) > ROLLOVER_DEG:
            event_zone, event_x = zone_of(p.x), p.x
            outcome = "EXPECTED_FLIP" if event_zone == "hill_ledge" \
                else "FAIL_ROLLOVER"
            print(f"[{time:6.2f}s] ROLLOVER in {event_zone} at x={p.x:.1f} "
                  f"(roll {math.degrees(roll):.0f} deg, "
                  f"pitch {math.degrees(pitch):.0f} deg)")
            break

        # course complete?
        if p.x >= FINISH_X:
            outcome = "PASS_COURSE"
            event_zone, event_x = zone_of(p.x), p.x
            print(f"[{time:6.2f}s] reached x={p.x:.1f} — course complete")
            break

        # stall (headless only): x_peak has not advanced for STALL_WINDOW
        # while throttle is applied -> cut throttle, brake to a stop
        if driver is None and braking_until is None \
                and driver_inputs.m_throttle > 0.1 \
                and time - max(x_peak_time, SETTLE_TIME + RAMP_TIME) > STALL_WINDOW:
            event_zone, event_x = zone_of(x_peak), x_peak
            print(f"[{time:6.2f}s] STALL: no progress past x={x_peak:.1f} "
                  f"({event_zone}) for {STALL_WINDOW:.0f} s — braking to stop")
            braking_until = time + 5.0
        if braking_until is not None and (speed < 0.2 or time > braking_until):
            outcome = "EXPECTED_STALL" if event_zone == "hill_ledge" \
                else "FAIL_STALL"
            print(f"[{time:6.2f}s] stopped (v={speed:.2f} m/s)")
            break

    if outcome is None:
        # windowed sessions that end without incident (window closed or
        # duration reached) are not regressions — only headless runs must
        # terminate in an expected outcome
        outcome = "MANUAL_END" if vis is not None else "TIMEOUT"
        event_zone, event_x = zone_of(x_peak), x_peak

    # --- Final report ---
    p = hmmwv.GetVehicle().GetPos()
    print("\n===== FINAL REPORT (simulate_hmmwv) =====")
    print(f"loop exit: {exit_reason} (sim time {system.GetChTime():.2f} s)")
    mode = "interactive" if driver is not None else \
        ("scripted-windowed" if vis is not None else "headless")
    print(f"mode: {mode}   "
          f"throttle: {args.throttle if driver is None else 'manual'}   "
          f"duration limit: {args.duration} s")
    print(f"end pos: ({p.x:.2f}, {p.y:.2f}, {p.z:.2f})   "
          f"furthest x: {x_peak:.2f} ({zone_of(x_peak)})")
    print(f"speed max: {max_speed:.2f} m/s ({max_speed * 3.6:.1f} km/h)   "
          f"|roll| max: {math.degrees(roll_max):.1f} deg   "
          f"|pitch| max: {math.degrees(pitch_max):.1f} deg")
    print(f"chassis z range: [{z_min:.2f}, {z_max:.2f}]   "
          f"contacts: [{contacts_min}, {contacts_max}] "
          "(informational — TMEASY tires bypass the collision system)")
    print(f"NaN: {nan_found}")
    if event_zone is not None:
        print(f"terminal event zone: {event_zone} (x={event_x:.1f})")
    print(f"OUTCOME: {outcome}")
    ok = outcome in ("PASS_COURSE", "EXPECTED_STALL", "EXPECTED_FLIP",
                     "MANUAL_END")
    print(f"RESULT: {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
