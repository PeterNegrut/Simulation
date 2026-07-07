"""Scripted waypoint navigation for the WL_P311D on the practice terrain.

Copy of simulate_terrain.py extended with wheel-speed control: legs stay
position-held at the URDF zero pose, the four wheel joints become
speed-actuated, and a differential-drive controller follows a fixed waypoint
route over ONE terrain feature at a time. No autonomy, no stairs/hurdles —
those need active leg posture.

Base frame: +x forward (front hips at x=+0.216), +y left, +z up.

Run a course:        python simulate_terrain_nav.py --course pipes
Headless check:      python simulate_terrain_nav.py --course pipes --headless
Wheel sign test:     python simulate_terrain_nav.py --calibrate-wheels
"""
import argparse
import math
import os

import pychrono as chrono
import pychrono.parsers as parsers

ASSETS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets")

# color per obstacle group, matching the Blender materials
MESHES = {
    "terrain.obj": {"color": (0.35, 0.55, 0.25), "friction": 0.9},
    "steps.obj":   {"color": (0.25, 0.35, 0.60), "friction": 0.8},
    "rubble.obj":  {"color": (0.45, 0.25, 0.18), "friction": 0.8},
    "ramp.obj":    {"color": (0.90, 0.45, 0.10), "friction": 0.7},
    "pipes.obj":   {"color": (0.90, 0.75, 0.10), "friction": 0.5},
    "hurdles.obj": {"color": (0.80, 0.15, 0.10), "friction": 0.8},
}

# course -> obstacle meshes to load (terrain.obj is always loaded) + route.
# Steps and hurdles are deliberately absent: not fair with legs frozen at zero.
# "speed" is the course target speed (m/s); obstacle crossings get a line-up
# waypoint before the feature so the robot enters straight (oblique entry into
# the pipes rolled it to 88 deg and wedged it between two pipes).
COURSES = {
    "flat":   {"meshes": [], "speed": 0.25,
               "route": [(0, -8), (0, -5), (0, -2)],
               "timeout": 60},
    # KNOWN FAIL with legs frozen: pipe crown ~0.17 m > wheel radius 0.127 m
    # (obstacle above axle height, mu=0.5). 0.35 m/s stalls square against the
    # first pipe; 0.7 m/s momentum entry rolls the robot to -62 deg and tips.
    # Needs active leg posture/stepping to pass.
    "pipes":  {"meshes": ["pipes.obj"], "speed": 0.35,
               "route": [(0, -8), (0, -6.0), (0, -5.2), (0, -2.0)],
               "timeout": 120},
    # KNOWN FAIL with legs frozen: ramp.obj is a floating slab — its top
    # surface starts at z=0.30 over terrain at z~0.13, an effective 0.17 m
    # sharp step onto the ramp (> wheel radius). The 17 deg slope itself would
    # be fine at mu=0.7; the entry lip is the blocker.
    "ramp":   {"meshes": ["ramp.obj"], "speed": 0.30,
               "route": [(0, -8), (5, -5), (5, -3.8), (5, 0.5)],
               "timeout": 150},
    # Route threads the mapped rock-free seam: the field's south edge is
    # 16-37 cm rocks (> wheel radius, impassable), so enter from the east
    # along the clear y~-1.7 corridor, then north up the clean x~-5.4 lane.
    "rubble": {"meshes": ["rubble.obj"], "speed": 0.20,
               "route": [(0, -8), (-1.6, -6.0), (-1.6, -1.9), (-3.6, -1.55),
                         (-5.4, -1.6), (-5.4, 0.8)],
               "timeout": 240},
    # Mixed: both features loaded, route threads the corridor between the
    # pipe field (x<=1.5) and the ramp slab (x>=4), then climbs the smooth
    # terrain hill (~6-9 deg) to the same goal the ramp would have reached.
    "mixed":  {"meshes": ["pipes.obj", "ramp.obj"], "speed": 0.30,
               "route": [(0, -8), (2.7, -6.0), (2.7, -3.0), (3.5, -0.5), (5, 0.5)],
               "timeout": 240},
    "all":    {"meshes": list(MESHES.keys())[1:], "speed": 0.25,
               "route": [(0, -8), (0, -5), (0, -2)],
               "timeout": 60},
}

START_POS = chrono.ChVector3d(0, -8, 1.0)  # above the flat start zone

ROBOT_DIR = os.path.join(ASSETS, "robots", "WL_P311D")
ROBOT_URDF = os.path.join(ROBOT_DIR, "urdf", "robot.urdf")
PACKAGE_PREFIX = "package://robot_description/wheellegged/WL_P311D"

WHEEL_JOINTS = ["LF_WHL", "RF_WHL", "LH_WHL", "RH_WHL"]
LEFT_WHEELS = ("LF_WHL", "LH_WHL")

# Calibrated with --calibrate-wheels: raw +2 rad/s on all wheels drove the
# base BACKWARD 0.76 m in 3 s (= 2*0.127*3, pure rolling), raw left+/right-
# yawed LEFT +65 deg. Sign -1 makes +command = forward, and keeps
# left=base-turn / right=base+turn turning LEFT for positive turn.
WHEEL_SIGNS = {
    "LF_WHL": -1.0,
    "RF_WHL": -1.0,
    "LH_WHL": -1.0,
    "RH_WHL": -1.0,
}

WHEEL_RADIUS = 0.127        # from the URDF wheel collision cylinders
MAX_WHEEL_SPEED = 6.0       # rad/s
YAW_GAIN = 2.0              # rad/s wheel-speed differential per rad of yaw error
TURN_CLAMP = 2.5            # rad/s max differential
ALIGN_ERR = 0.45            # rad; above this, turn in place instead of arcing
WAYPOINT_TOL = 0.35         # m, planar
SETTLE_TIME = 1.0           # s of zero command after the drop
RAMP_TIME = 1.0             # s to ramp commands up after settling
STEP = 1e-3

# failure gates
TIP_LIMIT_DEG = 45.0
MIN_CHASSIS_Z = -1.0        # terrain z min is -0.41; below this we fell through
STALL_WINDOW = 5.0          # s without progress
STALL_DIST = 0.05           # m of motion that still counts as stalled
NO_CONTACT_GRACE = 1.0      # s of zero contacts after settling


def add_static_mesh(system, objpath, color, friction):
    mat = chrono.ChContactMaterialNSC()
    mat.SetFriction(friction)

    mesh = chrono.ChTriangleMeshConnected.CreateFromWavefrontFile(objpath, True, True)

    body = chrono.ChBody()
    body.SetFixed(True)
    shape = chrono.ChCollisionShapeTriangleMesh(mat, mesh, True, False, 0.002)
    body.AddCollisionShape(shape)
    body.EnableCollision(True)

    vis = chrono.ChVisualShapeTriangleMesh()
    vis.SetMesh(mesh)
    vis.SetColor(chrono.ChColor(*color))
    body.AddVisualShape(vis)

    system.Add(body)
    return body


def resolved_urdf_path():
    """ChParserURDF cannot resolve package:// URIs, so rewrite them to the
    vendored mesh directory (regenerated, not committed)."""
    with open(ROBOT_URDF, encoding="utf-8") as f:
        text = f.read()
    text = text.replace(PACKAGE_PREFIX, ROBOT_DIR.replace(os.sep, "/"))

    out_path = os.path.join(ROBOT_DIR, "urdf", "robot_resolved.urdf")
    if os.path.isfile(out_path):
        with open(out_path, encoding="utf-8") as f:
            if f.read() == text:
                return out_path
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(text)
    return out_path


def add_robot(system):
    parser = parsers.ChParserURDF(resolved_urdf_path())
    parser.SetRootInitPose(chrono.ChFramed(START_POS, chrono.QUNIT))

    contact = chrono.ChContactMaterialData()
    contact.mu = 0.8
    parser.SetDefaultContactMaterial(contact)

    # legs hold the URDF zero pose; wheels get speed motors
    parser.SetAllJointsActuationType(parsers.ChParserURDF.ActuationType_POSITION)
    for name in WHEEL_JOINTS:
        parser.SetJointActuationType(name, parsers.ChParserURDF.ActuationType_SPEED)

    parser.PopulateSystem(system)

    # This parser build leaves EnableCollision off on every imported body
    # even though it created the collision shapes — switch them on.
    for body in system.GetBodies():
        model = body.GetCollisionModel()
        if not body.IsFixed() and model is not None and model.GetNumShapes() > 0:
            body.EnableCollision(True)

    return parser, parser.GetRootChBody()


def get_wheel_motors(parser):
    """name -> (motor, setpoint_function). We keep our own reference to the
    ChFunctionSetpoint (GetMotorFunction returns the ChFunction base)."""
    motors = {}
    for name in WHEEL_JOINTS:
        motor = parser.GetChMotor(name)
        func = chrono.ChFunctionSetpoint()
        motor.SetMotorFunction(func)
        motors[name] = (motor, func)
    return motors


def set_wheel_speeds(motors, left_w, right_w, t, use_signs=True):
    for name, (_, func) in motors.items():
        w = left_w if name in LEFT_WHEELS else right_w
        if use_signs:
            w *= WHEEL_SIGNS[name]
        func.SetSetpoint(w, t)


def build_system(course):
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.005)
    chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.005)
    system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    system.GetSolver().AsIterative().SetMaxIterations(150)

    for fname in ["terrain.obj"] + COURSES[course]["meshes"]:
        path = os.path.join(ASSETS, fname)
        if not os.path.isfile(path):
            raise FileNotFoundError(path)
        props = MESHES[fname]
        add_static_mesh(system, path, props["color"], props["friction"])

    parser, robot = add_robot(system)
    motors = get_wheel_motors(parser)
    return system, parser, robot, motors


# ---------------------------------------------------------------- kinematics

def wrap_angle(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def base_angles(robot):
    """(yaw, pitch, roll) in rad from the rotated base axes.
    pitch > 0 = nose up, roll > 0 = left side up."""
    rot = robot.GetRot()
    fwd = rot.Rotate(chrono.ChVector3d(1, 0, 0))
    left = rot.Rotate(chrono.ChVector3d(0, 1, 0))
    yaw = math.atan2(fwd.y, fwd.x)
    pitch = math.asin(max(-1.0, min(1.0, fwd.z)))
    roll = math.asin(max(-1.0, min(1.0, left.z)))
    return yaw, pitch, roll


# ---------------------------------------------------------------- navigation

def run_nav(system, robot, motors, course, headless, print_dt=0.1):
    route = COURSES[course]["route"]
    timeout = COURSES[course]["timeout"]

    vis = None
    if not headless:
        import pychrono.irrlicht as chronoirr
        vis = chronoirr.ChVisualSystemIrrlicht()
        vis.AttachSystem(system)
        vis.SetWindowSize(1280, 800)
        vis.SetWindowTitle(f"WL_P311D nav — {course}")
        vis.Initialize()
        vis.AddTypicalLights()
        vis.AddCamera(chrono.ChVector3d(4, -12, 3), chrono.ChVector3d(0, -6, 0.5))
    render_steps = max(1, int(1 / 60 / STEP))  # ~60 fps

    start = None                # planar start, captured after settling
    wp_idx = 0
    next_print = 0.0
    left_cmd = right_cmd = 0.0
    reached = False
    failure = None
    tipped = False
    max_roll = max_pitch = 0.0
    min_contacts = None
    max_contacts = 0
    last_contact_t = 0.0
    progress_log = []           # (t, x, y) samples for the stall gate
    step_count = 0

    while system.GetChTime() < timeout:
        t = system.GetChTime()
        p = robot.GetPos()
        yaw, pitch, roll = base_angles(robot)

        # --- controller
        if t < SETTLE_TIME:
            left_cmd = right_cmd = 0.0
        else:
            if start is None:
                start = (p.x, p.y)
                print(f"settled at ({p.x:.2f}, {p.y:.2f}, {p.z:.2f}), nav start")
            scale = min(1.0, (t - SETTLE_TIME) / RAMP_TIME)
            tx, ty = route[wp_idx]
            dist = math.hypot(tx - p.x, ty - p.y)
            if dist < WAYPOINT_TOL:
                if wp_idx == len(route) - 1:
                    reached = True
                    break
                wp_idx += 1
                tx, ty = route[wp_idx]
                dist = math.hypot(tx - p.x, ty - p.y)
            yaw_err = wrap_angle(math.atan2(ty - p.y, tx - p.x) - yaw)
            turn = max(-TURN_CLAMP, min(TURN_CLAMP, YAW_GAIN * yaw_err))
            base_w = COURSES[course]["speed"] / WHEEL_RADIUS
            if abs(yaw_err) > ALIGN_ERR:
                base_w *= 0.15  # mostly turn in place; keeps obstacle entry square
            left_cmd = max(-MAX_WHEEL_SPEED, min(MAX_WHEEL_SPEED, (base_w - turn) * scale))
            right_cmd = max(-MAX_WHEEL_SPEED, min(MAX_WHEEL_SPEED, (base_w + turn) * scale))
        set_wheel_speeds(motors, left_cmd, right_cmd, t)

        system.DoStepDynamics(STEP)
        step_count += 1

        # --- bookkeeping
        contacts = system.GetContactContainer().GetNumContacts()
        if contacts > 0:
            last_contact_t = t
        if t > SETTLE_TIME:
            min_contacts = contacts if min_contacts is None else min(min_contacts, contacts)
            max_contacts = max(max_contacts, contacts)
        max_roll = max(max_roll, abs(roll))
        max_pitch = max(max_pitch, abs(pitch))

        # --- render
        if vis is not None and step_count % render_steps == 0:
            if not vis.Run():
                failure = "window closed"
                break
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            cp = robot.GetPos()
            vis.SetCameraPosition(chrono.ChVector3d(cp.x + 3.0, cp.y - 3.0, cp.z + 2.0))
            vis.SetCameraTarget(chrono.ChVector3d(cp.x, cp.y, cp.z))

        # --- diagnostics + failure gates at print cadence
        if t >= next_print:
            next_print += print_dt
            p = robot.GetPos()
            if p.x != p.x or p.z != p.z:
                failure = "NaN position"
                break
            fwd_prog = math.hypot(p.x - start[0], p.y - start[1]) if start else 0.0
            tx, ty = route[wp_idx]
            dist = math.hypot(tx - p.x, ty - p.y)
            yaw, pitch, roll = base_angles(robot)
            yaw_err = wrap_angle(math.atan2(ty - p.y, tx - p.x) - yaw)
            print(f"t={t:6.2f}  pos=({p.x:6.2f},{p.y:6.2f},{p.z:5.2f})  wp={wp_idx}"
                  f"  dist={dist:5.2f}  yaw_err={math.degrees(yaw_err):6.1f}"
                  f"  rp=({math.degrees(roll):5.1f},{math.degrees(pitch):5.1f})"
                  f"  nc={contacts:2d}  cmd=({left_cmd:5.2f},{right_cmd:5.2f})"
                  f"  prog={fwd_prog:5.2f}")

            if abs(math.degrees(roll)) > TIP_LIMIT_DEG or abs(math.degrees(pitch)) > TIP_LIMIT_DEG:
                tipped = True
                failure = f"tipped (roll={math.degrees(roll):.0f}, pitch={math.degrees(pitch):.0f})"
                break
            if p.z < MIN_CHASSIS_Z:
                failure = f"chassis z={p.z:.2f} below terrain"
                break
            if t > SETTLE_TIME + 1.5 and t - last_contact_t > NO_CONTACT_GRACE:
                failure = "no contacts after settling"
                break
            progress_log.append((t, p.x, p.y))
            if start is not None and t > SETTLE_TIME + STALL_WINDOW:
                told = t - STALL_WINDOW
                old = next(s for s in progress_log if s[0] >= told)
                if math.hypot(p.x - old[1], p.y - old[2]) < STALL_DIST:
                    failure = f"no progress for {STALL_WINDOW:.0f}s"
                    break

    # --- end-of-run report
    p = robot.GetPos()
    nan = p.x != p.x or p.y != p.y or p.z != p.z
    disp = math.hypot(p.x - start[0], p.y - start[1]) if start else 0.0
    print("\n=== NAV REPORT ===")
    print(f"course:            {course}")
    print(f"sim time:          {system.GetChTime():.2f} s")
    print(f"final pos:         ({p.x:.2f}, {p.y:.2f}, {p.z:.2f})")
    print(f"total displacement:{disp:6.2f} m")
    print(f"reached final wp:  {'YES' if reached else 'NO'} (wp {wp_idx}/{len(route)-1})")
    print(f"max roll:          {math.degrees(max_roll):5.1f} deg")
    print(f"max pitch:         {math.degrees(max_pitch):5.1f} deg")
    print(f"contacts min/max:  {min_contacts}/{max_contacts}")
    print(f"NaN:               {nan}")
    print(f"tipped:            {tipped}")
    print(f"result:            {'PASS' if reached and not failure else 'FAIL: ' + (failure or 'timeout')}")
    if vis is not None:
        vis.GetDevice().closeDevice()
    return reached and not failure


# --------------------------------------------------------------- calibration

def run_calibration():
    """Drive raw wheel speeds (no WHEEL_SIGNS) on flat ground and report what
    the base does, to establish the sign convention."""
    tests = [
        ("all +2",              +2.0, +2.0),
        ("all -2",              -2.0, -2.0),
        ("left +2 / right -2",  +2.0, -2.0),
        ("left -2 / right +2",  -2.0, +2.0),
    ]
    for label, lw, rw in tests:
        system, _parser, robot, motors = build_system("flat")
        start = start_yaw = None
        while system.GetChTime() < SETTLE_TIME + 3.0:
            t = system.GetChTime()
            if t < SETTLE_TIME:
                set_wheel_speeds(motors, 0.0, 0.0, t, use_signs=False)
            else:
                if start is None:
                    p = robot.GetPos()
                    start = (p.x, p.y)
                    start_yaw, _, _ = base_angles(robot)
                set_wheel_speeds(motors, lw, rw, t, use_signs=False)
            system.DoStepDynamics(STEP)

        p = robot.GetPos()
        yaw, _, _ = base_angles(robot)
        dx, dy = p.x - start[0], p.y - start[1]
        dyaw = math.degrees(wrap_angle(yaw - start_yaw))
        # forward = component of displacement along the heading at test start
        fwd = dx * math.cos(start_yaw) + dy * math.sin(start_yaw)
        if abs(dyaw) > 20:
            verdict = "TURNED " + ("LEFT" if dyaw > 0 else "RIGHT")
        elif abs(fwd) > 0.1:
            verdict = "moved mostly " + ("FORWARD" if fwd > 0 else "BACKWARD")
        else:
            verdict = "no significant motion"
        print(f"{label:22s} dx={dx:6.2f} dy={dy:6.2f} |d|={math.hypot(dx, dy):5.2f}"
              f"  dyaw={dyaw:7.1f} deg  fwd={fwd:6.2f}  -> {verdict}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--course", choices=sorted(COURSES), default="flat")
    ap.add_argument("--headless", action="store_true", help="run without a window")
    ap.add_argument("--calibrate-wheels", action="store_true",
                    help="wheel sign test on flat ground (headless)")
    ap.add_argument("--print-dt", type=float, default=0.1,
                    help="diagnostics print interval, s")
    args = ap.parse_args()

    if args.calibrate_wheels:
        run_calibration()
    else:
        system, _parser, robot, motors = build_system(args.course)
        ok = run_nav(system, robot, motors, args.course, args.headless, args.print_dt)
        raise SystemExit(0 if ok else 1)
