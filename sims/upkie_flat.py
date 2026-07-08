"""Upkie wheeled biped balancing and driving on flat ground.

Loads the vendored upkie_description URDF (two legs, hip+knee+wheel each,
5.34 kg, wheel radius 0.05 m, CoM 0.314 m above ground and 6 mm behind the
axle). Hips and knees are position-held at the URDF zero pose (straight
legs); the two wheel joints get SPEED motors driven by a cascade balance
controller: pitch error -> desired base acceleration -> integrated wheel
velocity command (+ pitch-reference trim for velocity tracking / station
keeping, differential speed to damp yaw). TORQUE motors were tried first
and CANNOT work here: any clamp-level torque spins the 1.4e-4 kg.m^2 wheels
at ~12000 rad/s^2, traction scrubs off and the robot flips. The robot is an
inverted pendulum on two coaxial wheels, so "moving over a flat surface" =
balance + commanded forward velocity.

Phases: settle fixed (0-0.5 s) -> floor created under settled wheels ->
release at 1.0 s -> stabilize -> ramp to cruise velocity -> brake and hold.

Run visually:      python upkie_flat.py
Headless check:    python upkie_flat.py --headless
Torque sign cal:   python upkie_flat.py --calibrate-wheels
"""
import argparse
import math
import os

import pychrono as chrono

ASSETS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets")
ROBOT_DIR = os.path.join(ASSETS, "robots", "upkie_description")
ROBOT_URDF = os.path.join(ROBOT_DIR, "urdf", "upkie.urdf")
PACKAGE_PREFIX = "package://upkie_description"

WHEEL_JOINTS = ["left_wheel", "right_wheel"]
TIRE_BODIES = {"left_wheel": "left_wheel_tire", "right_wheel": "right_wheel_tire"}
WHEEL_RADIUS = 0.05
# +1 means positive motor speed drives the robot toward +x. Calibrated
# KINEMATICALLY (--calibrate-wheels): root fixed in the air, motor at
# +2 rad/s, read the tire's world angular velocity y-component (spin about
# +y = roll toward +x). Ground-based one-wheel tests are useless here: the
# unbalanced robot tips during the measurement and the other speed-locked
# wheel acts as a brake, which produced two contradictory wrong answers.
WHEEL_SIGNS = {"left_wheel": -1.0, "right_wheel": +1.0}

SPAWN = chrono.ChVector3d(0, 0, 1.0)

T_RELEASE = 1.0             # root unfixed, balance control active
T_CRUISE = 3.0              # start ramping forward velocity
T_BRAKE = 11.0              # ramp back to zero, station-keep
V_CRUISE = 0.5              # m/s
V_RAMP = 0.5                # m/s per s


def resolved_urdf_path():
    """ChParserURDF cannot resolve package:// URIs, so rewrite them to the
    vendored mesh directory. The result embeds an absolute path, so it is
    regenerated (not committed) and only rewritten when its content changes."""
    with open(ROBOT_URDF, encoding="utf-8") as f:
        text = f.read()
    text = text.replace(PACKAGE_PREFIX, ROBOT_DIR.replace(os.sep, "/"))
    # virtual links (base, imu, anchors, contacts) have exactly zero inertia
    # tensors, which crashes this Chrono build (the root body especially) —
    # give them a tiny but valid inertia instead
    text = text.replace(
        '<inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>',
        '<inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>')

    out_path = os.path.join(ROBOT_DIR, "urdf", "upkie_resolved.urdf")
    if os.path.isfile(out_path):
        with open(out_path, encoding="utf-8") as f:
            if f.read() == text:
                return out_path
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(text)
    return out_path


def add_robot(system):
    import pychrono.parsers as parsers

    parser = parsers.ChParserURDF(resolved_urdf_path())
    parser.SetRootInitPose(chrono.ChFramed(SPAWN, chrono.QUNIT))

    contact = chrono.ChContactMaterialData()
    contact.mu = 1.0        # tire lateral_friction from the URDF
    parser.SetDefaultContactMaterial(contact)

    # hips/knees hold the URDF zero pose; wheels get speed motors
    parser.SetAllJointsActuationType(parsers.ChParserURDF.ActuationType_POSITION)
    for name in WHEEL_JOINTS:
        parser.SetJointActuationType(name, parsers.ChParserURDF.ActuationType_SPEED)

    parser.PopulateSystem(system)

    # This parser build leaves EnableCollision off on every imported body.
    # Do NOT blanket-enable like the WL_P311D script: Upkie's collision
    # shapes overlap between welded neighbors (tire/hub, rotor/stator,
    # battery inside torso box), giving ~150 permanent self-contacts whose
    # forces tear the joints apart. Only the tires touch the world.
    for name in TIRE_BODIES.values():
        parser.GetChBody(name).EnableCollision(True)

    return parser


def get_wheel_motors(parser):
    """name -> setpoint function. We keep our own reference to the
    ChFunctionSetpoint (GetMotorFunction returns the ChFunction base)."""
    motors = {}
    for name in WHEEL_JOINTS:
        motor = parser.GetChMotor(name)
        func = chrono.ChFunctionSetpoint()
        motor.SetMotorFunction(func)
        motors[name] = func
    return motors


def add_floor(system, top_z, length=60.0, width=6.0, x_center=15.0):
    mat = chrono.ChContactMaterialNSC()
    mat.SetFriction(1.0)
    thick = 0.2
    floor = chrono.ChBodyEasyBox(length, width, thick, 1000, True, True, mat)
    floor.SetPos(chrono.ChVector3d(x_center, 0, top_z - thick / 2))
    floor.SetFixed(True)
    floor.GetVisualShape(0).SetColor(chrono.ChColor(0.35, 0.55, 0.25))
    system.Add(floor)
    return floor


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.005)
    chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.005)
    system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    # 150 iters (WL_P311D value) lets the joints drift apart here: 41 bodies,
    # 34 stacked fixed joints, gram-scale virtual links -> needs ~2000 to
    # converge (verified: base-torso stays exactly 0.100 m)
    system.GetSolver().AsIterative().SetMaxIterations(2000)

    parser = add_robot(system)

    # The floor must exist before the first DoStepDynamics: bodies added to a
    # running system never get bound into the Bullet collision world (verified:
    # a mid-run floor gives 0 contacts and the robot falls through). The robot
    # holds its parsed pose exactly, so the parsed tire height is the floor.
    tire_low = min(parser.GetChBody(b).GetPos().z
                   for b in TIRE_BODIES.values()) - WHEEL_RADIUS
    floor_top = tire_low - 0.002
    add_floor(system, floor_top)
    return system, parser, floor_top


def torso_state(torso):
    """(x, v_fwd, yaw, yaw_rate, pitch, pitch_rate); pitch > 0 = leaning
    toward the heading direction."""
    q = torso.GetRot()
    xw = q.Rotate(chrono.ChVector3d(1, 0, 0))
    zw = q.Rotate(chrono.ChVector3d(0, 0, 1))
    yaw = math.atan2(xw.y, xw.x)
    fx, fy = math.cos(yaw), math.sin(yaw)
    pitch = math.atan2(zw.x * fx + zw.y * fy, zw.z)

    w = torso.GetAngVelParent()
    yaw_rate = w.z
    pitch_rate = -w.x * fy + w.y * fx   # about the lateral (left) axis

    p = torso.GetPos()
    v = torso.GetPosDt()
    v_fwd = v.x * fx + v.y * fy
    return p.x, v_fwd, yaw, yaw_rate, pitch, pitch_rate


def pitch_trim(system, parser, floor_top):
    """Equilibrium pitch: the CoM sits 6 mm behind the axle at zero pose, so
    balancing requires a slight forward lean. Computed, not hardcoded."""
    m_tot = sx = sz = 0.0
    for b in system.GetBodies():
        if b.IsFixed():
            continue
        com = b.GetFrameCOMToAbs().GetPos()
        m_tot += b.GetMass()
        sx += b.GetMass() * com.x
        sz += b.GetMass() * com.z
    cx, cz = sx / m_tot, sz / m_tot
    axle_x = sum(parser.GetChBody(b).GetPos().x
                 for b in TIRE_BODIES.values()) / len(TIRE_BODIES)
    axle_z = floor_top + WHEEL_RADIUS
    return math.atan2(axle_x - cx, cz - axle_z)


class BalanceController:
    """Cascade balance for SPEED-actuated wheels. Inner loop: pitch error ->
    desired base acceleration -> integrated wheel velocity command (the motor
    is a constraint, so the wheel physically cannot overspin the way the
    torque-motor version did). Outer loop: trims the pitch reference to track
    the commanded velocity and hold station. Differential speed damps yaw."""

    KP = 25.0        # m/s^2 of base accel per rad of pitch error (> g)
    KD = 6.0         # m/s^2 per rad/s of pitch rate
    KV = 0.06        # rad of lean per m/s of velocity error
    KX = 0.03        # rad of lean per m of position error
    K_YAWRATE = 0.05  # m/s differential per rad/s of yaw rate
    ACCEL_MAX = 4.0  # keeps traction demand below mu*g
    V_MAX = 1.5
    LEAN_MAX = 0.10

    def __init__(self, trim):
        self.trim = trim
        self.v_cmd = 0.0
        self.x_ref = 0.0

    def wheel_speeds(self, torso, v_target, dt):
        x, _, _, yaw_rate, pitch, pitch_rate = torso_state(torso)
        self.x_ref += v_target * dt
        clamp = lambda v, lim: max(-lim, min(lim, v))

        lean = clamp(-self.KV * (self.v_cmd - v_target)
                     - self.KX * (x - self.x_ref), self.LEAN_MAX)
        accel = clamp(self.KP * (pitch - self.trim - lean)
                      + self.KD * pitch_rate, self.ACCEL_MAX)
        self.v_cmd = clamp(self.v_cmd + accel * dt, self.V_MAX)

        d = self.K_YAWRATE * yaw_rate
        # (left, right) forward wheel angular speed
        return (self.v_cmd + d) / WHEEL_RADIUS, (self.v_cmd - d) / WHEEL_RADIUS


def v_command(t):
    if t < T_CRUISE:
        return 0.0
    if t < T_BRAKE:
        return min(V_CRUISE, (t - T_CRUISE) * V_RAMP)
    return max(0.0, V_CRUISE - (t - T_BRAKE) * V_RAMP)


def run(system, parser, floor_top, duration=14.0, step=1e-3, vis=None, quiet=False):
    torso = parser.GetChBody("torso")
    root = parser.GetRootChBody()
    tires = {n: parser.GetChBody(b) for n, b in TIRE_BODIES.items()}
    motors = get_wheel_motors(parser)
    ctrl = BalanceController(pitch_trim(system, parser, floor_top))

    root.SetFixed(True)
    released = False

    stats = {"pitch_min": 0.0, "pitch_max": 0.0, "abs_rate_max": 0.0,
             "clear_min": 1e9, "nc_min": 1 << 30, "nc_max": 0, "fell": None}
    render_steps = max(1, int(round(1 / (60 * step))))
    n_steps = int(round(duration / step))

    for i in range(n_steps):
        t = system.GetChTime()

        if not released and t >= T_RELEASE:
            root.SetFixed(False)
            ctrl.x_ref = torso.GetPos().x
            released = True

        if released:
            wl, wr = ctrl.wheel_speeds(torso, v_command(t), step)
            motors["left_wheel"].SetSetpoint(WHEEL_SIGNS["left_wheel"] * wl, t)
            motors["right_wheel"].SetSetpoint(WHEEL_SIGNS["right_wheel"] * wr, t)

            x, v_fwd, yaw, _, pitch, pitch_rate = torso_state(torso)
            stats["pitch_min"] = min(stats["pitch_min"], pitch)
            stats["pitch_max"] = max(stats["pitch_max"], pitch)
            stats["abs_rate_max"] = max(stats["abs_rate_max"], abs(pitch_rate))
            clear = min(b.GetPos().z for b in tires.values()) - WHEEL_RADIUS - floor_top
            stats["clear_min"] = min(stats["clear_min"], clear)
            nc = system.GetContactContainer().GetNumContacts()
            stats["nc_min"] = min(stats["nc_min"], nc)
            stats["nc_max"] = max(stats["nc_max"], nc)
            if stats["fell"] is None and abs(pitch) > 0.6:
                stats["fell"] = t

        system.DoStepDynamics(step)

        if not quiet and i % 500 == 0 and released:
            x, v_fwd, yaw, _, pitch, _ = torso_state(torso)
            print(f"t={t:5.2f}s  x={x:+6.3f}  v={v_fwd:+5.2f}  "
                  f"pitch={math.degrees(pitch):+6.2f}deg  yaw={math.degrees(yaw):+6.2f}deg  "
                  f"z={torso.GetPos().z:5.3f}")

        if vis is not None and i % render_steps == 0:
            if not vis.Run():
                break
            p = torso.GetPos()
            vis.SetCameraPosition(chrono.ChVector3d(p.x - 1.5, p.y - 2.0, p.z + 0.8))
            vis.SetCameraTarget(p)
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

    x, v_fwd, yaw, _, pitch, _ = torso_state(torso)
    p = torso.GetPos()
    nan = not (p.x == p.x and p.z == p.z)
    upright = stats["fell"] is None and not nan
    dist = x - 0.0

    print("\n=== upkie flat report ===")
    print(f"final x={x:+.3f} m  (commanded travel ~{V_CRUISE * (T_BRAKE - T_CRUISE) - V_CRUISE**2 / (2 * V_RAMP):.2f} m)")
    print(f"pitch range [{math.degrees(stats['pitch_min']):+.2f}, "
          f"{math.degrees(stats['pitch_max']):+.2f}] deg, "
          f"max |pitch rate| {stats['abs_rate_max']:.2f} rad/s")
    print(f"final yaw {math.degrees(yaw):+.2f} deg  y-drift {p.y:+.3f} m")
    print(f"min wheel clearance {stats['clear_min']:+.4f} m  "
          f"contacts [{stats['nc_min']}, {stats['nc_max']}]")
    print(f"fell: {stats['fell']}  NaN: {nan}")

    ok = upright and dist > 2.5 and abs(p.y) < 1.0
    print("RESULT:", "PASS" if ok else "FAIL")
    return ok


def calibrate_wheels():
    """Kinematic sign test: root fixed (tires hang 2 mm off the floor), spin
    one motor at +2 rad/s, read the tire's world angular velocity. Spin about
    +y rolls toward +x. Do NOT measure this by driving on the ground: the
    unbalanced robot tips during the test and the other wheel is speed-locked
    (a brake), which yields garbage displacements."""
    for name in WHEEL_JOINTS:
        system, parser, _ = build_system()
        parser.GetRootChBody().SetFixed(True)
        motors = get_wheel_motors(parser)
        while system.GetChTime() < 0.3:
            motors[name].SetSetpoint(2.0, system.GetChTime())
            system.DoStepDynamics(1e-3)
        w = parser.GetChBody(TIRE_BODIES[name]).GetAngVelParent()
        print(f"{name}: +2 rad/s -> tire world omega_y={w.y:+.3f}"
              f"  => sign {'+1' if w.y > 0 else '-1'}")


def run_visual():
    import pychrono.irrlicht as chronoirr

    system, parser, floor_top = build_system()
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 800)
    vis.SetWindowTitle("Upkie flat ground")
    vis.Initialize()
    vis.AddTypicalLights()
    vis.AddCamera(chrono.ChVector3d(-1.5, -2.0, 1.5), chrono.ChVector3d(0, 0, 0.6))
    run(system, parser, floor_top, vis=vis)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--headless", action="store_true", help="run without a window")
    ap.add_argument("--calibrate-wheels", action="store_true",
                    help="measure per-wheel torque sign -> +x displacement")
    ap.add_argument("--duration", type=float, default=14.0)
    args = ap.parse_args()

    if args.calibrate_wheels:
        calibrate_wheels()
    elif args.headless:
        system, parser, floor_top = build_system()
        ok = run(system, parser, floor_top, duration=args.duration, quiet=False)
        raise SystemExit(0 if ok else 1)
    else:
        run_visual()
