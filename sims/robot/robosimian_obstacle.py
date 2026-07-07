"""RoboSimian, stage-3 terrain: flat ground with a single low obstacle laid
across the path. Follows the validated stage-2 pattern
(sims/robot/robosimian_ramp.py); the flat baseline sims/robot/robosimian.py and
the ramp file stay untouched as known-good references.

Stage-3 headline result: DRIVE MODE CANNOT CLIMB ANYTHING. Sharp steps of
0.08/0.04/0.02m and smooth bumps of 0.03/0.02m crown all stall the robot at
first contact, at mu=0.8 AND mu=1.5 (bit-for-bit identical stalls) -- the
binding constraint is the drive controller/wheel-torque path, not friction or
geometry. Drive is a flat-ground mode; obstacles are crossed with the WALKING
gait (9cm foot lift vs 3cm bump), exactly how the real JPL robot operates.
Hence MODE defaults to walk on obstacle terrains and drive on flat.

Terrain (TERRAIN_MODE, env var RS_TERRAIN): "bump" (default) is an 8m flat
floor plus a cylinder lying across the path, buried so only a smooth 0.03m
crown protrudes -- no corner, so a rolling wheel sees a gradually increasing
slope. "obstacle" is a sharp box step, kept as a mobility-envelope demo (see
below). "flat" is the regression fallback. Visual shapes are built from the
same dims/frames as the collision shapes. Procedural Chrono primitives only.

Sharp-step finding (measured): drive mode CANNOT climb a sharp step of ANY
tested height -- 0.08, 0.04, and 0.02m all stall the robot with the front
wheels pressed against the step face at exactly the contact geometry
x = step_x - sqrt(r^2-(r-h)^2), average speed collapsing from 0.048 to
~0.008 m/s. It is NOT friction-limited: raising contact friction from 0.8 to
1.5 reproduces the stall bit-for-bit, so the binding constraint is the drive
controller/wheel-torque path, not grip. That matches the real robot's usage:
RoboSimian drives on clear ground and WALKS over obstacles. Hence stage 3's
passable obstacle is the smooth bump; sharp steps are walking-gait territory.

Pacing: walking covers ~0.0104 m/s, so SIM_END is 180s in walk mode -- front
wheels (x ~ +0.46) reach the bump around t~65s and the rear wheels (x ~ -0.46)
clear it around t~170s. Drive mode (flat terrain) keeps 60s.

Wheel clearance is measured against the LOCAL terrain height under each wheel
(floor / step top / bump crown profile). Expect brief LOW readings while a
wheel center crosses an obstacle edge but the wheel still rests on the corner
or lower surface, and HIGH readings (~+0.21) in walk mode when a foot lifts --
both are real geometry, not hovering/sinking. Sinking means clearly negative.

Pass criteria (checked in the end-of-run report): front axle reaches AND both
axles fully cross the obstacle; contacts stay nonzero; no NaNs; no
hovering/sinking beyond the expected edge transients; measurable displacement;
pitch shows bumps as each axle crosses but changes smoothly, not explosively.
"""
import math
import os
import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.robot as robosimian

TERRAIN_MODE = os.environ.get("RS_TERRAIN", "bump")  # "flat", "obstacle" (sharp step), "bump"
# Drive mode cannot climb ANY obstacle (see docstring), so obstacle terrains
# default to the walking gait; flat defaults to drive. Override with RS_MODE.
MODE = os.environ.get("RS_MODE", "drive" if TERRAIN_MODE == "flat" else "walk")
# Walking covers ~0.0104 m/s, so a full four-wheel crossing needs ~170s.
SIM_END = 60.0 if MODE == "drive" else 180.0
SHOW_COLLISION = False  # wireframe collision-shape overlay (debugging)
SHOW_CONTACTS = False   # draw contact normals (debugging)

ACTUATION = {
    "walk": ("", "walking_cycle.txt", ""),
    "drive": ("driving_start.txt", "driving_cycle.txt", "driving_stop.txt"),
    "scull": ("sculling_start.txt", "sculling_cycle2.txt", "sculling_stop.txt"),
    "inchworm": ("inchworming_start.txt", "inchworming_cycle.txt", "inchworming_stop.txt"),
}

OBSTACLE_X = 1.0       # leading edge of the obstacle strip
# Validated limit: 0.08 and 0.04 both stall the robot against the step face
# (climbing a sharp step needs mu >= sqrt(h(2r-h))/(r-h); mu=0.8 gives
# h_max ~ 0.026 for r=0.118). 0.02 is inside the friction-limited envelope.
OBSTACLE_HEIGHT = 0.02
OBSTACLE_DEPTH = 0.4   # extent along the path (x)
WHEEL_RADIUS = 0.118   # from validated touching clearance on flat ground

# Bump: a cylinder lying across the path, buried so it protrudes smoothly --
# no corner, so the wheel sees a gradually increasing slope it can roll up.
# Required friction at first contact ~ tan(acos((r+R-p)/(r+R))) ~ 0.59 < 0.8.
BUMP_X = 1.2            # cylinder axis x-position
BUMP_RADIUS = 0.10
BUMP_PROTRUSION = 0.03  # height of the exposed crown above the floor
TERRAIN_WIDTH = 3.0
FLOOR_LENGTH = 8.0
THICKNESS = 0.2

duration_pose = 1.0          # time to assume the initial pose before terrain exists
duration_settle_robot = 0.5  # time to let it settle on terrain before actuation starts
time_create_terrain = duration_pose

WHEEL_IDS = {"FR": robosimian.FR, "RR": robosimian.RR,
             "RL": robosimian.RL, "FL": robosimian.FL}


def make_ground_material(sys):
    ground_mat = chrono.ChContactMaterial.DefaultMaterial(sys.GetContactMethod())
    ground_mat.SetFriction(0.8)
    ground_mat.SetRestitution(0)
    chrono.CastToChContactMaterialSMC(ground_mat).SetYoungModulus(1e7)
    return ground_mat


def create_terrain(sys, terrain_z):
    """8m flat floor plus (in obstacle mode) one low box strip across the path.
    Returns (ground body, height_under(x) function for the top surface)."""
    ground_mat = make_ground_material(sys)
    ground = chrono.ChBody()
    ground.SetFixed(True)
    ground.EnableCollision(True)

    def add_box(dims, frame):
        shape = chrono.ChCollisionShapeBox(ground_mat, *dims)
        ground.AddCollisionShape(shape, frame)
        box = chrono.ChVisualShapeBox(*dims)  # visual == collision, same dims/frame
        box.SetTexture(chrono.GetChronoDataFile("textures/pinkwhite.png"),
                       2 * dims[0], 2 * dims[1])
        ground.AddVisualShape(box, frame)

    # Floor spans x in [-2, +6]: well past everything either axle reaches in 60s.
    floor_frame = chrono.ChFramed(
        chrono.ChVector3d(2, 0, terrain_z - THICKNESS / 2), chrono.QUNIT)
    add_box((FLOOR_LENGTH, TERRAIN_WIDTH, THICKNESS), floor_frame)

    if TERRAIN_MODE == "obstacle":
        ob_frame = chrono.ChFramed(
            chrono.ChVector3d(OBSTACLE_X + OBSTACLE_DEPTH / 2, 0,
                              terrain_z + OBSTACLE_HEIGHT / 2),
            chrono.QUNIT)
        add_box((OBSTACLE_DEPTH, TERRAIN_WIDTH, OBSTACLE_HEIGHT), ob_frame)

    bump_center_z = terrain_z + BUMP_PROTRUSION - BUMP_RADIUS  # buried cylinder axis
    bump_half_width = math.sqrt(BUMP_RADIUS**2 - (BUMP_RADIUS - BUMP_PROTRUSION)**2)
    if TERRAIN_MODE == "bump":
        # Chrono 9 cylinders run along local Z; rotate 90deg about X to lie
        # across the path (axis along world Y).
        bump_frame = chrono.ChFramed(
            chrono.ChVector3d(BUMP_X, 0, bump_center_z),
            chrono.QuatFromAngleX(chrono.CH_PI_2))
        shape = chrono.ChCollisionShapeCylinder(ground_mat, BUMP_RADIUS, TERRAIN_WIDTH)
        ground.AddCollisionShape(shape, bump_frame)
        cyl = chrono.ChVisualShapeCylinder(BUMP_RADIUS, TERRAIN_WIDTH)
        ground.AddVisualShape(cyl, bump_frame)  # visual == collision, same dims/frame

    sys.GetCollisionSystem().BindItem(ground)
    sys.AddBody(ground)

    def height_under(x):
        """Terrain top-surface height at world x (assumes |y| < TERRAIN_WIDTH/2)."""
        if TERRAIN_MODE == "obstacle" and OBSTACLE_X <= x <= OBSTACLE_X + OBSTACLE_DEPTH:
            return terrain_z + OBSTACLE_HEIGHT
        if TERRAIN_MODE == "bump" and abs(x - BUMP_X) < bump_half_width:
            return bump_center_z + math.sqrt(BUMP_RADIUS**2 - (x - BUMP_X)**2)
        return terrain_z

    return ground, height_under


def set_contact_properties(robot):
    friction, restitution, young_modulus = 0.8, 0.0, 1e7
    for mat in (robot.GetSledContactMaterial(), robot.GetWheelContactMaterial()):
        mat.SetFriction(friction)
        mat.SetRestitution(restitution)
        chrono.CastToChContactMaterialSMC(mat).SetYoungModulus(young_modulus)


if TERRAIN_MODE not in ("flat", "obstacle", "bump"):
    raise SystemExit(f"FATAL: unknown TERRAIN_MODE {TERRAIN_MODE!r}; "
                     f"use 'flat', 'obstacle', or 'bump'")
if MODE not in ACTUATION:
    raise SystemExit(f"FATAL: unknown MODE {MODE!r}; pick one of {sorted(ACTUATION)}")

# Obstacle region along x for the reached/crossed detectors.
_bump_hw = math.sqrt(BUMP_RADIUS**2 - (BUMP_RADIUS - BUMP_PROTRUSION)**2)
REGION_START, REGION_END = {
    "flat": (None, None),
    "obstacle": (OBSTACLE_X, OBSTACLE_X + OBSTACLE_DEPTH),
    "bump": (BUMP_X - _bump_hw, BUMP_X + _bump_hw),
}[TERRAIN_MODE]

sys = chrono.ChSystemSMC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.GetSolver().AsIterative().SetMaxIterations(200)
sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.8))

robot = robosimian.RoboSimian(sys, True, True)
robot.Initialize(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI)))

# Driver compat shim: same class, renamed across Chrono versions.
DriverClass = getattr(robosimian, "ChRobotActuation", None) or robosimian.RS_Driver

actuation_dir = "robot/robosimian/actuation/"
start_file, cycle_file, stop_file = (
    chrono.GetChronoDataFile(actuation_dir + f) if f else "" for f in ACTUATION[MODE]
)
driver = DriverClass(start_file, cycle_file, stop_file, True)
if MODE == "drive":
    driver.SetDrivingMode(True)  # actuation file carries wheel speeds, not just limb angles
cbk = robosimian.RS_DriverCallback(robot)
driver.RegisterPhaseChangeCallback(cbk)
driver.SetTimeOffsets(duration_pose, duration_settle_robot)
robot.SetDriver(driver)

print(f"Using Chrono RoboSimian compiled model, mode={MODE}, terrain={TERRAIN_MODE}")
print("Driver:", type(driver))
print("Robot chassis fixed initially:", robot.GetChassisBody().IsFixed())
if not robot.GetChassisBody().IsFixed():
    raise SystemExit("FATAL: chassis must be fixed during the pose phase")

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle(f"RoboSimian - {TERRAIN_MODE} terrain [{MODE}]")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, -2.75, 0.75), chrono.ChVector3d(0, 0, 0))
vis.AddLight(chrono.ChVector3d(100, 100, 100), 290)
vis.AddLight(chrono.ChVector3d(100, -100, 80), 190)

if SHOW_COLLISION:
    vis.EnableCollisionShapeDrawing(True)
if SHOW_CONTACTS:
    vis.EnableContactDrawing(chronoirr.ContactsDrawMode_CONTACT_NORMALS)

contact_container = sys.GetContactContainer()
time_step = 1e-3
render_fps = 60
render_steps = max(1, int((1.0 / render_fps) / time_step))
cam_offset = chrono.ChVector3d(0, -2.75, 0.75)  # chase camera: chassis + this

terrain_created = False
height_under = None
start_pos = None
start_z = None
step = 0

# End-of-run stats (collected every 0.1s once the robot has settled on terrain)
stats_t0 = time_create_terrain + duration_settle_robot
clear_min, clear_max = float("inf"), float("-inf")
contacts_min, contacts_max = float("inf"), float("-inf")
max_tilt_deg = 0.0
pitch_min_deg, pitch_max_deg = float("inf"), float("-inf")
max_pitch_rate = 0.0  # deg per 0.1s sample; explosive pitching would spike this
prev_pitch = None
reached_obstacle_t = None
crossed_obstacle_t = None
saw_nan = False

while vis.Run() and sys.GetChTime() < SIM_END:
    time = sys.GetChTime()

    # Terrain creation/release happens BEFORE render and stepping.
    if not terrain_created and time > time_create_terrain:
        terrain_z = robot.GetWheelPos(robosimian.FR).z - 0.15
        ground, height_under = create_terrain(sys, terrain_z)
        set_contact_properties(robot)
        vis.BindItem(ground)  # ensure the rendered floor is the contacted body
        robot.GetChassisBody().SetFixed(False)
        # Copy, don't alias: GetChassisPos() returns a live reference that tracks
        # the body, which would make every displacement read exactly zero.
        p0 = robot.GetChassisPos()
        start_pos = chrono.ChVector3d(p0.x, p0.y, p0.z)
        start_z = p0.z
        terrain_created = True
        region = (f"{TERRAIN_MODE} at x=[{REGION_START:.2f}, {REGION_END:.2f}], "
                  if REGION_START is not None else "")
        print(f"t={time:.2f}  terrain created ({TERRAIN_MODE}), floor top z={terrain_z:.3f}, "
              f"{region}chassis released")

    # Render at ~60fps, not every 1ms physics step; chase camera follows the chassis.
    if step % render_steps == 0:
        p = robot.GetChassisPos()
        vis.SetCameraPosition(p + cam_offset)
        vis.SetCameraTarget(p)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    robot.DoStepDynamics(time_step)
    step += 1

    if terrain_created and step % 100 == 0:
        pos = robot.GetChassisPos()
        if any(math.isnan(v) for v in (pos.x, pos.y, pos.z)):
            saw_nan = True
            print(f"t={time:.2f}  NaN chassis position -- aborting")
            break
        if time >= stats_t0:
            # Clearance vs LOCAL terrain height under each wheel, not constant z.
            wheel_xs = []
            for w in WHEEL_IDS.values():
                wp = robot.GetWheelPos(w)
                wheel_xs.append(wp.x)
                c = wp.z - height_under(wp.x)
                clear_min = min(clear_min, c)
                clear_max = max(clear_max, c)
            if REGION_START is not None:
                # Face contact, not center-past-edge: a stalled wheel pressing the
                # step never gets its center past REGION_START, so that test can
                # never fire in exactly the failure case it should report.
                if reached_obstacle_t is None and max(wheel_xs) > REGION_START - WHEEL_RADIUS:
                    reached_obstacle_t = time
                    print(f"t={time:.2f}  front wheel reached the {TERRAIN_MODE}")
                if crossed_obstacle_t is None and min(wheel_xs) > REGION_END:
                    crossed_obstacle_t = time
                    print(f"t={time:.2f}  all wheels cleared the {TERRAIN_MODE}")
            n = contact_container.GetNumContacts()
            contacts_min = min(contacts_min, n)
            contacts_max = max(contacts_max, n)
            # Tilt: chassis local -Z is "up" for this model (local +Z points at the
            # belly). Pitch is the up-vector's lean along x (positive = nose up).
            up = robot.GetChassisRot().Rotate(chrono.ChVector3d(0, 0, -1))
            max_tilt_deg = max(max_tilt_deg, math.degrees(math.acos(max(-1.0, min(1.0, up.z)))))
            pitch = math.degrees(math.atan2(-up.x, up.z))
            pitch_min_deg = min(pitch_min_deg, pitch)
            pitch_max_deg = max(pitch_max_deg, pitch)
            if prev_pitch is not None:
                max_pitch_rate = max(max_pitch_rate, abs(pitch - prev_pitch))
            prev_pitch = pitch

    if terrain_created and step % 2000 == 0:
        pos = robot.GetChassisPos()
        dx, dz = pos.x - start_pos.x, pos.z - start_z
        print(f"t={time:.2f}  displacement dx={dx:+.3f} dz={dz:+.3f} "
              f"pitch={prev_pitch if prev_pitch is not None else 0:+.2f}deg  "
              f"contacts={contact_container.GetNumContacts()}  avg_speed={cbk.GetAvgSpeed():.4f}")

# ---- End-of-run report: the stage-3 pass criteria, measured, in one place ----
pos = robot.GetChassisPos()
elapsed = sys.GetChTime() - stats_t0
dx, dy, dz = pos.x - start_pos.x, pos.y - start_pos.y, pos.z - start_z
planar = math.hypot(dx, dy)
print(f"\n==== RUN REPORT [{MODE} / {TERRAIN_MODE}] t={sys.GetChTime():.1f}s ====")
print(f"final displacement:   dx={dx:+.3f}  dy={dy:+.3f}  planar={planar:.3f} m")
print(f"reached obstacle:     {reached_obstacle_t is not None}"
      + (f" (front wheel at t={reached_obstacle_t:.1f}s)" if reached_obstacle_t else ""))
print(f"crossed obstacle:     {crossed_obstacle_t is not None}"
      + (f" (all wheels clear at t={crossed_obstacle_t:.1f}s)" if crossed_obstacle_t else ""))
print(f"average speed:        {planar / elapsed if elapsed > 0 else 0:.4f} m/s (planar)  "
      f"driver avg: {cbk.GetAvgSpeed():.4f}  driver distance: {cbk.GetDistance():.4f}")
print(f"max chassis tilt:     {max_tilt_deg:.2f} deg   "
      f"pitch range: {pitch_min_deg:+.2f} to {pitch_max_deg:+.2f} deg   "
      f"max pitch rate: {max_pitch_rate:.2f} deg/0.1s")
print(f"wheel clearance:      min={clear_min:+.3f}  max={clear_max:+.3f} m vs local "
      f"terrain (touching ~ +0.118; brief ~ +0.04 at obstacle edges is expected; "
      f"clearly negative = sinking)")
print(f"contact count range:  {contacts_min:.0f} - {contacts_max:.0f}")
print(f"NaNs occurred:        {saw_nan}")
print("RoboSimian obstacle simulation complete.")
