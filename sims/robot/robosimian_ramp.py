"""RoboSimian, stage-2 terrain: flat lead-in + shallow 5-degree rigid ramp,
drive mode. Copied from the validated flat-ground baseline sims/robot/robosimian.py
(which stays untouched as the known-good reference); only the terrain logic and
the terrain-aware diagnostics differ.

Terrain: a flat box from behind the robot to RAMP_START_X, then a 5m box ramp
rotated about Y so its top surface rises at RAMP_SLOPE going +x. Both are plain
Chrono collision primitives with visual shapes built from the same dims/frames
(no Blender meshes yet -- procedural stages first: flat -> ramp -> single box
obstacle -> repeated bumps -> random boxes -> mesh visual w/ simple collision).

Geometry constraint: drive mode covers ~1.36m of ground in 30s, so the ramp
must start within ~1m of the robot or it is never reached inside SIM_END. With
RAMP_START_X=1.0 the front wheels (x ~ +0.46) hit the ramp around t=13-14s and
climb the last ~0.8m of the run uphill.

Wheel clearance is measured against the LOCAL terrain height under each wheel
(flat plane before RAMP_START_X, ramp plane equation after), not a constant
terrain z -- on any non-flat terrain a fixed reference would misread climbing
as floating. On the 5-degree slope the correct touching clearance is
wheel_radius/cos(5 deg) ~ +0.119, barely distinguishable from flat's +0.118.

Pass criteria (checked in the end-of-run report): robot reaches the ramp;
contacts stay nonzero; no NaNs; clearance stays at touching level (no
hovering/sinking); measurable uphill progress (chassis z gain past the ramp
start); chassis pitch changes smoothly to ~5 deg, not explosively.
"""
import math
import os
import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.robot as robosimian

MODE = "drive"        # stage 2 is drive-mode only; gaits come back on later stages
TERRAIN_MODE = os.environ.get("RS_TERRAIN", "ramp")  # "flat" or "ramp"
SIM_END = 30.0
SHOW_COLLISION = False  # wireframe collision-shape overlay (debugging)
SHOW_CONTACTS = False   # draw contact normals (debugging)

RAMP_SLOPE = math.radians(5)
RAMP_START_X = 1.0    # where the flat lead-in ends and the ramp top surface begins
RAMP_LENGTH = 5.0     # along the slope
TERRAIN_WIDTH = 3.0
LEADIN_LENGTH = 4.0   # flat box from x = RAMP_START_X - LEADIN_LENGTH to RAMP_START_X
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
    """Flat lead-in ending at RAMP_START_X plus (in ramp mode) a rigid box ramp
    rotated about Y, top surfaces meeting exactly at (RAMP_START_X, terrain_z).
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

    if TERRAIN_MODE == "flat":
        # Flat fallback must extend well PAST the ramp-start line, or the robot
        # drives off the edge mid-run (validated: it falls, tumbling, ~1200m).
        flat_frame = chrono.ChFramed(
            chrono.ChVector3d(2, 0, terrain_z - THICKNESS / 2), chrono.QUNIT)
        add_box((8, TERRAIN_WIDTH, THICKNESS), flat_frame)
    else:
        leadin_frame = chrono.ChFramed(
            chrono.ChVector3d(RAMP_START_X - LEADIN_LENGTH / 2, 0, terrain_z - THICKNESS / 2),
            chrono.QUNIT)
        add_box((LEADIN_LENGTH, TERRAIN_WIDTH, THICKNESS), leadin_frame)

    ramp_end_x = RAMP_START_X
    if TERRAIN_MODE == "ramp":
        # Rotate about Y so the box's local +x rises going +x world.
        q_ramp = chrono.QuatFromAngleY(-RAMP_SLOPE)
        # Place the center so the top surface's leading edge (local
        # (-L/2, 0, +T/2)) lands exactly at (RAMP_START_X, 0, terrain_z).
        edge_local = chrono.ChVector3d(-RAMP_LENGTH / 2, 0, THICKNESS / 2)
        center = chrono.ChVector3d(RAMP_START_X, 0, terrain_z) - q_ramp.Rotate(edge_local)
        add_box((RAMP_LENGTH, TERRAIN_WIDTH, THICKNESS), chrono.ChFramed(center, q_ramp))
        ramp_end_x = RAMP_START_X + RAMP_LENGTH * math.cos(RAMP_SLOPE)

    sys.GetCollisionSystem().BindItem(ground)
    sys.AddBody(ground)

    def height_under(x):
        """Terrain top-surface height at world x (assumes |y| < TERRAIN_WIDTH/2)."""
        if TERRAIN_MODE == "ramp" and x > RAMP_START_X:
            return terrain_z + math.tan(RAMP_SLOPE) * (min(x, ramp_end_x) - RAMP_START_X)
        return terrain_z

    return ground, height_under


def set_contact_properties(robot):
    friction, restitution, young_modulus = 0.8, 0.0, 1e7
    for mat in (robot.GetSledContactMaterial(), robot.GetWheelContactMaterial()):
        mat.SetFriction(friction)
        mat.SetRestitution(restitution)
        chrono.CastToChContactMaterialSMC(mat).SetYoungModulus(young_modulus)


if TERRAIN_MODE not in ("flat", "ramp"):
    raise SystemExit(f"FATAL: unknown TERRAIN_MODE {TERRAIN_MODE!r}; use 'flat' or 'ramp'")

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
driver = DriverClass(
    chrono.GetChronoDataFile(actuation_dir + "driving_start.txt"),
    chrono.GetChronoDataFile(actuation_dir + "driving_cycle.txt"),
    chrono.GetChronoDataFile(actuation_dir + "driving_stop.txt"),
    True)
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
reached_ramp_t = None
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
        print(f"t={time:.2f}  terrain created ({TERRAIN_MODE}), flat top z={terrain_z:.3f}, "
              f"ramp starts x={RAMP_START_X}, chassis released")

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
            for w in WHEEL_IDS.values():
                wp = robot.GetWheelPos(w)
                c = wp.z - height_under(wp.x)
                clear_min = min(clear_min, c)
                clear_max = max(clear_max, c)
            if reached_ramp_t is None and TERRAIN_MODE == "ramp" and \
                    max(robot.GetWheelPos(w).x for w in WHEEL_IDS.values()) > RAMP_START_X:
                reached_ramp_t = time
                print(f"t={time:.2f}  front wheel crossed onto the ramp")
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

    if terrain_created and step % 1000 == 0:
        pos = robot.GetChassisPos()
        dx, dy, dz = pos.x - start_pos.x, pos.y - start_pos.y, pos.z - start_z
        print(f"t={time:.2f}  displacement dx={dx:+.3f} dz={dz:+.3f} "
              f"pitch={prev_pitch if prev_pitch is not None else 0:+.2f}deg  "
              f"contacts={contact_container.GetNumContacts()}  avg_speed={cbk.GetAvgSpeed():.4f}")

# ---- End-of-run report: the stage-2 pass criteria, measured, in one place ----
pos = robot.GetChassisPos()
elapsed = sys.GetChTime() - stats_t0
dx, dy, dz = pos.x - start_pos.x, pos.y - start_pos.y, pos.z - start_z
planar = math.hypot(dx, dy)
print(f"\n==== RUN REPORT [{MODE} / {TERRAIN_MODE}] t={sys.GetChTime():.1f}s ====")
print(f"final displacement:   dx={dx:+.3f}  dy={dy:+.3f}  planar={planar:.3f} m")
print(f"uphill progress:      chassis z gain {dz:+.3f} m "
      f"(expect > 0 once past ramp start at 5 deg)")
print(f"reached ramp:         {reached_ramp_t is not None}"
      + (f" (front wheel crossed at t={reached_ramp_t:.1f}s)" if reached_ramp_t else ""))
print(f"average speed:        {planar / elapsed if elapsed > 0 else 0:.4f} m/s (planar)  "
      f"driver avg: {cbk.GetAvgSpeed():.4f}  driver distance: {cbk.GetDistance():.4f}")
print(f"max chassis tilt:     {max_tilt_deg:.2f} deg   "
      f"pitch range: {pitch_min_deg:+.2f} to {pitch_max_deg:+.2f} deg   "
      f"max pitch rate: {max_pitch_rate:.2f} deg/0.1s")
print(f"wheel clearance:      min={clear_min:+.3f}  max={clear_max:+.3f} m vs local "
      f"terrain (touching ~ +0.118 flat, +0.119 on slope)")
print(f"contact count range:  {contacts_min:.0f} - {contacts_max:.0f}")
print(f"NaNs occurred:        {saw_nan}")
print("RoboSimian ramp simulation complete.")
