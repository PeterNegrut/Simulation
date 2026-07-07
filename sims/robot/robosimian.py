"""RoboSimian on rigid flat terrain -- Chrono's compiled robot model with its
bundled actuation files, in one of four selectable locomotion modes.

MODE (env var RS_MODE or edit below):
  drive    -- wheel-driven, rolls like a vehicle; the default. Visibly traverses
              ground in seconds, so use it to prove the terrain/contact pipeline
              before trusting the slower gaits.
  walk     -- statically-stable crawl. Accurate but glacial: one gait cycle is
              19.16s and nets ~0.2m (~0.01 m/s), with long weight-shift pauses.
              Judge it over 25-40 simulated seconds minimum.
  scull    -- sculling gait (sculling_cycle2.txt).
  inchworm -- inchworming gait.

Verification: SHOW_COLLISION overlays wireframes of the actual collision
geometry (robot AND terrain); SHOW_CONTACTS draws live contact normals. The
wheel-clearance log is the ground truth for contact: ~+0.118m (one wheel
radius, i.e. touching) is correct; +0.5m means floating, strongly negative
means sinking. An end-of-run report prints displacement, speed, tilt,
clearance range, contact range, and a NaN check.

Driver note: this PyChrono build (9.0.1) ships the driver as RS_Driver; newer
Chrono (9.1+) renamed the same class to ChRobotActuation. The compat shim
below picks whichever exists.

Terrain stages (add ONLY after flat ground passes in drive mode):
flat rigid box (current) -> shallow ramp -> single low box obstacle ->
repeated low bumps -> uneven field of many small boxes.

Sequence: robot assumes its initial pose (root fixed, no terrain yet) ->
settles briefly -> terrain is created under its feet and the chassis is
released -> the mode's actuation cycle takes over until SIM_END.
"""
import math
import os
import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.robot as robosimian

MODE = os.environ.get("RS_MODE", "drive")  # "walk", "drive", "scull", "inchworm"
SIM_END = 30.0        # simulated seconds; walk mode needs 25-40s to show a full cycle
SHOW_COLLISION = False  # wireframe collision-shape overlay (debugging)
SHOW_CONTACTS = False   # draw contact normals (debugging)

ACTUATION = {
    "walk": ("", "walking_cycle.txt", ""),
    "drive": ("driving_start.txt", "driving_cycle.txt", "driving_stop.txt"),
    "scull": ("sculling_start.txt", "sculling_cycle2.txt", "sculling_stop.txt"),
    "inchworm": ("inchworming_start.txt", "inchworming_cycle.txt", "inchworming_stop.txt"),
}

duration_pose = 1.0          # time to assume the initial pose before terrain exists
duration_settle_robot = 0.5  # time to let it settle on terrain before actuation starts
time_create_terrain = duration_pose

WHEEL_IDS = {"FR": robosimian.FR, "RR": robosimian.RR,
             "RL": robosimian.RL, "FL": robosimian.FL}


def create_terrain(sys, length, width, height, offset):
    """Rigid box terrain whose top surface sits at z=height."""
    ground_mat = chrono.ChContactMaterial.DefaultMaterial(sys.GetContactMethod())
    ground_mat.SetFriction(0.8)
    ground_mat.SetRestitution(0)
    chrono.CastToChContactMaterialSMC(ground_mat).SetYoungModulus(1e7)

    ground = chrono.ChBody()
    ground.SetFixed(True)
    ground.EnableCollision(True)
    ground_shape = chrono.ChCollisionShapeBox(ground_mat, length, width, 0.2)
    frame = chrono.ChFramed(chrono.ChVector3d(offset, 0, height - 0.1), chrono.QUNIT)
    ground.AddCollisionShape(ground_shape, frame)
    sys.GetCollisionSystem().BindItem(ground)

    box = chrono.ChVisualShapeBox(length, width, 0.2)
    box.SetTexture(chrono.GetChronoDataFile("textures/pinkwhite.png"), 10 * length, 10 * width)
    ground.AddVisualShape(box, frame)
    sys.AddBody(ground)
    return ground


def set_contact_properties(robot):
    friction, restitution, young_modulus = 0.8, 0.0, 1e7
    for mat in (robot.GetSledContactMaterial(), robot.GetWheelContactMaterial()):
        mat.SetFriction(friction)
        mat.SetRestitution(restitution)
        chrono.CastToChContactMaterialSMC(mat).SetYoungModulus(young_modulus)


if MODE not in ACTUATION:
    raise SystemExit(f"FATAL: unknown MODE {MODE!r}; pick one of {sorted(ACTUATION)}")

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

print(f"Using Chrono RoboSimian compiled model, mode={MODE}")
print("Driver:", type(driver))
print("Robot chassis fixed initially:", robot.GetChassisBody().IsFixed())
if type(driver).__name__ not in ("RS_Driver", "ChRobotActuation"):
    raise SystemExit("FATAL: unexpected driver class -- want RS_Driver/ChRobotActuation")
if not robot.GetChassisBody().IsFixed():
    raise SystemExit("FATAL: chassis must be fixed during the pose phase")

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle(f"RoboSimian - Rigid terrain [{MODE}]")
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
terrain_z = None
start_pos = None
step = 0

# End-of-run stats (collected every 0.1s once the robot has settled on terrain)
stats_t0 = time_create_terrain + duration_settle_robot
clear_min, clear_max = float("inf"), float("-inf")
contacts_min, contacts_max = float("inf"), float("-inf")
max_tilt_deg = 0.0
saw_nan = False

while vis.Run() and sys.GetChTime() < SIM_END:
    time = sys.GetChTime()

    # Terrain creation/release happens BEFORE render and stepping.
    if not terrain_created and time > time_create_terrain:
        terrain_z = robot.GetWheelPos(robosimian.FR).z - 0.15
        length, width = 8, 2
        ground = create_terrain(sys, length, width, terrain_z, length / 4)
        set_contact_properties(robot)
        vis.BindItem(ground)  # ensure the rendered floor is the contacted body
        robot.GetChassisBody().SetFixed(False)
        # Copy, don't alias: GetChassisPos() returns a live reference that tracks
        # the body, which would make every displacement read exactly zero.
        p0 = robot.GetChassisPos()
        start_pos = chrono.ChVector3d(p0.x, p0.y, p0.z)
        terrain_created = True
        print(f"t={time:.2f}  terrain created, top at z={terrain_z:.3f}, chassis released")

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
            clearances = [robot.GetWheelPos(w).z - terrain_z for w in WHEEL_IDS.values()]
            clear_min = min(clear_min, *clearances)
            clear_max = max(clear_max, *clearances)
            n = contact_container.GetNumContacts()
            contacts_min = min(contacts_min, n)
            contacts_max = max(contacts_max, n)
            # Tilt: chassis local -Z is "up" for this model (local +Z points at the
            # belly); angle of that axis off world +Z combines roll and pitch.
            up = robot.GetChassisRot().Rotate(chrono.ChVector3d(0, 0, -1))
            max_tilt_deg = max(max_tilt_deg, math.degrees(math.acos(max(-1.0, min(1.0, up.z)))))

    if terrain_created and step % 1000 == 0:
        pos = robot.GetChassisPos()
        dx, dy = pos.x - start_pos.x, pos.y - start_pos.y
        planar = math.hypot(dx, dy)
        print(f"t={time:.2f}  displacement dx={dx:+.3f} dy={dy:+.3f} planar={planar:.3f}  "
              f"contacts={contact_container.GetNumContacts()}  avg_speed={cbk.GetAvgSpeed():.4f}")

# ---- End-of-run report: the success criteria, measured, in one place ----
pos = robot.GetChassisPos()
elapsed = sys.GetChTime() - stats_t0
dx, dy = pos.x - start_pos.x, pos.y - start_pos.y
planar = math.hypot(dx, dy)
print(f"\n==== RUN REPORT [{MODE}] t={sys.GetChTime():.1f}s ====")
print(f"final displacement:   dx={dx:+.3f}  dy={dy:+.3f}  planar={planar:.3f} m")
print(f"average speed:        {planar / elapsed if elapsed > 0 else 0:.4f} m/s (planar)  "
      f"driver avg: {cbk.GetAvgSpeed():.4f}  driver distance: {cbk.GetDistance():.4f}")
print(f"max chassis tilt:     {max_tilt_deg:.2f} deg")
print(f"wheel clearance:      min={clear_min:+.3f}  max={clear_max:+.3f} m "
      f"(touching ~ +0.118; floating >> that; sinking << 0)")
print(f"contact count range:  {contacts_min:.0f} - {contacts_max:.0f}")
print(f"NaNs occurred:        {saw_nan}")
print("RoboSimian simulation complete.")
