"""RoboSimian on rigid flat terrain -- reproduces Chrono's official
demo_ROBOT_RoboSimian_Rigid demo using Chrono's compiled robot model and its
bundled walking-cycle driver.

Use SHOW_COLLISION and the wheel-clearance logs to verify contact before
modifying terrain: the collision-geometry render shows the shapes physics
actually uses (not the pretty mesh), and the clearance log shows each wheel's
height above the terrain top. Trust those two signals, not appearances.

Driver note: this PyChrono build (9.0.1) ships the driver as RS_Driver; newer
Chrono (9.1+) renamed the same class to ChRobotActuation. The startup check
below accepts either name and aborts on anything else.

Sequence: robot assumes its initial pose (root fixed, no terrain yet) ->
settles briefly -> terrain is created under its feet and the chassis is
released -> the walking-cycle driver (actuation/walking_cycle.txt) takes over.
"""
import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.robot as robosimian

# Overlay wireframe collision shapes + contact normals on the render. This build
# (PyChrono 9.0.1) cannot use RoboSimian.SetVisualizationType*(COLLISION): the SWIG
# wrapper exposes the setters but not the robosimian VisualizationType enum values
# they require, so they are uncallable. Irrlicht's EnableCollisionShapeDrawing is
# strictly better for verification anyway: it draws the *actual* collision geometry
# of every body (robot AND terrain), and EnableContactDrawing shows live contacts.
SHOW_COLLISION = True

duration_pose = 1.0          # time to assume the initial pose before terrain exists
duration_settle_robot = 0.5  # time to let it settle on terrain before walking starts
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


sys = chrono.ChSystemSMC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.GetSolver().AsIterative().SetMaxIterations(200)
sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.8))

robot = robosimian.RoboSimian(sys, True, True)
robot.Initialize(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI)))

driver = robosimian.RS_Driver(
    "",
    chrono.GetChronoDataFile("robot/robosimian/actuation/walking_cycle.txt"),
    "",
    True)
cbk = robosimian.RS_DriverCallback(robot)
driver.RegisterPhaseChangeCallback(cbk)
driver.SetTimeOffsets(duration_pose, duration_settle_robot)
robot.SetDriver(driver)

# Hard API-path check: fail loudly rather than simulate a subtly wrong setup.
print("Using Chrono RoboSimian compiled model")
print("Driver:", type(driver))
print("Robot chassis fixed initially:", robot.GetChassisBody().IsFixed())
if type(driver).__name__ not in ("RS_Driver", "ChRobotActuation"):
    raise SystemExit("FATAL: unexpected driver class -- want RS_Driver/ChRobotActuation")
if not robot.GetChassisBody().IsFixed():
    raise SystemExit("FATAL: chassis must be fixed during the pose phase")

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("RoboSimian - Rigid terrain")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1, -2.75, 0.2), chrono.ChVector3d(1, 0, 0))
vis.AddLight(chrono.ChVector3d(100, 100, 100), 290)
vis.AddLight(chrono.ChVector3d(100, -100, 80), 190)

if SHOW_COLLISION:
    vis.EnableCollisionShapeDrawing(True)
    vis.EnableContactDrawing(chronoirr.ContactsDrawMode_CONTACT_NORMALS)

contact_container = sys.GetContactContainer()
time_step = 1e-3
terrain_created = False
terrain_z = None
step = 0

while vis.Run():
    time = sys.GetChTime()

    # Terrain creation/release happens BEFORE render and stepping, matching the
    # official demo's ordering.
    if not terrain_created and time > time_create_terrain:
        terrain_z = robot.GetWheelPos(robosimian.FR).z - 0.15
        length, width = 8, 2
        ground = create_terrain(sys, length, width, terrain_z, length / 4)
        set_contact_properties(robot)
        vis.BindItem(ground)  # ensure the rendered floor is the contacted body
        robot.GetChassisBody().SetFixed(False)
        terrain_created = True
        print(f"t={time:.2f}  terrain created, top at z={terrain_z:.3f}, chassis released")

    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    robot.DoStepDynamics(time_step)
    step += 1

    # Wheel-vs-terrain clearance every 0.1s: the ground-truth contact signal.
    # Expected once settled: clearance ~ wheel contact level (small, stable);
    # +0.5m means floating, strongly negative means sinking through the floor.
    if terrain_created and step % 100 == 0:
        clear = {name: robot.GetWheelPos(wid).z - terrain_z
                 for name, wid in WHEEL_IDS.items()}
        print(f"t={time:.2f}  clearance "
              + "  ".join(f"{n}={c:+.3f}" for n, c in clear.items())
              + f"  contacts={contact_container.GetNumContacts()}")

    if step % 1000 == 0:
        pos = robot.GetChassisPos()
        speed = cbk.GetAvgSpeed() if terrain_created else 0.0
        print(f"t={time:.2f}  chassis=({pos.x:+.3f},{pos.y:+.3f},{pos.z:+.3f})  "
              f"contacts={contact_container.GetNumContacts()}  avg_speed={speed:.4f}")

print(f"avg. speed: {cbk.GetAvgSpeed():.4f}  distance: {cbk.GetDistance():.4f}")
print("RoboSimian simulation complete.")
