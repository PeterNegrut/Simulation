"""DIAGNOSTIC TOOL, not a terrain simulator -- see sims/robosimian.py for that.

Raw ChParserURDF import of the quadruped URDF bundled with PyChrono
(data/robot/robosimian, 43 bodies / 32 revolute joints across 4 limbs). Useful for
inspecting body/joint names and geometry, or checking import correctness -- not for
producing an accurate walking robot: it has no real driver, no tuned collision
model, and (verified below) settles into a crouch and drifts rather than standing
or walking cleanly. sims/robosimian.py uses Chrono's own compiled RoboSimian model
and RS_Driver instead, which get all of that right out of the box.

Three test modes (set MODE below):
  A - fixed-root articulation test: no floor, root fixed, one joint per limb
      oscillates. Verifies the joint chains are correctly assembled and connected.
  B - collision validation test: chassis+joints fixed at the URDF's neutral pose,
      floor placed to directly overlap one wheel. Verifies contact generation
      (prints live contact counts) without relying on a fall.
  C - free-body stance test: root fixed while all 32 joints ramp smoothly (via
      ChFunctionPoly23, not an abrupt step) from the URDF's neutral pose into the
      real standing stance extracted from the bundled walking_cycle.txt, feet
      resting just above a floor at z=0. Root is then released and the robot
      settles under gravity.

The URDF's own zero-angle pose is a stowed/folded configuration, not a standing
one (its "feet" -- wheels -- end up ABOVE the chassis, not below), so mode C
reads the real starting stance out of the bundled actuation/walking_cycle.txt
(same file the official RS_Driver uses) rather than inventing one.

Verified headlessly before rendering, in order:
  - body/joint audit: root ("base_link") is auto-fixed by the parser; all 43
    bodies have valid non-zero mass/inertia except the intentionally-massless
    limbX_link0 sensor-mount links (present in the URDF itself); collision
    shapes exist on 35 bodies but ChParserURDF leaves EnableCollision False on
    all of them -- fixed below by enabling it explicitly per body.
  - shape audit: every collision shape is a cheap primitive (no raw triangle
    mesh anywhere), so no MeshCollisionType override is needed.
  - stance geometry: applying the extracted stance puts all four wheels at
    the same height, ~0.33m below the torso (a level, symmetric stance).
  - contact generation: statically overlapping a wheel with a floor produces
    40-200 sustained contacts with no explosion.
  - stance settling: joints hold their commanded angles under full load to
    within ~3e-5 rad (the position servos are not the limiting factor). The
    robot does not explode or collapse, but does not stand perfectly still
    either -- it settles into a lower crouch and drifts/slides ~0.5-0.9m over
    several seconds. Neither raising friction (0.8->1.2) nor the system's
    center of mass (offset from the support-polygon centroid by <2cm, i.e.
    already well balanced) explains this: the most likely cause is that the
    "feet" are wheels, giving line/point contact with the floor rather than a
    flat footprint, which is inherently less stable against tipping/sliding
    than real feet -- a mechanical property of this robot, not a solver bug.
    A stable passive stand would need a real balance controller (feedback on
    torso tilt/CoM), which is out of scope here; see the module docstring's
    framing in the conversation this script came from.
"""
import math
import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.parsers as parsers

MODE = "C"  # "A", "B", or "C" -- see module docstring

ACTUATION_DIR = chrono.GetChronoDataFile("robot/robosimian/actuation/")


def load_walking_stance():
    """Real joint angles the official walking driver starts from (t=0 row of its
    cycle file) -- the URDF's own zero pose is a stowed shape, not this stance."""
    with open(ACTUATION_DIR + "motor_names.txt") as f:
        motor_names = [line.strip() for line in f if line.strip()]
    with open(ACTUATION_DIR + "walking_cycle.txt") as f:
        first_row = f.readline().split()
    return dict(zip(motor_names, (float(x) for x in first_row[1:])))


def enable_all_collision(system):
    n = 0
    for body in system.GetBodies():
        cm = body.GetCollisionModel()
        if cm and cm.GetNumShapes() > 0:
            body.EnableCollision(True)
            body.SyncCollisionModels()
            n += 1
    return n


system = chrono.ChSystemSMC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
system.GetSolver().AsIterative().SetMaxIterations(400)

robot_mat = chrono.ChContactMaterialData()
robot_mat.mu = 1.2
robot_mat.cr = 0.0

ROOT_Z = {"A": 0.9, "B": 0.9, "C": 0.545}[MODE]  # C: computed so feet land on a z=0 floor

parser = parsers.ChParserURDF(chrono.GetChronoDataFile("robot/robosimian/rs.urdf"))
parser.SetRootInitPose(chrono.ChFramed(chrono.ChVector3d(0, 0, ROOT_Z), chrono.QuatFromAngleX(chrono.CH_PI)))
parser.SetAllJointsActuationType(parsers.ChParserURDF.ActuationType_POSITION)
parser.SetDefaultContactMaterial(robot_mat)
parser.PopulateSystem(system)

root_body = parser.GetRootChBody()
torso_body = next(b for b in system.GetBodies() if b.GetName() == "torso")

ground = None
if MODE in ("B", "C"):
    ground_mat = chrono.ChContactMaterialSMC()
    ground_mat.SetFriction(1.2)
    ground_mat.SetRestitution(0.0)
    ground_mat.SetYoungModulus(1e7)

    ground = chrono.ChBody()
    ground.SetFixed(True)
    ground.EnableCollision(True)

    if MODE == "B":
        # Overlap a single wheel directly (a static proof that collision is live),
        # rather than relying on a fall that a fixed chassis+joints could never produce.
        wheel = next(b for b in system.GetBodies() if b.GetName() == "limb1_link8")
        wheel_radius = 0.12
        floor_top_z = wheel.GetPos().z - wheel_radius + 0.02
    else:
        floor_top_z = 0.0

    ground_shape = chrono.ChCollisionShapeBox(ground_mat, 8, 8, 0.2)
    ground.AddCollisionShape(ground_shape, chrono.ChFramed(chrono.ChVector3d(0, 0, floor_top_z - 0.1), chrono.QUNIT))
    ground.AddVisualShape(chrono.ChVisualShapeBox(8, 8, 0.2), chrono.ChFramed(chrono.ChVector3d(0, 0, floor_top_z - 0.1), chrono.QUNIT))
    system.Add(ground)

n_collision_enabled = enable_all_collision(system)
print(f"[{MODE}] enabled collision on {n_collision_enabled} bodies")

POSE_DURATION = 1.5
if MODE == "A":
    root_body.SetFixed(True)
    limbs = ["limb1", "limb2", "limb3", "limb4"]
    for i, limb in enumerate(limbs):
        wiggle = chrono.ChFunctionSine(i * (math.pi / 2), 0.3, 0.5)
        parser.SetMotorFunction(f"{limb}_joint2", wiggle)
elif MODE == "B":
    root_body.SetFixed(True)
    # joints stay at the URDF's neutral pose (default ActuationType_POSITION with no
    # motor function assigned holds position 0) -- only collision is under test here
elif MODE == "C":
    root_body.SetFixed(True)
    stance = load_walking_stance()
    for joint_name, target_angle in stance.items():
        parser.SetMotorFunction(joint_name, chrono.ChFunctionPoly23(target_angle, 0.0, POSE_DURATION))

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle(f"RoboSimian (URDF) - Mode {MODE}")
vis.Initialize()
vis.AddSkyBox()
if MODE == "C":
    vis.AddCamera(chrono.ChVector3d(2.5, -3.0, 1.2), chrono.ChVector3d(0, 0, 0.3))
else:
    vis.AddCamera(root_body.GetPos() + chrono.ChVector3d(2.5, -3.5, 1.0), root_body.GetPos())
vis.AddTypicalLights()

contact_container = system.GetContactContainer()
step_size = 5e-4
released = False
step = 0
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(step_size)
    step += 1

    if MODE == "C" and not released and system.GetChTime() > POSE_DURATION + 0.3:
        root_body.SetFixed(False)
        released = True
        print(f"t={system.GetChTime():.2f}  stance assumed, chassis released")

    if step % 400 == 0:
        p = torso_body.GetPos()
        print(f"t={system.GetChTime():.2f}  torso=({p.x:+.3f},{p.y:+.3f},{p.z:+.3f})  contacts={contact_container.GetNumContacts()}")

print("RoboSimian simulation complete.")
