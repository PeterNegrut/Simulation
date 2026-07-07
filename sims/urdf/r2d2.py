"""R2D2 robot: parses the URDF model bundled with PyChrono (data/robot/r2d2). The
base drives forward along a linear motor (with wheels spinning in sync) toward the
edge of the floor and stops there; the head keeps swiveling throughout."""
import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.parsers as parsers

FLOOR_LENGTH_X = 3.0
FLOOR_THICKNESS = 0.1
START_X = -1.0
TARGET_X = 1.0  # stops short of the floor edge (+1.5) so the gripper doesn't overhang
DRIVE_SPEED = 0.5  # m/s
WHEEL_RADIUS = 0.035
# Root height so the wheels (offset -0.435 from the base_link origin, per the URDF's
# own joint chain) rest on the floor surface instead of floating: floor top
# (FLOOR_THICKNESS / 2) + WHEEL_RADIUS - wheel_offset.
ROOT_Z = FLOOR_THICKNESS / 2 + WHEEL_RADIUS + 0.435

system = chrono.ChSystemSMC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

ground = chrono.ChBody()
ground.SetFixed(True)
ground_box = chrono.ChVisualShapeBox(FLOOR_LENGTH_X, 2, FLOOR_THICKNESS)
ground_box.SetTexture(chrono.GetChronoDataFile("textures/checker2.png"))
ground.AddVisualShape(ground_box)
system.Add(ground)

parser = parsers.ChParserURDF(chrono.GetChronoDataFile("robot/r2d2/r2d2.urdf"))
parser.SetRootInitPose(chrono.ChFramed(chrono.ChVector3d(START_X, 0, ROOT_Z), chrono.QUNIT))
parser.SetAllJointsActuationType(parsers.ChParserURDF.ActuationType_POSITION)
parser.PopulateSystem(system)

root_body = parser.GetRootChBody()

head_swivel = chrono.ChFunctionSine(0, 0.2, 1.0)
parser.SetMotorFunction("head_swivel", head_swivel)

# Drive the base forward along world X. The motor's own Z axis is the actuated
# axis, so the joint frame is rotated to point that axis along world X.
drive_distance = TARGET_X - START_X
drive_function = chrono.ChFunctionRamp(0, DRIVE_SPEED)
drive_motor = chrono.ChLinkMotorLinearPosition()
drive_motor.Initialize(ground, root_body, chrono.ChFramed(root_body.GetPos(), chrono.QuatFromAngleY(-chrono.CH_PI_2)))
drive_motor.SetMotionFunction(drive_function)
system.Add(drive_motor)

# Spin the wheels in sync with the drive speed (rolling, not sliding).
wheel_angular_speed = DRIVE_SPEED / WHEEL_RADIUS
wheel_joints = ["right_front_wheel_joint", "right_back_wheel_joint", "left_front_wheel_joint", "left_back_wheel_joint"]
wheel_functions = [chrono.ChFunctionRamp(0, wheel_angular_speed) for _ in wheel_joints]
for name, function in zip(wheel_joints, wheel_functions):
    parser.SetMotorFunction(name, function)

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("R2D2 (URDF)")
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, -4.5, 1.2), chrono.ChVector3d(0, 0, 0.5))
vis.AddTypicalLights()

step_size = 1e-3
realtime_timer = chrono.ChRealtimeStepTimer()
stopped = False
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(step_size)
    realtime_timer.Spin(step_size)

    if not stopped and drive_motor.GetMotorPos() >= drive_distance:
        drive_motor.SetMotionFunction(chrono.ChFunctionConst(drive_distance))
        for name in wheel_joints:
            current_angle = chrono.CastToChLinkMotorRotationAngle(parser.GetChMotor(name)).GetMotorAngle()
            parser.SetMotorFunction(name, chrono.ChFunctionConst(current_angle))
        stopped = True

print("R2D2 simulation complete.")
