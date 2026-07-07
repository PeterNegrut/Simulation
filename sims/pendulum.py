"""Pendulum: a bob swings on a rigid rod from a fixed pivot, striking a cube obstacle mid-swing."""
import math
import os
import pychrono as chrono
import pychrono.irrlicht as chronoirr

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

contact_material = chrono.ChContactMaterialNSC()
contact_material.SetRestitution(0.7)

pivot_pos = chrono.ChVector3d(0, 5, 0)
arm_length = 2.0
bob_pos = pivot_pos + chrono.ChVector3d(arm_length, 0, 0)

anchor = chrono.ChBodyEasySphere(0.05, 1000)
anchor.SetPos(pivot_pos)
anchor.SetFixed(True)
system.Add(anchor)

bob = chrono.ChBodyEasySphere(0.15, 1000, True, True, contact_material)
bob.SetPos(bob_pos)
system.Add(bob)

joint = chrono.ChLinkLockRevolute()
joint.Initialize(anchor, bob, chrono.ChFramed(pivot_pos))
system.Add(joint)

# Rod: a real rigid body spanning the pivot and the bob, welded to the bob so the
# two move as a single rigid pendulum. Welding it to the fixed anchor instead would
# leave it static, since the anchor never moves.
rod = chrono.ChBodyEasyCylinder(chrono.ChAxis_X, 0.03, arm_length, 500)
rod.SetPos((pivot_pos + bob_pos) * 0.5)
rod.GetVisualShape(0).SetColor(chrono.ChColor(0.9, 0.1, 0.1))
system.Add(rod)

weld = chrono.ChLinkLockLock()
weld.Initialize(rod, bob, chrono.ChFramed(bob_pos))
system.Add(weld)

floor = chrono.ChBodyEasyBox(20, 0.2, 20, 1000)
floor.SetPos(chrono.ChVector3d(0, 0, 0))
floor.SetFixed(True)
system.Add(floor)

# Cube obstacle: sits on the bob's circular swing path (radius = arm_length from
# the pivot) partway down the arc, so the bob strikes it mid-swing and bounces off.
# The visual mesh (imported from Blender, a plain unit cube spanning -1..1) is
# scaled to match a physical box collision shape of the same size.
cube_size = 0.5
swing_angle = math.radians(35)  # from vertical; 0 = bottom of the arc
cube_pos = pivot_pos + chrono.ChVector3d(
    arm_length * math.sin(swing_angle), -arm_length * math.cos(swing_angle), 0
)

cube = chrono.ChBody()
cube.SetFixed(True)
cube.SetPos(cube_pos)

cube_collision_shape = chrono.ChCollisionShapeBox(contact_material, cube_size, cube_size, cube_size)
cube.AddCollisionShape(cube_collision_shape)
cube.EnableCollision(True)

cube_mesh = chrono.ChVisualShapeModelFile()
cube_mesh.SetFilename(os.path.join(SCRIPT_DIR, "assets", "cube.obj"))
cube_mesh.SetScale(chrono.ChVector3d(cube_size / 2, cube_size / 2, cube_size / 2))
cube.AddVisualShape(cube_mesh)

system.Add(cube)

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Pendulum")
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 6, -9), chrono.ChVector3d(0, 3, 0))
vis.AddTypicalLights()

dt = 0.005
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(dt)

print("Pendulum simulation complete.")
