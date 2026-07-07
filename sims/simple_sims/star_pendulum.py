"""Pendulum: a bob swings on a rigid rod from a fixed pivot, striking a vertically
facing star obstacle mid-swing."""
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

# Star obstacle: sits on the bob's circular swing path (radius = arm_length from
# the pivot) partway down the arc, so the bob strikes it mid-swing and bounces off.
#
# The source mesh (imported from Blender) is a star cut out flat in the X-Z plane,
# extruded a thin bit along Y -- like a cookie lying on a table. To make it stand
# upright and face the pendulum's X-Y swing plane (like a target), Y and Z are
# swapped so the thin extrusion runs along Z (depth) instead of Y (height), and the
# whole thing is scaled up and re-centered on its own origin so positioning the body
# places the star's center directly on the swing path.
star_scale = 5.5
star_mesh = chrono.ChTriangleMeshConnected_CreateFromWavefrontFile(
    os.path.join(SCRIPT_DIR, "assets", "star.obj"), False, False
)
star_rotscale = chrono.ChMatrix33d()
star_rotscale.SetFromDirectionAxes(
    chrono.ChVector3d(star_scale, 0, 0),
    chrono.ChVector3d(0, 0, star_scale),
    chrono.ChVector3d(0, star_scale, 0),
)
star_verts_raw = star_mesh.GetCoordsVertices()
star_center = chrono.ChVector3d(
    (min(v.x for v in star_verts_raw) + max(v.x for v in star_verts_raw)) / 2,
    (min(v.y for v in star_verts_raw) + max(v.y for v in star_verts_raw)) / 2,
    (min(v.z for v in star_verts_raw) + max(v.z for v in star_verts_raw)) / 2,
)
star_mesh.Transform(-(star_rotscale * star_center), star_rotscale)
star_verts = star_mesh.GetCoordsVertices()  # re-fetch: now scaled, rotated, and centered

swing_angle = math.radians(35)  # from vertical; 0 = bottom of the arc
star_pos = pivot_pos + chrono.ChVector3d(
    arm_length * math.sin(swing_angle), -arm_length * math.cos(swing_angle), 0
)

star = chrono.ChBody()
star.SetFixed(True)
star.SetPos(star_pos)

# A true concave collision mesh lets the bob wedge into the notches between the
# star's points instead of bouncing off. Use the convex hull of the same (already
# scaled/rotated) points as the collision proxy -- a robust, disc-like shape that
# still spans the star's full silhouette -- while the visual keeps the real points.
star_collision_shape = chrono.ChCollisionShapeConvexHull(contact_material, star_verts)
star.AddCollisionShape(star_collision_shape)
star.EnableCollision(True)

star_visual_shape = chrono.ChVisualShapeTriangleMesh(star_mesh)
star_visual_shape.SetColor(chrono.ChColor(0.95, 0.8, 0.1))
star.AddVisualShape(star_visual_shape)

system.Add(star)

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Star Pendulum")
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

print("Star pendulum simulation complete.")
