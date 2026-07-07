"""Wheel-legged robot practice terrain in PyChrono.

Loads the Blender-exported terrain and obstacle meshes as fixed collision
bodies and drops the LimX WL_P311D wheel-legged quadruped (URDF import) at
the flat start zone. All joints are position-held at the URDF zero pose, so
the robot stands passively; wheel motors can be driven later via the parser
joint names (LF_WHL, RF_WHL, LH_WHL, RH_WHL).

Run visually:      python simulate_terrain.py
Headless check:    python simulate_terrain.py --headless
"""
import argparse
import os

import pychrono as chrono

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

START_POS = chrono.ChVector3d(0, -8, 1.0)  # above the flat start zone

ROBOT_DIR = os.path.join(ASSETS, "robots", "WL_P311D")
ROBOT_URDF = os.path.join(ROBOT_DIR, "urdf", "robot.urdf")
# package URI prefix used by the vendored LimX description
PACKAGE_PREFIX = "package://robot_description/wheellegged/WL_P311D"


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
    vendored mesh directory. The result embeds an absolute path, so it is
    regenerated (not committed) and only rewritten when its content changes."""
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
    import pychrono.parsers as parsers

    parser = parsers.ChParserURDF(resolved_urdf_path())
    parser.SetRootInitPose(chrono.ChFramed(START_POS, chrono.QUNIT))

    contact = chrono.ChContactMaterialData()
    contact.mu = 0.8
    parser.SetDefaultContactMaterial(contact)

    # Position-actuate every revolute joint with the default zero function:
    # the robot rigidly holds its URDF pose instead of collapsing.
    parser.SetAllJointsActuationType(parsers.ChParserURDF.ActuationType_POSITION)

    parser.PopulateSystem(system)

    # This parser build leaves EnableCollision off on every imported body
    # even though it created the collision shapes — switch them on.
    for body in system.GetBodies():
        model = body.GetCollisionModel()
        if not body.IsFixed() and model is not None and model.GetNumShapes() > 0:
            body.EnableCollision(True)

    return parser.GetRootChBody()


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.005)
    chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.005)
    system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    system.GetSolver().AsIterative().SetMaxIterations(150)

    for fname, props in MESHES.items():
        path = os.path.join(ASSETS, fname)
        if not os.path.isfile(path):
            raise FileNotFoundError(path)
        add_static_mesh(system, path, props["color"], props["friction"])

    robot = add_robot(system)
    return system, robot


def run_headless(system, robot, duration=3.0, step=1e-3):
    t = 0.0
    while t < duration:
        system.DoStepDynamics(step)
        t += step
        if abs(t % 0.5) < step:
            p = robot.GetPos()
            print(f"t={t:4.1f}s  robot pos=({p.x:6.2f}, {p.y:6.2f}, {p.z:6.2f})")
    p = robot.GetPos()
    assert p.x == p.x and p.z == p.z, "NaN in robot position"
    print(f"final robot pos=({p.x:6.2f}, {p.y:6.2f}, {p.z:6.2f})")
    print("headless run finished")


def run_visual(system, robot, step=1e-3):
    import pychrono.irrlicht as chronoirr

    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 800)
    vis.SetWindowTitle("WL_P311D practice terrain")
    vis.Initialize()
    vis.AddTypicalLights()
    vis.AddCamera(chrono.ChVector3d(4, -12, 3), chrono.ChVector3d(0, -6, 0.5))

    rt = chrono.ChRealtimeStepTimer()
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step)
        rt.Spin(step)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true", help="run without a window")
    args = parser.parse_args()

    system, robot = build_system()
    if args.headless:
        run_headless(system, robot)
    else:
        run_visual(system, robot)
