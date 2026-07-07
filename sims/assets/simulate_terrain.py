"""RoboSimian practice terrain in PyChrono.

Loads the Blender-exported terrain and obstacle meshes as fixed collision
bodies and drops a placeholder robot chassis at the flat start zone.

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


def build_system():
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.005)
    chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.005)

    for fname, props in MESHES.items():
        path = os.path.join(ASSETS, fname)
        if not os.path.isfile(path):
            raise FileNotFoundError(path)
        add_static_mesh(system, path, props["color"], props["friction"])

    # Placeholder robot: a chassis-sized box dropped at the start zone.
    # Replace with the actual RoboSimian model (bodies + joints) later.
    robot_mat = chrono.ChContactMaterialNSC()
    robot_mat.SetFriction(0.8)
    robot = chrono.ChBodyEasyBox(0.8, 0.6, 0.3, 500, True, True, robot_mat)
    robot.SetPos(START_POS)
    robot.GetVisualShape(0).SetColor(chrono.ChColor(0.1, 0.1, 0.12))
    system.Add(robot)

    return system, robot


def run_headless(system, robot, duration=3.0, step=2e-3):
    t = 0.0
    while t < duration:
        system.DoStepDynamics(step)
        t += step
        if abs(t % 0.5) < step:
            p = robot.GetPos()
            print(f"t={t:4.1f}s  robot pos=({p.x:6.2f}, {p.y:6.2f}, {p.z:6.2f})")
    print("headless run finished")


def run_visual(system, robot, step=2e-3):
    import pychrono.irrlicht as chronoirr

    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 800)
    vis.SetWindowTitle("RoboSimian practice terrain")
    vis.Initialize()
    vis.AddTypicalLights()
    vis.AddCamera(chrono.ChVector3d(6, -14, 6), chrono.ChVector3d(0, -2, 0))

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
