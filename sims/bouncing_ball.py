"""Bouncing ball: a sphere with restitution dropped onto a floor, printing each bounce's apex height."""
import pychrono as chrono
import pychrono.irrlicht as chronoirr

system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

material = chrono.ChContactMaterialNSC()
material.SetRestitution(0.8)

floor = chrono.ChBodyEasyBox(20, 0.2, 20, 1000, True, True, material)
floor.SetPos(chrono.ChVector3d(0, 0, 0))
floor.SetFixed(True)
system.Add(floor)

ball = chrono.ChBodyEasySphere(0.3, 1000, True, True, material)
ball.SetPos(chrono.ChVector3d(0, 5, 0))
system.Add(ball)

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Bouncing Ball")
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(3, 4, -6), chrono.ChVector3d(0, 2, 0))
vis.AddTypicalLights()

dt = 0.002
rising = False
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(dt)

    vel_y = ball.GetPosDt().y
    if vel_y > 0:
        rising = True
    elif rising and vel_y <= 0:
        print(f"t={system.GetChTime():.2f}s  bounce apex height={ball.GetPos().y:.3f}")
        rising = False

print("Bouncing ball simulation complete.")
