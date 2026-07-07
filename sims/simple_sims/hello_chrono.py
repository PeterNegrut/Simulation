"""Smoke test: drop a box under gravity and print its fall over a few timesteps."""
import pychrono as chrono

system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

box = chrono.ChBodyEasyBox(1, 1, 1, 1000)
box.SetPos(chrono.ChVector3d(0, 5, 0))
system.Add(box)

floor = chrono.ChBodyEasyBox(20, 0.2, 20, 1000)
floor.SetPos(chrono.ChVector3d(0, 0, 0))
floor.SetFixed(True)
system.Add(floor)

dt = 0.01
for step in range(50):
    system.DoStepDynamics(dt)
    if step % 10 == 0:
        pos = box.GetPos()
        print(f"t={system.GetChTime():.2f}s  box y={pos.y:.4f}")

print("PyChrono is working.")
