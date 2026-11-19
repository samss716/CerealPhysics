# CerealPhysics

A custom rigid-body physics simulation engine written in C++.

CerealPhysics is a personal project in physics-based simulation and real-time computer graphics. The goal is to build a physics engine from the ground up, implementing the mathematical foundations and simulation algorithms behind rigid-body motion, collision detection, and collision response.

The project will simulate cereal pieces falling, colliding, and settling inside a cereal bowl, with a real-time 3D visualization.

## Goals

- Implement vector mathematics and rigid-body dynamics.
- Simulate forces, gravity, velocity, and acceleration.
- Develop semi-implicit eularian numerical integration and fixed-timestep simulation.
- Implement collision detection and response.
- Explore angular motion, torque, and rotational dynamics.
- Build a real-time 3D visualization of the simulation.

## Current Features

- **Vector Mathematics:** GLM-based 3D vector operations.
- **Rigid Bodies:** Mass, position, velocity, acceleration, and force accumulation.
- **Numerical Integration:** Semi-implicit Euler integration.
- **Physics World:** Centralized gravity and management of multiple rigid bodies.

## Tech Stack

- **Language:** C++20
- **Build System:** CMake
- **Compiler:** MSVC
- **Math Library:** GLM
- **Planned Visualization:** OpenGL and GLFW

## Building the Project

The project currently uses CMake with MSVC on Windows.

Configure the project from a Visual Studio Developer PowerShell terminal:

```powershell
cmake -S . -B build-x64 -G "NMake Makefiles"
```

Build:

```powershell
cmake --build build-x64
```

Run:

```powershell
.\build-x64\CerealPhysics.exe
```

GLM is managed through CMake FetchContent.

## Roadmap

- [x] Vector mathematics
- [x] Basic rigid-body implementation
- [x] Gravity and force integration
- [x] Physics world architecture
- [x] Fixed-timestep simulation
- [x] AABB collision detection
- [x] Collision response and friction
- [ ] Positional correctness
- [ ] Angular dynamics
- [ ] Cereal and bowl collision geometry
- [ ] Real-time 3D visualization

---
