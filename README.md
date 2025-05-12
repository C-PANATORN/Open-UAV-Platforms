# Open-UAV-Platforms
A list of Open-Source Hardware (OSH) Flight Controllers, Ground Control Stations (GCS), and UAV Simulators.  
All contributions are welcome! **Updated 13 May 2025**

---

## Flight Controllers

Comparison of MCUs, sensors and licenses for OSH flight controller platforms.
_All platforms have IMUs. Interfaces: UART, PWM, I2C._ [[1]](#references)

| Platform        | MCU           | Sensors      | License              | Interfaces                          |
|-----------------|---------------|--------------|----------------------|-------------------------------------|
| Pixhawk         | STM32F427     | b, m         | BSD-2-Clause         | c, s, a, pp, sb, ds                 |
| Pixhawk 2       | STM32F427     | b, m         | CC-BY-SA-3.0         | c, s, a, pp, sb, ds                 |
| PixRacer        | STM32F427     | b, m         | CC-BY-4.0            | c, pp, sb, ds                       |
| Pixhawk 3 Pro   | STM32F427     | b, m         | CC-BY-4.0            | c, s, pp, sb, ds                    |
| PX4 FMUv5 & v6  | STM32F427     | b, m         | CC-BY-4.0            | c, s, a, pp, sb, ds                 |
| Sparky2         | STM32F405     | b, m         | CC-BY-NC-SA-4.0      | c, pp, sb, ds, da                   |
| Chimera         | STM32F767     | b, m, p      | GPL-2.0              | c, s, a, da, pp, sb, ds, x, au      |
| CC3D            | STM32F103     | None         | GPL-3.0              | pp, ds, sb                          |
| Atom            | STM32F103     | None         | GPL-3.0              | pp, ds, sb                          |
| APM 2.8         | ATmega2560    | b            | GPL-3.0              | pp, a                               |
| FlyMaple        | STM32F103     | b, m         | GPL-3.0              | –                                   |
| Erle-Brain 3    | Raspberry Pi  | b, m         | CC-BY-NC-SA-4.0      | a                                   |
| PXFmini         | Raspberry Pi  | b, m         | CC-BY-NC-SA-4.0      | a                                   |
| AeroQuad [d]    | STM32F407     | b, m         | GPL-2.0              | –                                   |
| Mikrokopter [d] | ATmega644     | b            | –                    | s, pp                               |
| MatrixPilot [d] | dsPIC33FJ256  | None         | GPL-3.0              | –                                   |

> **Notes:**
> - b: barometer; m: magnetometer; p: pitot tube sensor c: CAN; s: SPI; a: ADC; pp: PPM; sb: S.BUS; ds: DSM; da: DAC; x: XBEE; au: AUX, [d]: discontinued

---

## Ground Control Stations (GCS)

Comparison of GCS software and their supported platforms.[[2]](#references)

| GCS                      | Lastest Release                                                                  | Supported Platforms          | OS (L/W/M)   | Protocol(s)        | Language / Framework   | License                   |
|--------------------------|----------------------------------------------------------------------------------|------------------------------|--------------|--------------------|------------------------|---------------------------|
| Mission Planner          | [v1.3.82](https://github.com/ArduPilot/MissionPlanner/releases)                  | ArduPilot                    | ✕/✓/✓       | MAVLink            | .NET / C#              | GPL-3.0-only              |
| APM Planner 2            | [v2.0.30-rc3](https://github.com/ArduPilot/apm_planner/releases)                 | ArduPilot, PX4               | ✓/✓/✓       | MAVLink            | Qt / C++               | GPL-3.0-or-later          |
| MAVProxy                 | [v1.8.71](https://github.com/ArduPilot/MAVProxy/releases)                        | ArduPilot                    | ✓/✕/✕       | MAVLink            | Python                 | GPL-3.0-or-later          |
| AndroPilot               | [v1.9.14](https://github.com/tstellanova/andropilot/blob/master/RELEASE-NOTES.md)| ArduPilot                    | ✕/✕/✕       | MAVLink            | Java                   | GPL-3.0-only              |
| QGroundControl           | [v4.4.4](https://github.com/mavlink/qgroundcontrol/releases)                     | ArduPilot, PX4               | ✓/✓/✓       | MAVLink            | Qt / C++               | Apache-2.0 / GPL-3.0-only |
| Paparazzi Center         | [v6.4.0](https://github.com/paparazzi/paparazzi/releases)                        | Paparazzi                    | ✓/✓/✓       | PprzLink           | Python                 | GPL-2.0-only              |
| LibrePilot GCS           | [v16.09](https://github.com/librepilot/LibrePilot/tags)                          | LibrePilot                   | ✓/✓/✓       | UAVTalk            | C++ / Qt               | GPL-3.0-only              |
| Betaflight-Configurator  | [v10.10.0](https://github.com/betaflight/betaflight-configurator/releases)       | Betaflight                   | ✓/✓/✓       | MSP, MAVLink       | Electron / JavaScript  | GPL-3.0-only              |
| iNAV-Configurator        | [v8.0.1](https://github.com/iNavFlight/inav-configurator/releases)               | iNAV                         | ✓/✓/✓       | MSP, MAVLink       | Electron / JavaScript  | GPL-3.0-only              |

> **Notes:**  
> - Order for OS is Linux/Windows/macOS, with ✓ = supported, ◐ = partial, ✕ = not supported.
> - AndroPilot is Android-only, hence ✕/✕/✕ here.  
> - All other entries match their official supported OS lists.

---

## UAV Simulators

Comparison of features for widely-used UAV simulators.[[3]](#references)

| Simulator                   | Lastest Release|  Physics Engine                     | Rendering     | OS (L/W/M)  | Interfaces                                        | (S/H)ITL           | License                 | 
|-----------------------------|----------------|-------------------------------------|---------------|-------------|---------------------------------------------------|--------------------|-------------------------|
| Gazebo Classic **(EOL)**    |[v11.15.1](https://github.com/gazebosim/gazebo-classic/tags)          | ODE, Bullet, DART, Simbody          | OGRE          | ✓/◐/✓ | ROS 1/2, C++, RL                                  | PX4, ArduPilot, CF | Apache-2.0              | 
| Gazebo                      |[gz-sim9_9.0.0](https://github.com/gazebosim/gz-sim/releases)         | Bullet, DART, TPE                   | OGRE          | ✓/◐/✓ | ROS 1/2, C++, Python, RL                          | PX4, ArduPilot, CF | Apache-2.0              |
| Isaac (Pegasus, Aerial Gym) |[v2.0.0](https://github.com/ntnu-arl/aerial_gym_simulator/releases)   | NVIDIA® PhysX, Flex                 | Vulkan        | ✓/✕/✕ | ROS 1/2, Python, RL                               | Pegasus: PX4      | Proprietary (BSD 3)     |
| Webots                      |[R2025a](https://github.com/cyberbotics/webots/releases)              | ODE                                 | OpenGL        | ✓/✕/✕ | ROS 1/2, C/C++, Python, MATLAB, Java              | ArduPilot, CF      | Apache-2.0              |
| CoppeliaSim                 |[v4.9.0-rev6](https://github.com/CoppeliaRobotics/coppeliaSimLib/tags)| Bullet, ODE, Vortex, Newton, MuJoCo | OpenGL        | ✓/✓/✓ | ROS 1/2, C/C++, Python, MATLAB, Java, Lua, Octave | —                 | GNU GPL & Commercial    |
| AirSim **(EOL)**            |[v1.8.1](https://github.com/Microsoft/AirSim/releases)                | NVIDIA® PhysX                       | Unreal, Unity | ✓/✓/✓ | ROS 1, C++, Python, C#, Java, RL                  | PX4, ArduPilot     | MIT                     |
| Flightmare                  |[v0.0.5](https://github.com/uzh-rpg/flightmare/releases/tag/0.0.5)    | Ad hoc, Gazebo Classic              | Unity         | ✓/✕/✕ | ROS 1, C++, RL                                    | —                 | MIT                     |
| FlightGoggles               |[v3.0](https://github.com/mit-aera/FlightGoggles/releases)            | Ad hoc                              | Unity         | ✓/◐/✕ | ROS 1, C++                                        | Motion Capture     | MIT                     |
| gym-pybullet-drones         |[v1.0.0](https://github.com/utiasDSL/gym-pybullet-drones/releases)    | PyBullet                            | OpenGL        | ✓/◐/✓ | Python, RL                                        | Betaflight, CF     | MIT                     |
| RotorTM                     |[rotortm2023](https://github.com/arplaboratory/RotorTM)               | Ad hoc                              | OpenGL        | ✓/✕/✕ | ROS 1, Python, MATLAB                             | —                  | GNU GPL                 |
| MATLAB UAV Toolbox          |[R2024b](https://www.mathworks.com/help/uav/release-notes.html)       | MATLAB                              | Unreal        | ✓/✓/✓ | ROS 2, MATLAB                                     | PX4                | Proprietary, Commercial |

> **Notes:**
> - Order for OS is Linux/Windows/macOS, with ✓ = supported, ◦ = partial, ✕ = not supported.  
> - EOL = End of Life Service.  
> - RotorTM has no specific release version; hence *rotortm2023*.  
> - All other entries match their official latest release.

---

## References
1. Aliane, N. (2024). A Survey of Open-Source UAV Autopilots. Electronics, 13(23), 4785. [https://doi.org/10.3390/electronics13234785.](https://doi.org/10.3390/electronics13234785)
2. Ebeid, E., Skriver, M., Terkildsen, K.H., Jensen, K. and Schultz, U.P. (2018) A Survey of Open-Source UAV Flight Controllers and Flight Simulators. Microprocessors and Microsystems, 61, 11-20.
[https://doi.org/10.1016/j.micpro.2018.05.002.](https://doi.org/10.1016/j.micpro.2018.05.002)
3. Dimmig, C. A., Silano, G., McGuire, K., Gabellieri, C., Hönig, W., Moore, J., & Kobilarov, M. (2024). Survey of simulators for aerial robots: An overview and in‐depth systematic comparisons. IEEE Robotics & Automation Magazine. [https://doi.org/10.1109/MRA.2024.3433171.](https://doi.org/10.1109/MRA.2024.3433171)
