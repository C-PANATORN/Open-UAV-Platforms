# Open-UAV-Platforms
A list of Open-Source Hardware (OSH) Flight Controllers, Ground Control Stations (GCS), and UAV Simulators. 
All contributions are welcome!

---

## Flight Controllers

Comparison of MCUs, sensors and licenses for OSH flight controller platforms. 
_All platforms have IMUs. Interfaces: UART, PWM, I2C._ [1].

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
> – b: barometer; m: magnetometer; p: pitot tube sensor c: CAN; s: SPI; a: ADC; pp: PPM; sb: S.BUS; ds: DSM; da: DAC; x: XBEE; au: AUX, [d]: discontinued

---

## Ground Control Stations (GCS)

Comparison of GCS software and their supported platforms [2].

| GCS                      | Supported Platforms          | OS (L/W/M)   | Protocol(s)        | Language / Framework   | License                   |
|--------------------------|------------------------------|--------------|--------------------|------------------------|---------------------------|
| Mission Planner          | ArduPilot                    | ❌/✅/✅    | MAVLink            | .NET / C#              | GPL-3.0-only              |
| APM Planner 2            | ArduPilot, PX4               | ✅/✅/✅    | MAVLink            | Qt / C++               | GPL-3.0-or-later          |
| MAVProxy                 | ArduPilot                    | ✅/❌/❌    | MAVLink            | Python                 | GPL-3.0-or-later          |
| AndroPilot               | ArduPilot                    | ❌/❌/❌    | MAVLink            | Java                   | GPL-3.0-only              |
| QGroundControl           | PX4, ArduPilot               | ✅/✅/✅    | MAVLink            | Qt / C++               | Apache-2.0 / GPL-3.0-only |
| Paparazzi Center         | Paparazzi                    | ✅/✅/✅    | PprzLink           | Python                 | GPL-2.0-only              |
| LibrePilot GCS           | LibrePilot                   | ✅/✅/✅    | UAVTalk            | C++ / Qt               | GPL-3.0-only              |
| Betaflight-Configurator  | Betaflight                   | ✅/✅/✅    | MSP, MAVLink       | Electron / JavaScript  | GPL-3.0-only              |
| iNAV-Configurator        | iNAV                         | ✅/✅/✅    | MSP, MAVLink       | Electron / JavaScript  | GPL-3.0-only              |

> **Notes:**  
> – Order for OS is Linux/Windows/macOS, with ✅ = supported, 🟡 = partial, ❌ = not supported.
> – AndroPilot is Android-only, hence ❌/❌/❌ here.  
> – All other entries match their official supported OS lists.

---

## UAV Simulators

Comparison of features for widely-used UAV simulators [3].

| Simulator                   | Physics Engine                      | Rendering     | OS (L/W/M)  | Interfaces                                        | (S/H)ITL           | License                 | 
|-----------------------------|-------------------------------------|---------------|-------------|---------------------------------------------------|--------------------|-------------------------|
| Gazebo Classic              | ODE, Bullet, DART, Simbody          | OGRE          | ✅/🟡/✅   | ROS 1/2, C++, RL                                  | PX4, ArduPilot, CF | Apache-2.0              | 
| Gazebo                      | Bullet, DART, TPE                   | OGRE          | ✅/🟡/✅   | ROS 1/2, C++, Python, RL                          | PX4, ArduPilot, CF | Apache-2.0              |
| Isaac (Pegasus, Aerial Gym) | NVIDIA® PhysX, Flex                 | Vulkan        | ✅/❌/❌   | ROS 1/2, Python, RL                               | Pegasus: PX4       | Proprietary (BSD 3)     |
| Webots                      | ODE                                 | OpenGL        | ✅/✅/✅   | ROS 1/2, C/C++, Python, MATLAB, Java              | ArduPilot, CF      | Apache-2.0              |
| CoppeliaSim                 | Bullet, ODE, Vortex, Newton, MuJoCo | OpenGL        | ✅/✅/✅   | ROS 1/2, C/C++, Python, MATLAB, Java, Lua, Octave | —                  | GNU GPL & Commercial    |
| AirSim                      | NVIDIA® PhysX                       | Unreal, Unity | ✅/✅/✅   | ROS 1, C++, Python, C#, Java, RL                  | PX4, ArduPilot     | MIT                     |
| Flightmare                  | Ad hoc, Gazebo Classic              | Unity         | ✅/❌/❌   | ROS 1, C++, RL                                    | —                  | MIT                     |
| FlightGoggles               | Ad hoc                              | Unity         | ✅/🟡/❌   | ROS 1, C++                                        | Motion Capture     | MIT                     |
| gym-pybullet-drones         | PyBullet                            | OpenGL        | ✅/🟡/✅   | Python, RL                                        | Betaflight, CF     | MIT                     |
| RotorTM                     | Ad hoc                              | OpenGL        | ✅/❌/❌   | ROS 1, Python, MATLAB                             | —                  | GNU GPL                 |
| MATLAB UAV Toolbox          | MATLAB                              | Unreal        | ✅/✅/✅   | ROS 2, MATLAB                                     | PX4                | Proprietary, Commercial |

> **Notes:**  
> – OS order: Linux/Windows/macOS.  
> – ✅ = supported, 🟡 = partial, ❌ = not supported.  
> – “Open-Source” and “Active” also use ✅/❌.  

---

## References

[1]. N. Aliane, “A Survey of Open-Source UAV Autopilots,” *Electronics*, vol. 13, art. 4785, Dec. 2024.
[2]. E. Ebeid, M. Skriver, K. H. Terkildsen, K. Jensen, and U. P. Schultz, “A survey of Open-Source UAV flight controllers and flight simulators,” *Microprocessors and Microsystems*, vol. 61, pp. 11–20, 2018. 
[3]. C. A. Dimmig, G. Silano, K. McGuire, C. Gabellieri, W. Hönig, J. Moore, and M. Kobilarov, “Survey of Simulators for Aerial Robots,” *IEEE Robotics & Automation Magazine*, Nov. 2023. 
