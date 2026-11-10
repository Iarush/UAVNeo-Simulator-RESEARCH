# UAVNeo-Simulator

_The MIT Beaver Works UAV simulation environment, based on the [RACECAR Neo Simulator](https://github.com/MITRacecarNeo/RacecarNeo-Simulator)._

UAVSim is a Unity-based simulator for an educational quadrotor drone. It pairs with the [`uav-neo-library`](https://github.com/MITUavNeo/uav-neo-library) Python control library, which students import to control the simulated (or physical) drone via UDP.

> **Research fork**: this version adds custom STL drone import and PX4-style flight physics. See [Research Additions](#research-additions-custom-frames-and-real-flight-physics).

> **Note**: As of v0.0.3, UAVSim is built using Unity `6000.4.5f1` (Unity 6). Older or newer versions of Unity are not supported. Migration to a newer version of Unity or opening the project in an older version may be risky and cause unexpected results.

## Table of Contents
- [Changelog](#changelog)
- [Getting Started](#getting-started)
    - [Manual Flight Controls](#manual-flight-controls)
    - [Visual Check: Drone and Propellers](#visual-check-drone-and-propellers)
- [Research Additions: Custom Frames and Real Flight Physics](#research-additions-custom-frames-and-real-flight-physics)
- [Repository Contents](#repository-contents)
    - [Drone Modules](#drone-modules)
- [Python Interface](#python-interface)
    - [Synchronous Communication Protocol](#synchronous-communication-protocol-used-for-python-scripts)
    - [Asynchronous Communication Protocol](#asynchronous-communication-protocol-used-for-jupyter-notebook)
    - [Send Fragmented](#send-fragmented)
- [Modeling Error](#modeling-error)
- [Heritage](#heritage)

## Changelog

See [CHANGELOG.md](./CHANGELOG.md) for a full version history. Highlights:

Changes made by this research project, newest first.

- **2026-11-09** - A Windows build of this version is published on the `windows` branch. It includes its own Python for custom frames, so there is nothing to install, and it is what the student installer downloads. Mac and Linux builds are not made.

- **2026-11-09** - The mass service now rejects a material density of 0 or less with an error. Before, it used 1250 kg/m^3 without telling the user. Added the "Research Additions" section to this README.

- **2026-10-20** - Flight now behaves like the real UAV Neo drone. The sim uses the real drone's PX4 flight-controller settings.
  - Motors take time to speed up and slow down. Air drag grows with speed and frame size.
  - Takeoff and landing follow real physics. The drone holds its position when you release the sticks.
  - Light frames that used to climb away now hover.
  - The sim warns you if a custom frame would be hard to control.
  - Takeoff, landing, drag and the warning are built but not yet approved. The labs still need re-checking against the new physics.

- **2026-10-13** - You can import your own `.stl` drone frame and fly it. Enter the material density and the mass and position of each motor, battery and electronics part on one import panel (4 to 12 rows). The sim finds the centre of mass and inertia, swaps in your mesh and moves the propellers to your motor positions. If the import fails, the default drone loads.
  - Save an import as a preset and pick it from a dropdown later.
  - The sim starts its own Python mass service from a bundled copy, so you install nothing. This works in the Editor. In a Windows build, the bundle is present and the bundled Python answers on port 5070 when started from the build folder. Starting it from the import panel in the build has not been clicked through.
  - A zoom bar fits the camera to the size of the frame.

- **v1.0.0** - Redesign efforts to labs, autograder, UI, and curriculum. Arena prefab added
  to reinforce theming, as well as standardizing lighting to prevent unwanted scene bakes.
  Scenes are now procedurally generated on start, including the ones within the autograder.
  Cleaned up remaining racecar prefabs and assets that are no longer used.
  
- **v0.0.3** — Module 6, autograder integration, and a major refactor that finishes
  the migration from RACECAR Neo (class renames, folder reorg, LIDAR/Drive/IMU
  bandaid removal, Hud refactor, Unity 6 API cleanup, ~1700-file racecar asset purge).
  
- **v0.0.2** — AirSim Labs integration: Modules 1–5 scenes (HelloDrone, DroneControl,
  ObjectDetection, ArUcoMaze, SearchAndRescue, MazeNavigation) and updated drone
  flight behavior. Merged via [PR #1](https://github.com/MITUavNeo/UAVNeo-Simulator/pull/1).
  
- **v0.0.1** — Initial beta release; Unity project import forked from RACECAR Neo
  Simulator with AR markers, tape patterns, and base drone player prefab.

## Getting Started

UAVSim is built with Unity and C#. Before you begin, you will need to download Unity [here](https://unity3d.com/get-unity/download). Select the option that says "Download Unity Hub".

Open Unity Hub and install Unity `6000.4.5f1` from the "Installs" page. In the "Projects" page, click the "Add" button and select the root of this repository. This will import the UAVSim project into Unity.

If you installed [Microsoft Visual Studio](https://visualstudio.microsoft.com/) with Unity, you can open the generated solution file to edit the C# scripts directly.

Inside the Unity Editor, you can build UAVSim from `File -> Build Settings`.

To control the simulated drone, install [`uav-neo-library`](https://github.com/MITUavNeo/uav-neo-library) on the Python side. A minimal student program looks like:

```python
from uav_neo_library import Drone

drone = Drone()

def start():
    drone.drive.set_speed_angle(0, 0)

def update():
    # ... read sensors, command the drone
    pass

drone.go(start, update)
```

### Manual Flight Controls

In default flight mode you can fly the drone from the keyboard or a controller:

| Action | Keyboard | Controller |
|---|---|---|
| Launch / land | `L` | — |
| Forward / back / strafe | `W` `A` `S` `D` | Left joystick |
| Ascend / descend | `Right Shift` / `Left Shift` | Right / left trigger |
| Yaw | `←` / `→` | Right joystick X |
| Max speed down / up | `Z` / `/` | LB / RB |
| Cycle camera (chase, top-down, front) | `Space` | — |
| Zoom camera in / out | Mouse wheel, or `=` / `-` (keypad `+` / `-`) | — |

#### Zoom bar

A **zoom bar** (slider) appears on screen in the flight scene. Drag it to zoom in or out: moving it up (right) zooms in, down (left) zooms out. When a custom STL frame is loaded, the bar starts at a zoom fitted to the frame's size, so large or small imported drones start fully in view.

The bar is shown while the drone is on the ground and hides once the drone launches, so it can't capture the arrow keys used for yaw. The mouse wheel and `=` / `-` keys keep zooming in flight. Zoom only moves the follow cameras, so the drone's flight and propeller animation are unchanged.

### Visual Check: Drone and Propellers

Run this check after changing the drone prefab, the flight code, or the custom-frame import, and whenever you import a new model. It confirms that the drone looks right and its propellers spin properly.

**1. Default drone**

1. Open `Main.unity`, press Play, and press **Begin** without importing a frame.
2. Use the zoom bar to zoom in. You should see the stock drone body with **four propellers, one at each corner**.
3. Before launch, the propellers are **still**. This is expected because the motors are off.
4. Press `L` to launch. All four propellers should **start spinning right away and keep spinning** for the whole flight, including while hovering, turning, and landing.
5. Spin directions should alternate diagonally: **front-left and back-right clockwise, front-right and back-left counter-clockwise** (seen from above).
6. Fly around (`W` `A` `S` `D`, `Shift`, arrow keys). No propeller should freeze, drift away from its arm, or flicker.

**2. Imported model**

1. Open the import panel and wait for **Mass service: ready** under the status text. The panel starts the bundled service itself (run `tools/build-mass-service.ps1` once first so the bundle exists). If it shows **Mass service: failed**, the reason is shown there and in the Console, and the service log is `mass-service.log` in `Application.persistentDataPath`.
2. On the same panel, load an `.stl`, enter the density, and fill the node rows in the scrolling table below them. Or pick a saved setup from **Preset Drone config** at the top, which fills all three; to save the current setup, type a name next to it and press **Save Preset**. Press **Build Drone**, then **Back**, then **Begin** on the main menu.
3. Check that **your frame is shown and the stock body is hidden**. If you see both overlapping, or only the stock drone, check the Console (see below).
4. Use the zoom bar to fit the whole frame on screen. Nothing should be clipped or too small to inspect.
5. Check that **each propeller sits on a motor position** of your frame, not where the stock drone's propellers were.
6. Repeat steps 3–6 from the default-drone check. Propellers should be still before launch, spin as soon as you press `L`, and keep the same diagonal directions.

**3. Console messages**

With the Console open during Play:

- **Expected:** `[PropellerSpin] Found 4 propellers` and one `[PropellerSpin] Propeller_N ...` line per propeller showing its corner (Front Left, Front Right, Back Left, Back Right) and spin direction. With an imported model, `[CustomFrameLoader] Mass properties: ...` and `[CustomFrameLoader] Motor positions ...` should also appear.
- **Problem:** any `[CustomFrameLoader] ... Using the default drone.` error means the import failed and the stock drone was used. The message gives the reason (for example, a missing file, an invalid motor layout, or the service not responding).
- **Problem:** `... is inside default body root ...` means a propeller is under an object that is hidden for custom frames, so that propeller will disappear with an imported model.
- **Problem:** `defaultBodyObjects is empty` means the stock body will draw on top of your frame.

> **Tip**: Fast-spinning propellers can look slow, stopped, or even backwards on screen because of the frame rate (the "wagon-wheel" effect). If a propeller looks wrong, check its direction in the `[PropellerSpin]` Console line before treating it as a bug.

## Research Additions: Custom Frames and Real Flight Physics

This research project added two things on top of the original UAVSim: importing a custom drone frame from a CAD file, and a flight model closer to a real PX4 quadcopter. The work is on the `feature/real-flight-physics` branch. Project notes are in `CLAUDE.md`, `DEVLOG.md`, `.plans/real-flight-physics-spec.md` and `docs/flight-audit-2026-10-13.md`.

### Custom Drone Frame Import

A student can import their own `.stl` frame and fly it. The goal is a CAD file in and a physically plausible quadcopter out.

On the main menu, open the custom drone panel. Then:

1. Browse to an `.stl` file and enter a material density in kg/m³.
2. Fill the node table: type, position and mass for each node. Types are motor, battery, flight controller and companion computer. The table starts with 6 rows and can hold 4 to 12. Positions are in the CAD file's Z-up frame, in metres.
3. Press **Build Drone**. It checks the STL, the density and the rows together.
4. Optionally save the setup as a preset. The **Preset Drone config** dropdown at the top lists saved presets and fills in the path, density and rows. Presets are stored in `drone_presets.json` in `Application.persistentDataPath`.
5. Press **Back**, then **Begin** on the main menu.

The mass numbers come from a small Python service:

- Unity sends the STL path, density and nodes to `127.0.0.1:5070` over UDP.
- The service uses the [`masscompose`](https://pypi.org/project/masscompose/) package. Its two scripts, `udp_listener.py` and `mass_composer.py`, are in `UAVSim/Assets/MassService~/mass-properties/`.
- Unity starts the service itself when the import panel opens. It uses a bundled Python 3.13 (build it once with `tools/build-mass-service.ps1`; the pinned packages are in `tools/mass-service-requirements.txt`). Unity stops the service when it quits.
- The service returns the centre of mass, the principal moments of inertia, an inertia rotation and an arm length.

Unity then swaps the default drone body for the STL mesh. It sets the `Rigidbody` centre of mass, inertia tensor and inertia rotation from the reply. It moves the propellers to the motor positions. `CustomFrameLoader.cs` does this work.

If anything fails, Unity logs a clear reason and flies the default drone. Failures include the service being down, a bad STL and an invalid motor layout. The wire protocol is fixed; see `CLAUDE.md` for its exact layout. For the step-by-step check, see [Visual Check: Drone and Propellers](#visual-check-drone-and-propellers).

A cube of known size and density was checked against hand arithmetic for mass, centre of mass and inertia. They matched.

### Real Flight Physics

The original flight model reacted almost instantly. The new model follows PX4 v1.17.0 and applies to every drone, including the default one. It was built in phases 0 to 5. The plan is in `.plans/real-flight-physics-spec.md`.

- **Config:** PX4 parameters from the real drone are in `UAVSim/Assets/StreamingAssets/Flight/uav-neo.params`. `tools/extract-px4-params.py` makes the file from QGroundControl `.tlog` files. Sim-only values are in `flight-sim.json`. `FlightConfig.cs` reads both. Only the real drone's behaviour limits are used, not its hardware.
- **Attitude and rate control:** PX4's attitude and rate loops and control allocation, run at 250 Hz inside `Flight.cs`. Gains are PX4 stock values.
- **Motors:** each motor is sized so the drone hovers at `MPC_THR_HOVER`. Each has a first-order lag and pushes at its own position, so the centre of mass and motor layout matter.
- **Position control:** PX4 velocity, position and vertical loops. Released sticks brake by tilting, then hold position. The keyboard and controller fly PX4 POSCTL. `send_pcmd` keeps its existing tilt meaning.
- **Takeoff and landing:** a physics takeoff to 1.0 m and a landing at 0.6 m/s with a land detector. Motors spin down on disarm. Custom frames get a convex `MeshCollider` for ground contact.
- **Drag and stability:** quadratic drag per body axis, sized from the frame's bounds. A warning appears if the frame's control authority is outside the tested band or the rate error oscillates. `FrameStabilityEstimate.cs` gives an estimate at Build Drone.

Status: phases 0 to 3 are done. Phases 4 and 5 are built and awaiting owner approval.

Tests are PlayMode tests in `UAVSim/Assets/Tests/PlayMode/`. Close the Unity Editor and run them with the Unity CLI:

```
"C:\Program Files\Unity\Hub\Editor\6000.4.5f1\Editor\Unity.exe" -batchmode -projectPath UAVSim -runTests -testPlatform PlayMode -testFilter <regex> -testResults <xml> -logFile <log>
```

Use `-testCategory "!Sweep"` to skip the slow stability sweep.

### Known Limits

- Windows only. The bundled Python is the Windows embeddable package.
- Quadcopters in an X layout only: exactly four motors, one per corner. Plus layouts, hex and octo frames are not supported.
- The loaded mesh is mirrored relative to the CAD file. This is harmless on a symmetric frame and wrong on a chiral one. It is deferred, not fixed.
- Preset STL paths are absolute. If the file moves, the preset loads with a missing file and the panel says so.
- In a Windows build (2026-11-09) the bundled Python is copied into the player and answers a probe on port 5070 when started from the build folder. Starting it from the import panel in the player, and a machine without Python, have not been tested.

### Future Work: Labs

The labs have not been re-checked against the new physics. Flight now uses PX4-style control, motor lag, drag and a real takeoff and landing. Timing and behaviour the labs relied on may be different. The labs must be re-run and probably re-tuned.

`send_pcmd` keeps its existing tilt meaning. It was deliberately not changed to the real drone's velocity-setpoint meaning. The lab regression has not been run yet. It is planned once, after all physics phases are approved.

Other open items:

- Click through the import, preset and flight steps by hand.
- Test a built player on a machine without Python, including Windows Firewall and SmartScreen behaviour.
- Remove `DebugFrameTestSetup` and the `TestSetup` object in `Main.unity`.
- The PyPI `masscompose` package asks for Python 3.13 or later. The bundled Python 3.13 covers the sim, but the student's own Python 3.9 environment cannot install it.

## Repository Contents

You will find the following directories inside of `Assets/`:

- **2D**: Images, including AR-tag patterns, tape patterns, HUD content, and UI textures.
- **Editor**: Editor-only scripts and tooling (`BuildAltitudeGates`, `BuildF1Track`, `GRGAutograderSetup`).
- **Fonts**: Non-standard fonts used in the HUD and menus.
- **Materials**: Render and physics materials.
- **Models**: 3D models, including the drone body, gates, and lab props.
- **Prefabs**: Pre-made GameObjects organized into:
  - `Drone/` — the `DronePlayer` prefab.
  - `Shared/` — `LevelManager`, `LevelManagerAutograder`, `ResetBox`, `Sun`, `Terrain`.
  - `Legacy/` — staging folder for legacy racecar prefabs (held back pending deletion).
  - `KeyPoints/`, `Obstacles/`, `UI/` — supporting prefabs.
- **Resources**: Runtime-loadable assets.
- **Scenes**: The "levels" of the simulation. `Main.unity` provides the main menu;
  `FlightDemo.unity` and `Demo.unity` are sandbox-style scenes; the
  `UAV_Neo_Labs/` subdirectory contains the lab modules and their autograder
  variants.
- **Scripts**: The C# scripts that control the simulator.
- **Terrain**: Terrain assets.
- **Textures**: Render textures (color camera, depth camera, downward-facing camera).

Inside of the `Assets/Scripts` directory, you will find the following organization:

- **(root level)**: Cross-cutting scripts (`CenterOfMass`, `PerformanceOptimizer`, `ReloadBuffer`).
- **LevelManagement**: Scripts that control objects in a level (finish lines, key points, autograder triggers).
- **Drone**: Modules that control the drone and all of its hardware.
- **NonMonoBehavior**: Plain C# helpers including `PythonInterface`, the UDP bridge to Python.
- **UI**: Scripts controlling the 2D user interface, including the HUD and menus.

### Drone Modules

The [`Drone`](./Assets/Scripts/Drone/Drone.cs) class controls the drone and roughly mirrors the structure of the [`uav-neo-library`](https://github.com/MITUavNeo/uav-neo-library) library used by the real drone. The following sub-modules each handle a particular aspect of the drone's hardware:

- **`CameraModule`**: Models the color and depth imaging capabilities of the drone's forward-facing camera, plus the downward-facing nadir camera used for line-following and AR-tag tasks. The `DronePlayer` prefab includes a color camera that renders to a dedicated render texture; `CameraModule` pulls this texture from the GPU when requested. The depth image is created by performing a ray cast at "each pixel", at a lower resolution to preserve performance. Both operations are expensive, so color and depth images are cached per frame.
- **`Controller`**: Handles input from the keyboard and Xbox controller. Xbox controllers are mapped differently per operating system, so this module includes a compilation constant which must be set based on the operating system. Arming is bound to `Controller.Button.A` (the `1` key on keyboard).
- **`Flight`**: Handles the drone's motion model — thrust, attitude, and translational/yaw rate commands. Replaces the racecar's `Drive` module.
- **`PhysicsModule`**: Models the drone's IMU. Angular velocity is taken directly from the rigidbody. Linear acceleration is calculated as change in linear velocity, taken from the rigidbody.
- **`CrashSystem`**: Detects collisions and arms/disarms the drone in response, with optional reset behavior tied to `LevelManager`.
- **`PropellerSpin`**: Visual-only propeller animation tied to throttle state.

_The `CameraModule` and `PhysicsModule` are named as such to avoid conflicting with the `Camera` and `Physics` classes provided by Unity._

> **Note**: The 2D LIDAR module from RACECAR Neo has been removed. Its opcodes
> (`lidar_get_num_samples`, `lidar_get_samples`) are reserved on the wire so
> existing protocol versions remain byte-compatible, but they fall through to
> the default "not supported" branch. Drone-side perception uses the forward
> color/depth cameras and the downward nadir camera instead.

## Python Interface

UAVSim communicates with a user's Python program using a custom protocol sent over a UDP connection. This is all handled in the [`PythonInterface`](./Assets/Scripts/NonMonoBehavior/PythonInterface.cs) class. On the Python end, this communication is handled by `drone_core_sim.py` in the `uav-neo-library` library.

`PythonInterface` has two `UdpClient`s operating on fixed ports:

- **Sync port**: `5065`
- **Async port**: `5064`
- **Protocol version**: `1`

The async client operates on a separate thread and handles asynchronous data requests from Jupyter and connection requests from user programs. Once a connection has been established, UAVSim communicates with the user's program through the synchronous client, which operates on the main thread.

In `PythonInterface`, the `Header` enum declares the reserved messages used in the communication protocol. Both Unity and Python declare `Header` and `Error` enums in matching positional order, so byte values align across both sides regardless of the C#/Python identifier names.

### Synchronous Communication Protocol (used for Python scripts)

1. The Python program periodically sends `connect` messages to the UAVSim async client.
2. The async client receives a `connect` message, records the port of the Python program, and responds with a `connect` message. From now on, communication will occur through the synchronous client.
3. When the user enters "User Program" mode in UAVSim, the sync client sends a `unity_start` message to Python and blocks to await a response from Python.
4. Python runs the user's `start` function. If the `start` function calls any of the `drone_core` APIs, these API calls are passed back to the sync client with the corresponding header. For example, if the user calls `drone.camera.get_color_image()`, Python will send the `camera_get_color_image` message to the sync client. UAVSim then responds with the requested data.
5. Once the Python program finishes executing the user's `start` function, it sends back a `python_finished` message. UAVSim stops blocking.
6. Once per frame, the sync client sends a `unity_update` message to Python and blocks to await a response from Python.
7. Python runs the user's `update` function. Once again, it passes API calls back to the sync client.
8. When Python finishes executing the user's `update` function, it sends back a `python_finished` message. UAVSim stops blocking.
9. This process repeats until the user exits or restarts the level in UAVSim. In either of these cases, the sync client sends a `unity_exit` message to Python.
10. Upon receiving the `unity_exit` message, the Python program closes.

Note that after a connection is established, this protocol is **completely synchronous**: both systems are synchronized to the Unity update clock. UAVSim will always block the main thread until receiving a message from Python (or until a timeout occurs — 5 seconds by default).

### Asynchronous Communication Protocol (used for Jupyter Notebook)

1. Jupyter Notebook sends the async client a request for data from a particular sensor (such as a `camera_get_color_image` or `camera_get_depth_image` message).
2. The async client receives the request and tells the corresponding sensor on the main thread that data has been requested.
3. In the subsequent Update frame, the main thread updates the data from that sensor.
4. After a short waiting period, the async client reads the updated data and returns it to Jupyter Notebook.

### Send Fragmented

Color images are too large to send in a single UDP packet, so they are split across several UDP packets (max packet size: 65507 bytes). After each packet, UAVSim blocks until Python responds with the `python_send_next` message, which indicates that it is ready to receive the next fragment.

## Modeling Error

When the "Realism" option is enabled via the settings, a realistic amount of random gaussian error is added to the following sensors:

- depth camera
- IMU

The error rate is based on the data sheets for these sensors.

## Heritage

UAVSim is forked from the [RACECAR Neo Simulator](https://github.com/MITRacecarNeo/RacecarNeo-Simulator), which itself descends from the original [MIT-LL RACECAR](https://github.com/MITLLRacecar) Simulation. The wire protocol is byte-compatible with RACECAR Neo's by design — opcode positions are preserved across the rename so messages send and parse identically on either side. Internal naming, scene content, and prefabs have all been migrated to drone-first conventions; see [CHANGELOG.md](./CHANGELOG.md) for the full migration history.
