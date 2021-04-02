![rfsim](https://raw.githubusercontent.com/EgorOrachyov/RobotFootballSim/main/docs/pictures/rfsim_app.png)

# Robot Football Simulator

Robot football simulator application with physically correct game simulation,
different game scenario and game rules, rich settings and customization features 
and with C API to develop and run algorithms, required to control robots, test
various game strategies and game mechanics.

### Features Summary

- [X] Simulated 2d physics of the game
- [X] Optimized 2d rendering for the football game
- [X] Game scenario for different game setup
- [X] Game rules to control game flow
- [X] Plugin system for user-defined algorithms loading at runtime
- [X] GUI application
- [X] Settings to tweak simulation and display options
- [X] Debug window with current simulation state info
- [X] Example Algorithms
- [X] Config file support

### Platforms

* macOS (tested on Mojave)
* Linux based OS (tested on Ubuntu 20.04)
* Windows (tested on 10)

## Third-party dependencies

* [glfw](https://www.glfw.org) for cross-platform window and input management
* [glew](https://github.com/Perlmint/glew-cmake) for OpenGL functions and extensions loading
* [box2d](https://github.com/erincatto/box2d) for 2d robots physics simulation 
* [imgui](https://github.com/ocornut/imgui) for creating application GUI
* [stb image](https://github.com/nothings/stb) for images processing
* [dynalo](https://github.com/maddouri/dynalo) for shared/dynamics libraries management
* [picojson](https://github.com/kazuho/picojson) for json configs loading
* [glm](https://github.com/g-truc/glm) for 3d math functions

## Getting started

### Dependencies

The project uses cross-platform windowing and input management library **glfw**.
This library uses native windowing APIs and built-in OS window frameworks,
however it may require additional setup step for linux users. Follow the official
**glfw** guide to setup this dependencies by `apt-get` tool.s

### Source code

Get the source code and initialize dependencies of the project.

```shell script
$ git clone https://github.com/EgorOrachyov/RobotFootballSim.git
$ cd RobotFootballSim
$ git submodule update --init --recursive
```

### Build

Configure build directory and run the build process.

```shell script
mkdir build
cd build
cmake ..
cmake --build .
```

Alternatively import project into desired IDE (import as cmake project).

In order to start the simulator `rfsim` target, you will have to build all default
algorithm plugins targets `randommove` and `followmove`. As soon as its done,
you will be able to correctly start the simulator application.

### Run

Inside the build directory go into directory with actual `rfsim` executable
(on linux/macos it will be placed in `build/rfsim`, on windows some generators
can output the actual executable into `build/Debug/rfsim` and something similar):

```shell script
./rfsim
```

This command starts simulator application, so the app primary window must appear on the screen.

### Config

Simulator uses `config.json` to tweak simulator params, which are parsed and applied on application start-up.
This config file is placed near `rfsim` executable inside build directory in time of cmake build process.
Use this config file to customize the following params:

* **windowWidth** Primary window width in universal units **ADVANCED**
* **windowHeight** Primary window height in universal units **ADVANCED**
* **fontScale** Font scale for hdpi displays
* **guiScale** GUI elements scale for hdpi displays **ADVANCED**
* **resourcesPath** Prefix path to simulator resources **ADVANCED**
* **pluginPathPrefix** Prefix path to algorithms plugins **ADVANCED**
* **pluginPaths** Actual plugins names to load on start-up (use name without `lib` and `ext` specifiers)

> macOS: Tweak fontScale and set it to 1.0 or less on high-resolution Retina displays.
> Windowing library glfw and imgui automatically scales font and window size on this
> platform, so explicit scaling is not required.

### Algorithm

In order to create custom algorithm you need to create a shared/dynamic library object
and expose API conventions functions, which will be called by the simulator for each simulation step.
The API is described in `include/rfsim/rfsim.h` header file. The template of the basic algorithm is following:

```c++
#define RFSIM_EXPORTS
#include <rfsim/rfsim.h>
#include <cstring>

RFSIM_DEFINE_FUNCTION_INIT {
    std::strcpy(context->name, "Algorithm Template");
    std::strcpy(context->description, "Default algorithm template for readme");
    return rfsim_status_success;
};

RFSIM_DEFINE_FUNCTION_BEGIN_GAME {
    return rfsim_status_success;
};

RFSIM_DEFINE_FUNCTION_TICK_GAME {
    return rfsim_status_success;
};

RFSIM_DEFINE_FUNCTION_END_GAME {
    return rfsim_status_success;
};

RFSIM_DEFINE_FUNCTION_FINALIZE {
    return rfsim_status_success;
};
```

This algorithm does actually nothing. However, it provides five callback functions,
which will be called for each simulation step. Description of the functions:

1. `rfsim_status rfsim_init(rfsim_algo_state* context)` - called when the algorithm is loaded into the simulator
2. `rfsim_status rfsim_begin_game(rfsim_algo_state* context, const rfsim_game_settings* settings, const rfsim_game_start_info* start)` - called when new game is started
3. `rfsim_status rfsim_tick_game(rfsim_algo_state* context, rfsim_game_state_info* state)` - called each sim step
4. `rfsim_status rfsim_end_game(rfsim_algo_state* context)` - called when game is finished
5. `rfsim_status rfsim_finalize(rfsim_algo_state* context)` - called when algorithm is unloaded from simulator

Follow `rfsim.h` and some example algorithms implementations to create your custom algo.

### Conventions

1. Simulator uses 2d coordinates space with horizontal from left to right X-axis,
and vertical from top to bottom Y-axis. Rotations are encoded in radians, full rotation is 2 * PI respectively.
By default the body (robot) direction vector is `(1, 0)` and its angle is `0.0f`. 
The rotation has clockwise order, so the body with angle `PI / 4` has direction vector (0.707, 0.707).
More info is shown on the image: 
![rfsim](https://raw.githubusercontent.com/EgorOrachyov/RobotFootballSim/main/docs/pictures/coords-space.png)

2. For measurements simulator uses the metric system of measures and weights.
Robots and ball mass is in kilograms, speed is in meter/second, force is in Newtons, time in seconds and etc.
Physics properties of the field, robots and ball, friction and restitution are set in the 
`physics/PhysicsGameProperties.hpp` structure. This properties are custom for each game and can be found
in the `scenarion/ScenarionCommon.hpp` class. Physics settings are listed bellow:

* **fieldFriction** field friction for bodies
* **robotRadius** in meters
* **robotHeight** in meters
* **robotMass** in kilograms
* **robotFriction** robot material friction
* **robotRestitution** robot restitution for contacts
* **robotMaxSpeed** in meters/second
* **robotWheelXOffset** offset in meters from the center of the robot where wheels are placed
* **ballRadius** in meters
* **ballMass** in kilograms
* **ballFriction** ball material friction
* **ballRestitution** ball restitution for contacts

## Simulator Features

1. Main menu screen animated logo and menu to start new game with selected scenario, control algorithm and game rules.
![rfsim](https://raw.githubusercontent.com/EgorOrachyov/RobotFootballSim/main/docs/pictures/rfsim_main_menu.png)

2. Simulation control menu with features to `pause` game, `continue` game from current state of `restart`.
Also control window displays some statistic info current simulation: total time in game, current
frame rate, score and status. Selected scenario, rules and algorithm also displayed.
![rfsim](https://raw.githubusercontent.com/EgorOrachyov/RobotFootballSim/main/docs/pictures/rfsim_before_game.png)

3. Simulator allows you to pause and play game, tweak settings, dive into game process, display out and collision info.
![rfsim](https://raw.githubusercontent.com/EgorOrachyov/RobotFootballSim/main/docs/pictures/rfsim_ingame.png)

4. Simulator provides some basic settings tool, which allows to tweak simulation time scale, drawing settings, such as 
trace information, debug info window, collisions/out info display, shadows settings (including the Sun position),
field customization color.
![rfsim](https://raw.githubusercontent.com/EgorOrachyov/RobotFootballSim/main/docs/pictures/rfsim_settings.png)

5. Debug info provides current actual information about game state. In includes all dynamic objects
properties: position, velocity, rotation (angle, the same as direction), and motors forces.
![rfsim](https://raw.githubusercontent.com/EgorOrachyov/RobotFootballSim/main/docs/pictures/rfsim_debug_info.png)

## Directory structure

```
RobotFootballSim
├── docs - documents, text files and various helpful stuff
├── resource - data used by simulator at runtime
├── rfsim - simulator core source code
│   ├── include - library public C API for algorithm plugins
│   ├── plugins - algorithms for robots (loaded at runtime as shared/dynamic libraries)
│   ├── sources - source-code for implementation
│   │   ├── graphics - graphics server and image io for displaying actual game
│   │   ├── gui - user application sources
│   │   ├── logic - game logic, algorithms and managers
│   │   ├── opengl - low-level gl drawing stuff
│   │   ├── physics - game physics simulation source code
│   │   ├── rules - various game rules to control the game process
│   │   ├── scenario - scenario collectio for dirrent games setup
│   │   ├── shaders - glsl low-level drawing shaders code
│   │   └── utils - utility classes shared among source code modules
│   └── CMakeLists.txt - actual sim build config
├── deps - project dependencies
│   ├── box2d - physics engine 
│   ├── dynalo2 - patched dynalo lib for cross-platform shared lib management  
│   ├── glew - opengl loader
│   ├── glfw - cross-platform native windowing manager
│   ├── glm - 3d math 
│   ├── imgui - cross-platform GUI framework
│   ├── picojson - lightweight json parser
│   └── stb - collection of cross-platfom stb-based image utils 
├── CMakeLists.txt - project cmake config, add this as sub-directory to your project
└── config.json - application config file, copyied into build dir at cmake build step
```

## License

This project is licensed under MIT license. License text can be found in the 
[license file](https://github.com/EgorOrachyov/RobotFootballSim/blob/main/LICENSE.md).

## Contributors

- Unknown Contributor 1
- Unknown Contributor 2
- Unknown Contributor 3
- Unknown Contributor 4
- Unknown Contributor 5

## Also

This is a research project of students of the Faculty of Mathematics and Mechanics of 
St. Petersburg State University as part of the course `Algorithmic foundations of robotics`
under the guidance of Professor Andrey Nikolaevich Terekhov.

