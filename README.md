![rfsim](https://raw.githubusercontent.com/EgorOrachyov/RobotFootballSim/main/docs/pictures/rfsim_app.png)

# Robot Football Simulator

Robot football simulator with graphical user interface and the feature
to develop and run algorithms, required to control robots, test
various game strategies and game mechanics.

### Features Summary

- [X] Dynamic algorithms loading
- [X] Simulated 2d physics of the game
- [X] 2d rendering for the football game
- [X] Game logic and rules 
- [X] GUI application, widgets and menus
- [ ] Example Algorithms
- [ ] Config support

## Third-party dependencies

* [glfw](https://www.glfw.org) for cross-platform window and input management
* [glew](https://github.com/Perlmint/glew-cmake) for OpenGL functions and extensions loading
* [box2d](https://github.com/erincatto/box2d) for 2d robots physics simulation 
* [imgui](https://github.com/ocornut/imgui) for creating application GUI
* [stb image](https://github.com/nothings/stb) for images processing
* [glm](https://github.com/g-truc/glm) for 3d math functions

## Getting started

Get the source code and initialize dependencies of the project.

```shell script
$ git clone https://github.com/EgorOrachyov/RobotFootballSim.git
$ cd RobotFootballSim
$ git submodule update --init --recursive
```

Configure build directory and run the build process.

```shell script
mkdir build
cd build
cmake ..
cmake --build .
```

Alternatively import project into desired IDE (import as cmake project).

## Simulator Features

Main menu screen animated logo and menu to start new game with selected scenario and control algorithm.
![rfsim](https://raw.githubusercontent.com/EgorOrachyov/RobotFootballSim/main/docs/pictures/rfsim_main_menu.png)

In-game control menu with features to `pause` game, `continue` game from current state of `restart`.
Also control window displays some statistic info current simulation: total time in game, current
frame rate, score and status. Selected scenario and algorithm also displayed.
![rfsim](https://raw.githubusercontent.com/EgorOrachyov/RobotFootballSim/main/docs/pictures/rfsim_ingame.png)

Simulator provides some basic settings tool, which allows to tweak drawing settings such as 
collisions/out info display, shadows settings (including the Sun position), field customisation color
and debug info window show/hide.
![rfsim](https://raw.githubusercontent.com/EgorOrachyov/RobotFootballSim/main/docs/pictures/rfsim_settings.png)

Debug info provides current actual information about game state. In includes all dynamic objects
properties: position, velocity, rotation (angle, the same as direction), and motors forces.
![rfsim](https://raw.githubusercontent.com/EgorOrachyov/RobotFootballSim/main/docs/pictures/rfsim_debug_info.png)

## Directory structure

```
RobotFootballSim
├── docs - documents, text files and various helpful stuff
├── resource - data used by simulator at runtime
├── rfsim - simulator core source code
│   ├── include - library public C API for algorithm plugins
│   ├── plugins - algorithms for robots (loaded in runtime as shared library)
│   ├── sources - source-code for implementation
│   │   ├── graphics - graphics server and image io for displaying actual game
│   │   ├── gui - user application sources
│   │   ├── logic - game logic, algorithms and managers
│   │   ├── opengl - low-level gl drawing stuff
│   │   ├── physics - game physics simulation source code
│   │   ├── scenario - scenario collectio for dirrent games setup
│   │   └── shaders - glsl low-level drawing shaders code
│   └── CMakeLists.txt - actual sim build config
├── deps - project dependencies
│   ├── box2d - physics engine 
│   ├── dynalo2 - patched dynalo lib for cross-platform shared lib management  
│   ├── glew - opengl loader
│   ├── glfw - cross-platform native windowing manager
│   ├── glm - 3d math 
│   ├── imgui - cross-platform GUI framework
│   └── stb - collection of cross-platfom stb-based image utils 
└── CMakeLists.txt - project cmake config, add this as sub-directory to your project
```

## License

This project is licensed under MIT license. License text can be found in the 
[license file](https://github.com/EgorOrachyov/RobotFootballSim/blob/main/LICENSE.md).

## Contributors

## Also