![rfsim](https://raw.githubusercontent.com/EgorOrachyov/RobotFootballSim/main/docs/pictures/rfsim_app_concept.png)

# Robot Football Simulator

Robot football simulator with graphical user interface and the feature
to develop and run algorithms, required to control robots, test
various game strategies and game mechanics.

### Features

- [ ] Dynamic algorithms loading
- [ ] Simulated 2d physics of the game
- [ ] 2d rendering for the football game
- [ ] Game logic and rules 
- [ ] GUI application, widgets and menus

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
cmake --build . --target all -j `nproc`
```

Alternatively import project into desired IDE (import as cmake project).

## License

This project is licensed under MIT license. License text can be found in the 
[license file](https://github.com/EgorOrachyov/RobotFootballSim/blob/main/LICENSE.md).

## Contributors

## Also