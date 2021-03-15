cmake_minimum_required(VERSION 3.11)
project(rfsim LANGUAGES CXX)

# Configure dependencies
message(STATUS "Add GLFW as window system API")
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
add_subdirectory(deps/glfw)

message(STATUS "Add GLEW as gl loader")
set(glew-cmake_BUILD_SHARED ON CACHE BOOL "" FORCE)
set(glew-cmake_BUILD_STATIC OFF CACHE BOOL "" FORCE)
set(USE_GLU OFF CACHE BOOL "" FORCE)
set(PKG_CONFIG_REPRESENTATIVE_TARGET OFF CACHE BOOL "" FORCE)
set(ONLY_LIBS ON CACHE BOOL "" FORCE)
add_subdirectory(deps/glew)

message(STATUS "Add box2d as 2d physics simulation engine")
set(BOX2D_BUILD_UNIT_TESTS OFF CACHE BOOL "" FORCE)
set(BOX2D_BUILD_TESTBED OFF CACHE BOOL "" FORCE)
set(BOX2D_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(BOX2D_USER_SETTINGS OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS ON CACHE BOOL "" FORCE)
add_subdirectory(deps/box2d)

message(STATUS "Add imgui as library for GUI elements")
add_library(imgui INTERFACE)
target_include_directories(imgui INTERFACE deps/imgui)
target_include_directories(imgui INTERFACE deps/imgui/backends)

message(STATUS "Add stb image for image data processing")
add_subdirectory(deps/stb)

message(STATUS "Add glm for 3d math functions")
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(BUILD_STATIC_LIBS OFF CACHE BOOL "" FORCE)
add_subdirectory(deps/glm)
target_include_directories(glm INTERFACE deps/glm)

# Copy resources files into binary dir
file(COPY resources DESTINATION ${CMAKE_BINARY_DIR})

# Actual rfsim application
add_subdirectory(rfsim)