cmake_minimum_required(VERSION 3.11)
project(rfsim-app LANGUAGES CXX)

set(RFSIM_C_API
        include/rfsim/rfsim.h)

set(RFSIM_SOURCES
        sources/Main.cpp
        sources/Simulator.cpp
        sources/Simulator.hpp)

set(RFSIM_GRAPHICS_SOURCES
        sources/graphics/Window.cpp
        sources/graphics/Window.hpp
        sources/graphics/WindowManager.cpp
        sources/graphics/WindowManager.hpp
        sources/graphics/Painter.cpp
        sources/graphics/Painter.hpp
        sources/graphics/PainterItems.hpp
        sources/graphics/GraphicsServer.cpp
        sources/graphics/GraphicsServer.hpp
        sources/graphics/GraphicsSettings.hpp
        sources/graphics/GraphicsGameState.hpp
        sources/graphics/GraphicsSceneSettings.hpp
        sources/graphics/Image.cpp
        sources/graphics/Image.hpp)

set(RFSIM_OPENGL_SOURCES
        sources/opengl/GLDynamicBuffer.hpp
        sources/opengl/GLDynamicGeometry.cpp
        sources/opengl/GLDynamicGeometry.hpp
        sources/opengl/GLGeometryLayout.hpp
        sources/opengl/GLShader.cpp
        sources/opengl/GLShader.hpp
        sources/opengl/GLTexture.cpp
        sources/opengl/GLTexture.hpp)

set(RFSIM_SHADERS
        sources/shaders/ImageDraw.hpp
        sources/shaders/RectDraw.hpp)

set(RFSIM_PHYSICS_SOURCES
        sources/physics/PhysicsServer.cpp
        sources/physics/PhysicsServer.hpp
        sources/physics/PhysicsGameProperties.hpp
        sources/physics/PhysicsGameInitInfo.hpp
        sources/physics/PhysicsGameState.hpp)

set(RFSIM_LOGIC_SOURCES
        sources/logic/Game.hpp
        sources/logic/GameCommon.hpp
        sources/logic/Algorithm.cpp
        sources/logic/Algorithm.hpp
        sources/logic/AlgorithmManager.cpp
        sources/logic/AlgorithmManager.hpp
        sources/logic/GameManager.cpp
        sources/logic/GameManager.hpp
        sources/logic/GameScenario.hpp)

set(RFSIM_GUI_SOURCES
        sources/gui/GuiSimulator.cpp
        sources/gui/GuiSimulator.hpp
        sources/gui/GuiStyle.hpp
        sources/gui/GuiMenuBar.cpp
        sources/gui/GuiMenuBar.hpp)

set(RFSIM_SCENARIO_SOURCES
        sources/scenario/Scrum.hpp
        sources/scenario/ScenarioCommon.hpp sources/scenario/Duel.hpp)

set(RSIM_UTILS_SOURCES
        sources/utils/CircularBuffer.hpp
        sources/utils/ConfigManager.hpp
        sources/utils/ConfigManager.cpp)

add_executable(rfsim
        ${RFSIM_C_API}
        ${RFSIM_SOURCES}
        ${RFSIM_GRAPHICS_SOURCES}
        ${RFSIM_OPENGL_SOURCES}
        ${RFSIM_SHADERS}
        ${RFSIM_PHYSICS_SOURCES}
        ${RFSIM_LOGIC_SOURCES}
        ${RFSIM_GUI_SOURCES}
        ${RFSIM_SCENARIO_SOURCES}
        ${RSIM_UTILS_SOURCES})

target_include_directories(rfsim PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)
target_include_directories(rfsim PRIVATE ${CMAKE_CURRENT_LIST_DIR}/sources)

set(DEFINES_LIST RFSIM_DEBUG)

if(APPLE)
    list(APPEND DEFINES_LIST RFSIM_PLATFORM_MACOS)
elseif(UNIX)
    list(APPEND DEFINES_LIST RFSIM_PLATFORM_LINUX)
elseif(WIN32)
    list(APPEND DEFINES_LIST RFSIM_PLATFORM_WIN)
else()
    message(FATAL_ERROR "Platform not specified")
endif()

foreach(DEFINE ${DEFINES_LIST})
    target_compile_definitions(rfsim PUBLIC ${DEFINE})
endforeach()

set_target_properties(rfsim PROPERTIES CXX_STANDARD 14)
set_target_properties(rfsim PROPERTIES CXX_STANDARD_REQUIRED ON)

target_link_libraries(rfsim PRIVATE imgui)
target_link_libraries(rfsim PRIVATE box2d)
target_link_libraries(rfsim PRIVATE stb_image)
target_link_libraries(rfsim PRIVATE glm)
target_link_libraries(rfsim PRIVATE dynalo)
target_link_libraries(rfsim PRIVATE picojson)

add_library(randommove SHARED plugins/RandomMovement.cpp)
target_include_directories(randommove PRIVATE include)
target_link_libraries(randommove glm)

add_library(followmove SHARED plugins/BallFollowMovement.cpp)
target_include_directories(followmove PRIVATE include)
target_link_libraries(followmove glm)

message(STATUS "${RESOURCES_PATH}")
file(COPY ${RESOURCES_PATH} DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
file(COPY ${CONFIG_PATH} DESTINATION ${CMAKE_CURRENT_BINARY_DIR})