cmake_minimum_required(VERSION 3.11)
project(rfsim-app LANGUAGES CXX)

set(RFSIM_SOURCES
        sources/Main.cpp
        sources/Simulator.cpp
        sources/Simulator.hpp
        sources/GameState.hpp
        )

set(RFSIM_GRAPHICS_SOURCES
        sources/graphics/Window.cpp
        sources/graphics/Window.hpp
        sources/graphics/WindowManager.cpp
        sources/graphics/WindowManager.hpp
        sources/graphics/PainterEngine.cpp
        sources/graphics/PainterEngine.hpp
        sources/graphics/GraphicsServer.cpp
        sources/graphics/GraphicsServer.hpp
        sources/graphics/Image.cpp
        sources/graphics/Image.hpp
        sources/graphics/PainterItems.hpp)

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

add_executable(rfsim
        ${RFSIM_SOURCES}
        ${RFSIM_GRAPHICS_SOURCES}
        ${RFSIM_OPENGL_SOURCES}
        ${RFSIM_SHADERS})

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

target_link_libraries(rfsim PRIVATE glfw)
target_link_libraries(rfsim PRIVATE libglew_shared)
target_link_libraries(rfsim PRIVATE box2d)
target_link_libraries(rfsim PRIVATE imgui)
target_link_libraries(rfsim PRIVATE stb_image)
target_link_libraries(rfsim PRIVATE glm)
