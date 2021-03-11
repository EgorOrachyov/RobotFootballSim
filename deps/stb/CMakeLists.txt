cmake_minimum_required(VERSION 3.17)
project(stb)

set(STB_IMAGE_SOURCES
        include/stb/stb_image.hpp
        include/stb/stb_image_resize.hpp
        include/stb/stb_image_write.hpp
        sources/stb_image.cpp
        sources/stb_image_resize.cpp
        sources/stb_image_write.cpp)

add_library(stb_image STATIC ${STB_IMAGE_SOURCES})
target_include_directories(stb_image PUBLIC include/)
set_target_properties(stb_image PROPERTIES CXX_STANDARD 17)
set_target_properties(stb_image PROPERTIES CXX_STANDARD_REQUIRED ON)