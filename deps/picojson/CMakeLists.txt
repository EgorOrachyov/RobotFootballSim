cmake_minimum_required(VERSION 3.11)
project(picojson)

set(PICOJSON_IMAGE_SOURCES
        include/picojson/picojson.h
        sources/picojson.cpp)

add_library(picojson STATIC ${PICOJSON_IMAGE_SOURCES})
target_include_directories(picojson PUBLIC include/)
set_target_properties(picojson PROPERTIES CXX_STANDARD 14)
set_target_properties(picojson PROPERTIES CXX_STANDARD_REQUIRED ON)