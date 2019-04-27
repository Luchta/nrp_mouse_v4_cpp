cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")


project(MauSim)

# Useful CMake options for Qt projects
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

# Search desired Qt packages
#find_package(Qt5Core REQUIRED)

# Create a list with all .cpp source files
set( project_sources
   main.cpp
   kinematics.cpp
   RPI.cpp
   Mouse_UI.cpp
   Mouse_COM.cpp
)

add_executable(${PROJECT_NAME} ${project_sources})
