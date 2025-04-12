cmake_minimum_required(VERSION 3.16)
project(unitree_mujoco)

# Configuración del modo Debug
option(ENABLE_DEBUG "Enable debug mode" On)

if (ENABLE_DEBUG)
    message(STATUS "Debug mode enabled")
    set(CMAKE_BUILD_TYPE Debug)
    add_definitions(-DDEBUG)
else()
    message(STATUS "Debug mode disabled")
    set(CMAKE_BUILD_TYPE Release)
endif()

enable_language(C)
enable_language(CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(mujoco REQUIRED)
find_package(unitree_sdk2 REQUIRED)


file(GLOB SIM_SRC
    src/joystick/joystick.cc
    src/mujoco/*.cc
    src/unitree_sdk2_bridge/*.cc)

set(SIM_DEPENDENCIES
    pthread
    mujoco::mujoco
    glfw
    yaml-cpp
    unitree_sdk2)


add_executable(unitree_mujoco ${SIM_SRC} src/main.cc)
target_link_libraries(unitree_mujoco ${SIM_DEPENDENCIES})

add_executable(test test/test_unitree_sdk2.cpp)
target_link_libraries(test unitree_sdk2)

add_executable(jstest src/joystick/jstest.cc src/joystick/joystick.cc)
