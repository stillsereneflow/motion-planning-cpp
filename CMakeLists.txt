cmake_minimum_required(VERSION 3.20)

project(motion_planning_cpp
  VERSION 0.1.0
  DESCRIPTION "Motion planning algorithms for autonomous driving in modern C++"
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(MPL_BUILD_TESTS "Build unit tests" ON)
option(MPL_BUILD_APPS "Build demo applications" ON)

if(MSVC)
  add_compile_options(/W4 /permissive-)
else()
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_library(mpl
  src/distance_field.cpp
  src/grid_map.cpp
  src/planners/astar.cpp
  src/planners/hybrid_astar.cpp
  src/reeds_shepp.cpp
  src/vehicle.cpp)
target_include_directories(mpl PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

if(MPL_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

if(MPL_BUILD_APPS)
  add_subdirectory(apps)
endif()
