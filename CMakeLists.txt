cmake_minimum_required(VERSION 3.16)

# Sets project name
project(high-rollers LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# Sets C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Specifies required Qt components
find_package(Qt6 REQUIRED COMPONENTS Core)
find_package(Qt6 REQUIRED COMPONENTS Gui)
find_package(Qt6 REQUIRED COMPONENTS Xml)

# Specifies .cpp and .h files to be passed to the compiler
add_executable(${PROJECT_NAME}
    src/main.cpp

    src/util/quickhull/ConvexHull.hpp
    src/util/quickhull/Quickhull.hpp

    src/util/quickhull/Quickhull.cpp

    src/mesh.h 
    src/gaussmap.h
    src/unionfind.h
    src/sim.h

    src/mesh.cpp
    src/gaussmap.cpp
    src/unionfind.cpp
    src/sim.cpp
)

# set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
# set(BUILD_UNIT_TESTS OFF CACHE BOOL "" FORCE)
# set(BUILD_BULLET2_DEMOS OFF CACHE BOOL "" FORCE)
# set(BUILD_EXTRAS OFF CACHE BOOL "" FORCE)

add_subdirectory(deps/eigen)
add_subdirectory(deps/geometry-central)
add_subdirectory(deps/polyscope)
add_subdirectory(deps/bullet3)

# Specifies libraries to be linked (Qt components, etc)
target_link_libraries(${PROJECT_NAME} PRIVATE
    Qt::Core
    Qt::Gui
    Qt::Xml
    Eigen3::Eigen
    geometry-central
    polyscope
    BulletDynamics
    BulletCollision
    LinearMath
)

# This allows you to include libraries
target_include_directories(${PROJECT_NAME} PRIVATE
    src
    # deps/bullet3/src
)

# Set this flag to silence warnings on Windows
if (MSVC OR MSYS OR MINGW)
  set(CMAKE_CXX_FLAGS "-Wno-volatile")
endif()

# Set this flag to silence warnings on MacOS
if (APPLE)
  set(CMAKE_CXX_FLAGS "-Wno-deprecated-volatile")
endif()
