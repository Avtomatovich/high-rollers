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

include_directories(src)
include_directories(deps)



# Specifies .cpp and .h files to be passed to the compiler
add_executable(${PROJECT_NAME}
    main.cpp
    tiny_obj_loader.h
    meshloader.h
    meshloader.cpp
    gaussmap.h
    gaussmap.cpp
)

set(BUILD_STATIC_LIBS ON CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(WITH_QHULL_TESTS OFF CACHE BOOL "" FORCE)
set(WITH_QHULL_TOOLS OFF CACHE BOOL "" FORCE)

add_subdirectory(deps/geometry-central)
add_subdirectory(deps/polyscope)
set(CMAKE_AUTOMOC OFF)
add_subdirectory(deps/qhull)
set(CMAKE_AUTOMOC ON)

# Specifies libraries to be linked (Qt components, glew, etc)
# Specifies .cpp and .h files to be passed to the compiler
add_executable(${PROJECT_NAME}
    src/main.cpp

    src/util/quickhull/ConvexHull.hpp
    src/util/quickhull/Quickhull.hpp

    src/util/quickhull/Quickhull.cpp

    src/mesh.h 
    src/mesh.cpp
    src/gaussmap.h 
    src/gaussmap.cpp
)

add_subdirectory(deps/eigen)
add_subdirectory(deps/geometry-central)
add_subdirectory(deps/polyscope)

# Specifies libraries to be linked (Qt components, etc)
target_link_libraries(${PROJECT_NAME} PRIVATE
    Qt::Core
    Qt::Gui
    Qt::Xml
    geometry-central
    polyscope
    qhullcpp
    qhull_r
)


# This allows you to `#include "Eigen/..."`
target_include_directories(${PROJECT_NAME} PRIVATE BEFORE
    deps/eigen
    deps/geometry-central
    deps/qhull/src/libqhullcpp
    deps/qhull/src
)

target_include_directories(high-rollers PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/")
target_include_directories(high-rollers PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/deps/polyscope/deps/args")
    Eigen3::Eigen
    geometry-central
    polyscope
)

# This allows you to include libraries
target_include_directories(${PROJECT_NAME} PRIVATE
    src
)

# Set this flag to silence warnings on MacOS
if (APPLE)
  set(CMAKE_CXX_FLAGS "-Wno-deprecated-volatile")
endif()
