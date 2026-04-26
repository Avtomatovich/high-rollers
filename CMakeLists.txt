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
    src/mesh.cpp
    src/gaussmap.h 
    src/gaussmap.cpp
)

add_subdirectory(deps/eigen)
add_subdirectory(deps/geometry-central)
add_subdirectory(deps/polyscope)

find_package(Bullet REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE
    Qt::Core
    Qt::Gui
    Qt::Xml
    Eigen3::Eigen
    geometry-central
    polyscope
    ${BULLET_LIBRARIES}
)

# This allows you to include libraries
target_include_directories(${PROJECT_NAME} PRIVATE
    src
    ${BULLET_INCLUDE_DIRS}
)


# Set this flag to silence warnings on Windows
if (MSVC OR MSYS OR MINGW)
  set(CMAKE_CXX_FLAGS "-Wno-volatile")
endif()

# Set this flag to silence warnings on MacOS
if (APPLE)
  set(CMAKE_CXX_FLAGS "-Wno-deprecated-volatile")
endif()
