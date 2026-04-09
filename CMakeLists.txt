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
    main.cpp
    tiny_obj_loader.h
meshloader.h
meshloader.cpp
gaussmap.h gaussmap.cpp

)

# Disable PMP examples and tests to avoid name conflicts
set(PMP_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(PMP_BUILD_TESTS OFF CACHE BOOL "" FORCE)

# Disable geometry-central tests/examples if they conflict
set(GC_BUILD_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory(deps/pmp-library)
add_subdirectory(deps/geometry-central)

# Specifies libraries to be linked (Qt components, glew, etc)
target_link_libraries(${PROJECT_NAME} PRIVATE
    Qt::Core
    Qt::Gui
    Qt::Xml
pmp-library
    geometry-central
)


# This allows you to `#include "Eigen/..."`
target_include_directories(${PROJECT_NAME} PRIVATE
    deps/eigen
	
)
target_include_directories(high-rollers PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/")
# Set this flag to silence warnings on Windows
if (MSVC OR MSYS OR MINGW)
  set(CMAKE_CXX_FLAGS "-Wno-volatile")
endif()

# Set this flag to silence warnings on MacOS
if (APPLE)
  set(CMAKE_CXX_FLAGS "-Wno-deprecated-volatile")
endif()
