# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "/Users/marissashaffer/Desktop/cs/cs2240/high-rollers/build/build-mesh-Qt_6_9_2_for_macOS_qt_qt6-Release/deps/geometry-central/deps/eigen-src")
  file(MAKE_DIRECTORY "/Users/marissashaffer/Desktop/cs/cs2240/high-rollers/build/build-mesh-Qt_6_9_2_for_macOS_qt_qt6-Release/deps/geometry-central/deps/eigen-src")
endif()
file(MAKE_DIRECTORY
  "/Users/marissashaffer/Desktop/cs/cs2240/high-rollers/build/build-mesh-Qt_6_9_2_for_macOS_qt_qt6-Release/deps/geometry-central/deps/eigen-build"
  "/Users/marissashaffer/Desktop/cs/cs2240/high-rollers/build/build-mesh-Qt_6_9_2_for_macOS_qt_qt6-Release/_deps/eigen-subbuild/eigen-populate-prefix"
  "/Users/marissashaffer/Desktop/cs/cs2240/high-rollers/build/build-mesh-Qt_6_9_2_for_macOS_qt_qt6-Release/_deps/eigen-subbuild/eigen-populate-prefix/tmp"
  "/Users/marissashaffer/Desktop/cs/cs2240/high-rollers/build/build-mesh-Qt_6_9_2_for_macOS_qt_qt6-Release/_deps/eigen-subbuild/eigen-populate-prefix/src/eigen-populate-stamp"
  "/Users/marissashaffer/Desktop/cs/cs2240/high-rollers/build/build-mesh-Qt_6_9_2_for_macOS_qt_qt6-Release/_deps/eigen-subbuild/eigen-populate-prefix/src"
  "/Users/marissashaffer/Desktop/cs/cs2240/high-rollers/build/build-mesh-Qt_6_9_2_for_macOS_qt_qt6-Release/_deps/eigen-subbuild/eigen-populate-prefix/src/eigen-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/marissashaffer/Desktop/cs/cs2240/high-rollers/build/build-mesh-Qt_6_9_2_for_macOS_qt_qt6-Release/_deps/eigen-subbuild/eigen-populate-prefix/src/eigen-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/marissashaffer/Desktop/cs/cs2240/high-rollers/build/build-mesh-Qt_6_9_2_for_macOS_qt_qt6-Release/_deps/eigen-subbuild/eigen-populate-prefix/src/eigen-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
