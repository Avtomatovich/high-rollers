# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "Release")
  file(REMOVE_RECURSE
  "CMakeFiles/high-rollers_autogen.dir/AutogenUsed.txt"
  "CMakeFiles/high-rollers_autogen.dir/ParseCache.txt"
  "high-rollers_autogen"
  "pmp-library/src/CMakeFiles/geometry-central_autogen.dir/AutogenUsed.txt"
  "pmp-library/src/CMakeFiles/geometry-central_autogen.dir/ParseCache.txt"
  "pmp-library/src/geometry-central_autogen"
  )
endif()
