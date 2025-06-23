cmake_minimum_required(VERSION 3.16.3)
project(flexiv_tdk-examples)

# Show verbose build info
SET(CMAKE_VERBOSE_MAKEFILE ON)

message("OS: ${CMAKE_SYSTEM_NAME}")
message("Processor: ${CMAKE_SYSTEM_PROCESSOR}")

# Configure build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "CMake build type" FORCE)
endif()

# Example list
set(EXAMPLE_LIST
  transparent_cartesian_teleop_lan
  transparent_cartesian_teleop_wan
  cartesian_teleop_under_lan
  joint_teleop_over_wan
  joint_teleop_under_lan
)

# Find flexiv_tdk INTERFACE library
find_package(flexiv_tdk REQUIRED)

# Build all selected examples
foreach(example ${EXAMPLE_LIST})
  add_executable(${example} ${example}.cpp)
  target_link_libraries(${example} flexiv::flexiv_tdk)

  # C++17 required
  set_target_properties(${example} PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON)
endforeach()
