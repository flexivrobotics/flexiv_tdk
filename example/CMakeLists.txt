cmake_minimum_required(VERSION 3.16.3)

project(flexiv_tdk-examples)

message("OS: ${CMAKE_SYSTEM_NAME}")
message("Processor: ${CMAKE_SYSTEM_PROCESSOR}")

# Show verbose build info
SET(CMAKE_VERBOSE_MAKEFILE ON)

# Configure build type
if(NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE Release CACHE STRING "CMake build type" FORCE) 
endif()

# Set default install prefix
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    message(FATAL_ERROR "CMAKE_INSTALL_PREFIX must be set to a non-default value.
   Please set it explicitly using -DCMAKE_INSTALL_PREFIX=<path-to-your-install-directory> or using cmake-gui")
endif()

# Add to CMAKE_PREFIX_PATH so the sub-dependencies can also be found
list(APPEND CMAKE_PREFIX_PATH ${CMAKE_INSTALL_PREFIX})

set(EXAMPLE_LIST
   basics1_cartesian_teleop
)

# Find flexiv_tdk INTERFACE library
find_package(flexiv_tdk 1.1.0 REQUIRED)
if(flexiv_tdk_FOUND)
   message(STATUS "Found flexiv_tdk:${flexiv_tdk_DIR}") 
endif()

# Build all examples
foreach(example ${EXAMPLE_LIST})
   add_executable(${example} ${example}.cpp) 
   target_link_libraries(${example} flexiv::flexiv_tdk)

     # C++17 required
   set_target_properties(${example} PROPERTIES
   CXX_STANDARD 17
   CXX_STANDARD_REQUIRED ON)
endforeach()
