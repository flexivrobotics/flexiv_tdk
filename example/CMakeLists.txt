cmake_minimum_required(VERSION 3.18.6)

project(flexiv_omni_teleop_example)

SET(CMAKE_VERBOSE_MAKEFILE ON)

if(NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE Release CACHE STRING "CMake build type" FORCE) 
endif()

set(EXAMPLE_LIST
    basic_free_space_motion_teleop
)

find_package(flexiv_omni_teleop REQUIRED)
if(flexiv_omni_teleop_FOUND)
   message(STATUS "Found flexiv_omni_teleop:${flexiv_omni_teleop_DIR}") 
endif()


foreach(example ${EXAMPLE_LIST})
   add_executable(${example} ${example}.cpp) 
   target_link_libraries(${example} flexiv::omni::flexiv_omni_teleop)
   set_target_properties(${example} PROPERTIES
   CXX_STANDARD 17
   CXX_STANDARD_REQUIRED ON)
endforeach()
