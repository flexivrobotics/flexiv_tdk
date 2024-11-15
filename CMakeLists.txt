cmake_minimum_required(VERSION 3.16.3)

# ===================================================================
#      PROJECT SETUP
# ===================================================================
project(flexiv_tdk VERSION 1.2.0)

# Configure build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "CMake build type" FORCE)
endif()
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Release" "Debug" "RelWithDebInfo")

# Set static library
message("OS: ${CMAKE_SYSTEM_NAME}")
message("Processor: ${CMAKE_SYSTEM_PROCESSOR}")
if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
    set(TDK_STATIC_LIB "libflexiv_tdk.x86_64-linux-gnu.a")
  elseif(${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
    set(TDK_STATIC_LIB "libflexiv_tdk.aarch64-linux-gnu.a")
  else()
    message(FATAL_ERROR "Linux with ${CMAKE_SYSTEM_PROCESSOR} processor is currently not supported.")
  endif()
endif()

# ===================================================================
#      PROJECT DEPENDENCIES
# ===================================================================
# Threads
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
if(Threads_FOUND)
  message(STATUS "Found Threads: HAVE_PTHREAD = ${THREADS_HAVE_PTHREAD_ARG}")
endif()

# spdlog
find_package(spdlog REQUIRED)
if(spdlog_FOUND)
  message(STATUS "Found spdlog: ${spdlog_DIR}")
endif()

# Fast-DDS (Fast-RTPS)
find_package(fastrtps 2.6.7 REQUIRED)
if(fastrtps_FOUND)
  message(STATUS "Found fastrtps: ${fastrtps_DIR}")
endif()

# ===================================================================
#      CREATE LIBRARY
# ===================================================================
# Create an INTERFACE library with no source file to compile
add_library(${PROJECT_NAME} INTERFACE)

# Create an alias of the library using flexiv namespace, 
# to imitate the install target which uses flexiv namespace.
add_library(flexiv::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} INTERFACE 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/${TDK_STATIC_LIB}
  Threads::Threads
  spdlog::spdlog
  fastrtps
)

# Use moderate compiler warning option 
if(CMAKE_HOST_UNIX)
  target_compile_options(${PROJECT_NAME} INTERFACE -Wall -Wextra)
else()
  target_compile_options(${PROJECT_NAME} INTERFACE /W1)
endif()

# Install the INTERFACE library
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/FlexivInstallLibrary.cmake)
FlexivInstallLibrary()
