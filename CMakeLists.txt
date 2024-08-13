cmake_minimum_required(VERSION 3.16.3)

# ===================================================================
#      PROJECT SETUP
# ===================================================================
project(flexiv_tdk VERSION 1.1.0)

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
    set(TELEOP_STATIC_LIBRARY "${CMAKE_CURRENT_SOURCE_DIR}/lib/libflexiv_tdk.x86_64-linux-gnu.a")
  else()
    message(FATAL_ERROR "Linux with ${CMAKE_SYSTEM_PROCESSOR} processor is currently not supported.")
  endif()
else()
  message(FATAL_ERROR "OS:${CMAKE_SYSTEM_NAME} with processor:${CMAKE_SYSTEM_PROCESSOR} is currently not supported.")
endif()

# Set install prefix to non default
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    message(FATAL_ERROR "CMAKE_INSTALL_PREFIX must be set to a non-default value.
    Please set it explicitly using -DCMAKE_INSTALL_PREFIX=<path-to-your-install-directory> or using cmake-gui")
endif()
# Add to CMAKE_PREFIX_PATH so the sub-dependencies can also be found
list(APPEND CMAKE_PREFIX_PATH ${CMAKE_INSTALL_PREFIX})

# ===================================================================
#      PROJECT DEPENDENCIES
# ===================================================================

# OpenSSL
find_package(OpenSSL REQUIRED)
if(OpenSSL_FOUND)
  message(STATUS "Found OpenSSL, version: ${OPENSSL_VERSION}, libraries: ${OPENSSL_LIBRARIES}")
endif()

# flexiv_rdk
find_package(flexiv_rdk REQUIRED)
if(flexiv_rdk_FOUND)
  message(STATUS "Found flexiv_rdk: ${flexiv_rdk_DIR}")
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
  ${TELEOP_STATIC_LIBRARY}
  ${OPENSSL_LIBRARIES}
  flexiv::flexiv_rdk
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
