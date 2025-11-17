cmake_minimum_required(VERSION 3.16.3)

# ===================================================================
#      PROJECT SETUP
# ===================================================================
project(flexiv_tdk VERSION 1.5.0 LANGUAGES CXX)

# Configure build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "CMake build type" FORCE)
endif()
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Release" "Debug" "RelWithDebInfo" "MinSizeRel")

# Set static library according to platform
message(STATUS "OS: ${CMAKE_SYSTEM_NAME}")
message(STATUS "Processor: ${CMAKE_SYSTEM_PROCESSOR}")
if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
    set(TDK_LIB "libflexiv_tdk.x86_64-linux-gnu.a")
  elseif(${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
    set(TDK_LIB "libflexiv_tdk.aarch64-linux-gnu.a")
  else()
    message(FATAL_ERROR "Linux with ${CMAKE_SYSTEM_PROCESSOR} processor is currently not supported.")
  endif()
else()
  message(FATAL_ERROR "${CMAKE_SYSTEM_NAME} is currently not supported.")
endif()

# Read expected SHA256 hash
file(READ ${CMAKE_CURRENT_SOURCE_DIR}/lib/${TDK_LIB}.sha256 EXPECTED_HASH_RAW)
string(STRIP "${EXPECTED_HASH_RAW}" EXPECTED_HASH)
string(LENGTH "${EXPECTED_HASH}" HASH_LEN)
if(NOT HASH_LEN EQUAL 64)
    message(FATAL_ERROR "Invalid SHA256 hash length: ${HASH_LEN}. Expected 64. Value: '${EXPECTED_HASH}'")
endif()

# Download the library file from GitHub 
set(TDK_LIB_URL https://github.com/flexivrobotics/flexiv_tdk/releases/download/v1.5/${TDK_LIB})
message(STATUS "Downloading ${TDK_LIB_URL} ...")
file(DOWNLOAD ${TDK_LIB_URL} ${CMAKE_CURRENT_BINARY_DIR}/${TDK_LIB}
     SHOW_PROGRESS
     TLS_VERIFY ON
     EXPECTED_HASH SHA256=${EXPECTED_HASH}
     STATUS status)
list(GET status 0 code)
if(code EQUAL 0)
  message(STATUS "Download finished")
else()
  message(FATAL_ERROR "Download failed: ${status}")
endif()

# ===================================================================
#      DEPENDENCIES
# ===================================================================
# Threads
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
message(STATUS "Found Threads")

# Flexiv RDK
find_package(flexiv_rdk 1.8.0 REQUIRED)
message(STATUS "Found flexiv_rdk: ${flexiv_rdk_DIR}, version: ${flexiv_rdk_VERSION}")

# ===================================================================
#      CREATE LIBRARY
# ===================================================================
# Create static library target with dummy source
add_library(${PROJECT_NAME} STATIC ${CMAKE_CURRENT_SOURCE_DIR}/lib/dummy.cpp)

# Create an alias of the target with flexiv namespace
add_library(flexiv::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

# Set include directories
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link to dependencies
target_link_libraries(${PROJECT_NAME} PUBLIC
  Threads::Threads
  flexiv::flexiv_rdk
)

# Use moderate compiler warning option 
if(CMAKE_HOST_UNIX)
  target_compile_options(${PROJECT_NAME} PUBLIC -Wall -Wextra)
else()
  target_compile_options(${PROJECT_NAME} PUBLIC /W1)
endif()

# Install the library
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/FlexivInstallLibrary.cmake)
FlexivInstallLibrary()
