######################################################
#  Title: Dynamic Control for Robotics (Expedition)  #
######################################################

cmake_minimum_required(VERSION 3.1)
project(Humanoid_2018)

# Use MACOSX_RPATH by default on OS X. This was added in CMake 2.8.12 and
# became default in CMake 3.0. Explicitly setting this policy is necessary to
# suppress a warning in CMake 3.0 and above.
if(POLICY CMP0042)
  cmake_policy(SET CMP0042 NEW)
endif()

# Simplify variable reference and escape sequence evaluation. This was added in
# CMake 3.1. Explicitly setting this policy is necessary to suppress a warning
# in CMake 3.1 and above.
if(POLICY CMP0053)
  cmake_policy(SET CMP0053 NEW)
endif()

add_definitions(-O3)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

#------------------------------------------------------------------------
#     Build Setup
#------------------------------------------------------------------------
set(INSTALL_LIB_DIR ${CMAKE_INSTALL_PREFIX}/lib )
set(INSTALL_BIN_DIR ${CMAKE_INSTALL_PREFIX}/bin )
set(INSTALL_INCLUDE_DIR ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME})

find_package(PkgConfig)
pkg_check_modules(EIGEN3 REQUIRED eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "-std=c++11")

include_directories ("/usr/local/include")
include_directories ("${PROJECT_SOURCE_DIR}/")
include_directories ("${PROJECT_SOURCE_DIR}/DynaController")
include_directories ("${PROJECT_SOURCE_DIR}/RobotSystems")
include_directories ("${PROJECT_SOURCE_DIR}/ExternalSource")
include_directories ("${PROJECT_SOURCE_DIR}/ExternalSource/ParamHandler/include")
include_directories ("${PROJECT_SOURCE_DIR}/WBC")
include_directories ("${PROJECT_SOURCE_DIR}/Simulator")
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

#===============================================================================
# Build type settings
#===============================================================================
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build, options are: Debug | Release | RelWithDebInfo | MinSizeRel" FORCE)
endif()
string(TOUPPER ${CMAKE_BUILD_TYPE} CMAKE_BUILD_TYPE_UPPERCASE)

set(BUILD_TYPE_DEBUG FALSE)
set(BUILD_TYPE_RELEASE FALSE)
set(BUILD_TYPE_RELWITHDEBINFO FALSE)
set(BUILD_TYPE_MINSIZEREL FALSE)

if("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "DEBUG")
  set(BUILD_TYPE_DEBUG TRUE)
elseif("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "RELEASE")
  set(BUILD_TYPE_RELEASE TRUE)
elseif("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "RELWITHDEBINFO")
  set(BUILD_TYPE_RELWITHDEBINFO TRUE)
elseif("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "MINSIZEREL")
  set(BUILD_TYPE_MINSIZEREL TRUE)
else()
  build_error("CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} unknown. Valid options are: Debug | Release | RelWithDebInfo | MinSizeRel")
endif()
:;;lsflsfuuuuuu-----------------------------------------------------
#    Directory Selection
#------------------------------------------------------------------------
add_subdirectory (Simulator)
add_subdirectory (Utils)
add_subdirectory (Filter)
add_subdirectory (ExternalSource)
add_subdirectory (RobotSystems)
add_subdirectory (WBC)
add_subdirectory (DynaController)
add_subdirectory (Planner)

#------------------------------------------------------------------------
# Configuration
#------------------------------------------------------------------------
SET (THIS_COM "${PROJECT_SOURCE_DIR}/" )
CONFIGURE_FILE(${PROJECT_SOURCE_DIR}/config.h.cmake ${PROJECT_SOURCE_DIR}/Configuration.h)


#------------------------------------------------------------------------
# Printout
#------------------------------------------------------------------------
message ("")
message ("-[ MAKE INFORMATION ]----------------------------------------")
message (" The project name: ${PROJECT_NAME}")
message (" The version: ${${PROJECT_NAME}_VERSION}")
message (" The project source directory: ${PROJECT_SOURCE_DIR}")
message (" The project binary directory: ${PROJECT_BINARY_DIR}")
message (" The install destination: ${CMAKE_INSTALL_PREFIX}")
message ("--------------------------------------------------------------")
message ("")

install(FILES ${PROJECT_SOURCE_DIR}/Configuration.h DESTINATION "${INSTALL_INCLUDE_DIR}")
