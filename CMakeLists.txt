cmake_minimum_required(VERSION 3.8)

# Set compiler
set(CMAKE_CXX_COMPILER "clang++")
set(CMAKE_C_COMPILER "clang")

# Set the build type
if(NOT CMAKE_BUILD_TYPE)
set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING
"Choose the type of build, options are: Debug Release RelWithDebInfo."
FORCE)
endif(NOT CMAKE_BUILD_TYPE)

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")


# --> Install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
    set(CMAKE_INSTALL_PREFIX "/usr")
endif()
message(STATUS "Install prefix: ${CMAKE_INSTALL_PREFIX}")

# Set compilation options
SET(CMAKE_CXX_FLAGS "" CACHE STRING "" FORCE)
SET(CMAKE_CXX_FLAGS_DEBUG  "-O0 -g" CACHE STRING "" FORCE) 
SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -g" CACHE STRING "" FORCE)
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG" CACHE STRING "" FORCE) 

# Set path to find additional dependencies
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
# set(CMAKE_MACOSX_RPATH 1)
# set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
# SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 
# set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Version of the std library
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Project infos
project(sdms
        VERSION 1.0
        LANGUAGES C CXX)

set(PROJECT_DESCRIPTION "Sequential Decision Making Problem Solver.")

# options for compilation
option(SDMS_BUILD_SDMS "Build SDM'Studio" ON)
option(SDMS_BUILD_DOCS "Build SDMS documentation" OFF)
option(SDMS_BUILD_TESTS "Build tests for SDMS" ON)

# Path to third party (can be defined if different from default) 
# --> Pytorch
if(NOT DEFINED CMAKE_PREFIX_PATH)
    set(CMAKE_PREFIX_PATH "/opt/libtorch")
endif()

# --> ILOG CPLEX
if(NOT DEFINED CPLEX_ROOT_DIR)
    set(CPLEX_ROOT_DIR "/opt/ibm/ILOG/CPLEX_Studio201/" CACHE STRING "Path to ILOG CPLEX root directory (default: '/opt/ibm/ILOG/CPLEX_Studio201/')")
endif()

# Set useful variables
set(SDMS_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/src)
set(SDMS_SOURCE_DIRS ${CMAKE_SOURCE_DIR}/src)
set(SDMS_LIB_DIR ${CMAKE_SOURCE_DIR}/lib)
set(SDMS_DOC_DIR ${CMAKE_SOURCE_DIR}/docs)
set(SDMS_DATA_DIR ${CMAKE_SOURCE_DIR}/data)
set(SDMS_MAN_DIR ${SDMS_DOC_DIR}/man)
set(SDMS_WORLD_DIR ${SDMS_DATA_DIR}/world)
set(SDMS_CONFIG_DIR ${SDMS_DATA_DIR}/config)

set(TOULBAR_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/include/tb2)
set(SPIRIT_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/include)


set(INCLUDE_DIR ${SDMS_INCLUDE_DIRS}/sdm)
set(SOURCE_DIR ${SDMS_SOURCE_DIRS}/sdm)
set(EXAMPLES_DIR ${SDMS_SOURCE_DIRS}/examples)
set(ALGOS_DIR ${SDMS_SOURCE_DIRS}/algos)


# Set names of libraries
set(LIB_SDMS "sdm")
set(LIB_SDMS_STATIC "${LIB_SDMS}_static")

# Build SDMS sources
if(SDMS_BUILD_SDMS)
    message(STATUS "Building SDMS sources.")
    add_subdirectory(src)
endif()

# Build documentation
if(SDMS_BUILD_DOCS)
    message(STATUS "Building SDMS documentation.")
    add_subdirectory(docs)
endif()

# Build tests
if(SDMS_BUILD_TESTS)
    message(STATUS "Building SDMS tests.")
    enable_testing ()
    add_subdirectory(tests)
endif()

# Configure sdms.pc file and install it
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/sdms.pc.in ${CMAKE_CURRENT_BINARY_DIR}/sdms.pc @ONLY)	
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/sdms.pc DESTINATION lib/pkgconfig/)