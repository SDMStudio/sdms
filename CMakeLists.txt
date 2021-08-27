cmake_minimum_required(VERSION 3.8)

# Set compiler
set(CMAKE_CXX_COMPILER "clang++")
set(CMAKE_C_COMPILER "clang")

# Set compilation options
set(CMAKE_CXX_FLAGS_DEBUG_INIT "-O3 -W -Wall -pedantic")
set(CMAKE_CXX_FLAGS_RELEASE_INIT "-Wall")

set(CMAKE_BUILD_TYPE Debug)

# Set path to find additional dependencies
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

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
option(BUILD_SDMS "Build SDM'Studio" ON)
option(BUILD_DOCS "Build SDMS documentation" OFF)
option(BUILD_TESTS "Build tests for SDMS" OFF)
option(GLOBAL "Build SDM'Studio globally" ON)

# Path to third party (can be defined if different from default) 
# --> Pytorch
if(NOT DEFINED CMAKE_PREFIX_PATH)
    set(CMAKE_PREFIX_PATH "/opt/libtorch")
endif()

# --> ILOG CPLEX
if(NOT DEFINED CPLEX_ROOT_DIR)
    set(CPLEX_ROOT_DIR "/opt/ibm/ILOG/CPLEX_Studio201/")
endif()


# Set useful variables
set(SDMS_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/src)
set(SDMS_SOURCE_DIRS ${CMAKE_SOURCE_DIR}/src)
set(SDMS_LIB_DIR ${CMAKE_SOURCE_DIR}/lib)
set(SDMS_DOC_DIR ${CMAKE_SOURCE_DIR}/docs)
set(SDMS_DATA_DIR ${CMAKE_SOURCE_DIR}/data)
set(SDMS_MAN_DIR ${SDMS_DOC_DIR}/man)
set(SDMS_WORLD_DIR ${SDMS_DATA_DIR}/world)

set(TOULBAR_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/include/tb2)

set(INCLUDE_DIR ${SDMS_INCLUDE_DIRS}/sdm)
set(SOURCE_DIR ${SDMS_SOURCE_DIRS}/sdm)
set(EXAMPLES_DIR ${SDMS_SOURCE_DIRS}/examples)
set(ALGOS_DIR ${SDMS_SOURCE_DIRS}/algos)


# Set names of libraries
set(LIB_SDMS ${CMAKE_PROJECT_NAME})

if (BUILD_SDMS)
    message(STATUS "Building SDMS sources.")
    add_subdirectory(src)
endif()

# Build documentation
if(BUILD_DOCS)
    message(STATUS "Building SDMS documentation.")
    add_subdirectory(docs)
endif()

# Build tests
if(BUILD_TESTS)
    message(STATUS "Building SDMS tests.")
    add_subdirectory(tests)
    enable_testing ()
    add_test(   NAME TestWorld 
                COMMAND test_world
            )
    add_test(   NAME TestSpace 
                COMMAND test_space
            )
endif()

# Configure sdms.pc file and install it
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/sdms.pc.in ${CMAKE_CURRENT_BINARY_DIR}/sdms.pc @ONLY)	

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/sdms.pc    
        DESTINATION lib/pkgconfig/)