cmake_minimum_required(VERSION 2.8)

# Set compiler and options
set(CMAKE_C_COMPILER "clang")
set(CMAKE_CXX_COMPILER "clang++")

set(CMAKE_CXX_FLAGS_DEBUG_INIT "-O3 -W -Wall -pedantic")
set(CMAKE_CXX_FLAGS_RELEASE_INIT "-Wall")
set(CMAKE_BUILD_TYPE Debug)

# Version of the std library
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

project(SDMStudio)

set(LIB_NAME "sdm")

add_subdirectory(src)
add_subdirectory(doc)


add_subdirectory(tests)
enable_testing ()
add_test (NAME TestWorld 
          COMMAND test_world
      )
add_test (NAME TestSpace 
        COMMAND test_space
)