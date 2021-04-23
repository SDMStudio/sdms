#  Copyright 2012 HHMI.  All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:

#     * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above
# copyright notice, this list of conditions and the following disclaimer
# in the documentation and/or other materials provided with the
# distribution.
#     * Neither the name of HHMI nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Author: katzw@janelia.hhmi.org (Bill Katz)
# Written as part of the FlyEM Project at Janelia Farm Research Center.

# This module finds cplex.
#
# User can give CPLEX_ROOT_DIR as a hint stored in the cmake cache.
#
# It sets the following variables:
#  CPLEX_FOUND              - Set to false, or undefined, if cplex isn't found.
#  CPLEX_INCLUDE_DIRS       - include directory
#  CPLEX_LIBRARIES          - library files

if(WIN32)
  execute_process(COMMAND cmd /C set CPLEX_STUDIO_DIR OUTPUT_VARIABLE CPLEX_STUDIO_DIR_VAR ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
  
  if(NOT CPLEX_STUDIO_DIR_VAR)
    MESSAGE(FATAL_ERROR "Unable to find CPLEX: environment variable CPLEX_STUDIO_DIR<VERSION> not set.")
  endif()
  
  STRING(REGEX REPLACE "^CPLEX_STUDIO_DIR" "" CPLEX_STUDIO_DIR_VAR ${CPLEX_STUDIO_DIR_VAR})
  STRING(REGEX MATCH "^[0-9]+" CPLEX_WIN_VERSION ${CPLEX_STUDIO_DIR_VAR})
  STRING(REGEX REPLACE "^[0-9]+=" "" CPLEX_STUDIO_DIR_VAR ${CPLEX_STUDIO_DIR_VAR})
  file(TO_CMAKE_PATH "${CPLEX_STUDIO_DIR_VAR}" CPLEX_ROOT_DIR_GUESS) 
  
  set(CPLEX_WIN_VERSION ${CPLEX_WIN_VERSION} CACHE STRING "CPLEX version to be used.")
  set(CPLEX_ROOT_DIR "${CPLEX_ROOT_DIR_GUESS}" CACHE PATH "CPLEX root directory.")
  
  MESSAGE(STATUS "Found CLPEX version ${CPLEX_WIN_VERSION} at '${CPLEX_ROOT_DIR}'")
  
  STRING(REGEX REPLACE "/VC/bin/.*" "" VISUAL_STUDIO_PATH ${CMAKE_C_COMPILER})
  STRING(REGEX MATCH "Studio [0-9]+" CPLEX_WIN_VS_VERSION ${VISUAL_STUDIO_PATH})
  STRING(REGEX REPLACE "Studio " "" CPLEX_WIN_VS_VERSION ${CPLEX_WIN_VS_VERSION})
  
  if(${CPLEX_WIN_VS_VERSION} STREQUAL "9")
    set(CPLEX_WIN_VS_VERSION 2008)
  elseif(${CPLEX_WIN_VS_VERSION} STREQUAL "10")
    set(CPLEX_WIN_VS_VERSION 2010)
  elseif(${CPLEX_WIN_VS_VERSION} STREQUAL "11")
    set(CPLEX_WIN_VS_VERSION 2012)
  else()
    MESSAGE(FATAL_ERROR "CPLEX: unknown Visual Studio version at '${VISUAL_STUDIO_PATH}'.")
  endif()
  
  set(CPLEX_WIN_VS_VERSION ${CPLEX_WIN_VS_VERSION} CACHE STRING "Visual Studio Version")
  
  if("${CMAKE_C_COMPILER}" MATCHES "amd64")
    set(CPLEX_WIN_BITNESS x64)
  else()
    set(CPLEX_WIN_BITNESS x86)
  endif()
  
  set(CPLEX_WIN_BITNESS ${CPLEX_WIN_BITNESS} CACHE STRING "On Windows: x86 or x64 (32bit resp. 64bit)")

  MESSAGE(STATUS "CPLEX: using Visual Studio ${CPLEX_WIN_VS_VERSION} ${CPLEX_WIN_BITNESS} at '${VISUAL_STUDIO_PATH}'")

  if(NOT CPLEX_WIN_LINKAGE)
    set(CPLEX_WIN_LINKAGE mda CACHE STRING "CPLEX linkage variant on Windows. One of these: mda (dll, release), mdd (dll, debug), mta (static, release), mtd (static, debug)")
  endif(NOT CPLEX_WIN_LINKAGE)

  # now, generate platform string
  set(CPLEX_WIN_PLATFORM "${CPLEX_WIN_BITNESS}_windows_vs${CPLEX_WIN_VS_VERSION}/stat_${CPLEX_WIN_LINKAGE}")

else()

  set(CPLEX_WIN_PLATFORM "")
  string(TOLOWER ${CMAKE_SYSTEM_NAME} CPLEX_SYSTEM_NAME_LOWER)
  set(CPLEX_PLATFORM "${CMAKE_SYSTEM_PROCESSOR}_${CPLEX_SYSTEM_NAME_LOWER}")
    
endif()

message(STATUS "cplex root dir: ${CPLEX_ROOT_DIR}")

FIND_PATH(CPLEX_INCLUDE_DIR
  ilcplex/cplex.h
  HINTS ${CPLEX_ROOT_DIR}/cplex/include
        ${CPLEX_ROOT_DIR}/include
  PATHS ENV C_INCLUDE_PATH
        ENV C_PLUS_INCLUDE_PATH
        ENV INCLUDE_PATH
  )

FIND_PATH(CPLEX_CONCERT_INCLUDE_DIR
  ilconcert/iloenv.h 
  HINTS ${CPLEX_ROOT_DIR}/concert/include
        ${CPLEX_ROOT_DIR}/include
  PATHS ENV C_INCLUDE_PATH
        ENV C_PLUS_INCLUDE_PATH
        ENV INCLUDE_PATH
  )

FIND_LIBRARY(CPLEX_LIBRARY
  NAMES cplex${CPLEX_WIN_VERSION} cplex
  HINTS ${CPLEX_ROOT_DIR}/cplex/lib/${CPLEX_WIN_PLATFORM} #windows
        ${CPLEX_ROOT_DIR}/cplex/lib/${CPLEX_PLATFORM}/static_pic #generic
        ${CPLEX_ROOT_DIR}/cplex/lib/x86-64_debian4.0_4.1/static_pic #unix
        ${CPLEX_ROOT_DIR}/cplex/lib/x86-64_sles10_4.1/static_pic #unix 
        ${CPLEX_ROOT_DIR}/cplex/lib/x86-64_linux/static_pic #linux (ubuntu)
        ${CPLEX_ROOT_DIR}/cplex/lib/x86-64_osx/static_pic #osx 
        ${CPLEX_ROOT_DIR}/cplex/lib/x86-64_darwin/static_pic #osx 
  PATHS ENV LIBRARY_PATH #unix
        ENV LD_LIBRARY_PATH #unix
  )

if(CPLEX_LIBRARY)
    message(STATUS "CPLEX Library - found")
else()
    message(STATUS "CPLEX Library - not found")
endif()

FIND_LIBRARY(CPLEX_ILOCPLEX_LIBRARY
  ilocplex
  HINTS ${CPLEX_ROOT_DIR}/cplex/lib/${CPLEX_WIN_PLATFORM} #windows 
        ${CPLEX_ROOT_DIR}/cplex/lib/${CPLEX_PLATFORM}/static_pic #generic
        ${CPLEX_ROOT_DIR}/cplex/lib/x86-64_debian4.0_4.1/static_pic #unix 
        ${CPLEX_ROOT_DIR}/cplex/lib/x86-64_sles10_4.1/static_pic #unix 
        ${CPLEX_ROOT_DIR}/cplex/lib/x86-64_osx/static_pic #osx 
        ${CPLEX_ROOT_DIR}/cplex/lib/x86-64_darwin/static_pic #osx 
        ${CPLEX_ROOT_DIR}/cplex/lib/x86-64_linux/static_pic #linux (ubuntu)
  PATHS ENV LIBRARY_PATH
        ENV LD_LIBRARY_PATH
  )

if(CPLEX_ILOCPLEX_LIBRARY)
    message(STATUS "ILOCPLEX Library - found")
else()
    message(STATUS "ILOCPLEX Library - not found")
endif()

FIND_LIBRARY(CPLEX_CONCERT_LIBRARY
  concert
  HINTS ${CPLEX_ROOT_DIR}/concert/lib/${CPLEX_WIN_PLATFORM} #windows 
        ${CPLEX_ROOT_DIR}/concert/lib/${CPLEX_PLATFORM}/static_pic #generic
        ${CPLEX_ROOT_DIR}/concert/lib/x86-64_debian4.0_4.1/static_pic #unix 
        ${CPLEX_ROOT_DIR}/concert/lib/x86-64_sles10_4.1/static_pic #unix 
        ${CPLEX_ROOT_DIR}/concert/lib/x86-64_osx/static_pic #osx 
        ${CPLEX_ROOT_DIR}/concert/lib/x86-64_darwin/static_pic #osx 
        ${CPLEX_ROOT_DIR}/concert/lib/x86-64_linux/static_pic #linux (ubuntu)
  PATHS ENV LIBRARY_PATH
        ENV LD_LIBRARY_PATH
  )

if(CPLEX_CONCERT_LIBRARY)
    message(STATUS "CONCERT Library - found")
else()
    message(STATUS "CONCERT Library - not found")
endif()

if(WIN32)
	FIND_PATH(CPLEX_BIN_DIR
	  cplex${CPLEX_WIN_VERSION}.dll
          HINTS ${CPLEX_ROOT_DIR}/cplex/bin/${CPLEX_WIN_PLATFORM} #windows
	  )
else()
	FIND_PATH(CPLEX_BIN_DIR
	  cplex 
          HINTS ${CPLEX_ROOT_DIR}/cplex/bin/x86-64_sles10_4.1 #unix
	        ${CPLEX_ROOT_DIR}/cplex/bin/${CPLEX_PLATFORM}/static_pic #generic

                ${CPLEX_ROOT_DIR}/cplex/bin/x86-64_debian4.0_4.1 #unix 
                ${CPLEX_ROOT_DIR}/cplex/bin/x86-64_osx #osx
		${CPLEX_ROOT_DIR}/cplex/bin/x86-64_darwin #osx 
		${CPLEX_ROOT_DIR}/cplex/bin/x86-64_linux #linux (ubuntu)
	  ENV LIBRARY_PATH
          ENV LD_LIBRARY_PATH
	  )
endif()
message(STATUS "CPLEX Bin Dir: ${CPLEX_BIN_DIR}")

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(CPLEX DEFAULT_MSG 
 CPLEX_LIBRARY CPLEX_INCLUDE_DIR CPLEX_ILOCPLEX_LIBRARY CPLEX_CONCERT_LIBRARY CPLEX_CONCERT_INCLUDE_DIR)

IF(CPLEX_FOUND)
  SET(CPLEX_INCLUDE_DIRS ${CPLEX_INCLUDE_DIR} ${CPLEX_CONCERT_INCLUDE_DIR})
  SET(CPLEX_LIBRARIES ${CPLEX_CONCERT_LIBRARY} ${CPLEX_ILOCPLEX_LIBRARY} ${CPLEX_LIBRARY} )
  IF(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    SET(CPLEX_LIBRARIES "${CPLEX_LIBRARIES};m;pthread;dl")
  ENDIF(CMAKE_SYSTEM_NAME STREQUAL "Linux")
ENDIF(CPLEX_FOUND)

MARK_AS_ADVANCED(CPLEX_LIBRARY CPLEX_INCLUDE_DIR CPLEX_ILOCPLEX_LIBRARY CPLEX_CONCERT_INCLUDE_DIR CPLEX_CONCERT_LIBRARY CPLEX_BIN_DIR)
