# Find packages
find_package(Boost COMPONENTS system filesystem program_options iostreams serialization REQUIRED)
find_package(Torch REQUIRED)
find_package(CPLEX)

# Check TORCH dependancy and include directories
if(TORCH_FOUND)
	include_directories(${TORCH_INCLUDE_DIRS})
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")
	add_definitions("-DWITH_TORCH")
endif()

# Check CPLEX dependancy and include directories
if(CPLEX_FOUND)
	include_directories(${CPLEX_INCLUDE_DIRS})
	add_definitions("-DWITH_CPLEX")
	add_definitions("-DIL_STD")
endif()

# Add boost definitions
add_definitions (-DBOOST)
add_definitions (-DDENSE)

# Add TOULBAR2 definitions
add_definitions (-DLONGDOUBLE_PROB -DLONGLONG_COST -DWCSPFORMATONLY)

# Add include directories
include_directories(${SDMS_INCLUDE_DIRS}
					${Boost_INCLUDE_DIRS}
					${TOULBAR_INCLUDE_DIR}
					)
# LIB CORE
file(
	GLOB_RECURSE
	lib_core_src
	${INCLUDE_DIR}/tools.cpp
	${INCLUDE_DIR}/common.cpp
	${INCLUDE_DIR}/exception.cpp
	${INCLUDE_DIR}/public/**
	${INCLUDE_DIR}/core/**
	${INCLUDE_DIR}/utils/struct/**
	${INCLUDE_DIR}/utils/linear_algebra/**
	${INCLUDE_DIR}/utils/toml/**
	${INCLUDE_DIR}/utils/config.cpp
)
# this is the "object library" target: compiles the sources only once
add_library(core_obj OBJECT ${lib_core_src})

# shared libraries need PIC
set_property(TARGET core_obj PROPERTY POSITION_INDEPENDENT_CODE 1)

# shared and static libraries built from the same object files
add_library(core SHARED $<TARGET_OBJECTS:core_obj>)
add_library(core_static STATIC $<TARGET_OBJECTS:core_obj>)
# LIB VALUE FUNCTION
file(
	GLOB_RECURSE
	lib_value_function_src
	${INCLUDE_DIR}/utils/value_function/**
	${INCLUDE_DIR}/utils/linear_programming/**
)
# this is the "object library" target: compiles the sources only once
add_library(value_function_obj OBJECT ${lib_value_function_src})

# shared libraries need PIC
set_property(TARGET value_function_obj PROPERTY POSITION_INDEPENDENT_CODE 1)

# shared and static libraries built from the same object files
add_library(value_function SHARED $<TARGET_OBJECTS:value_function_obj>)
target_link_libraries(value_function core ${CPLEX_LIBRARIES})
add_library(value_function_static STATIC $<TARGET_OBJECTS:value_function_obj>)
target_link_libraries(value_function_static core_static ${CPLEX_LIBRARIES})

# LIB WORLD
file(
	GLOB_RECURSE
	lib_world_src
	${INCLUDE_DIR}/world/**
	${INCLUDE_DIR}/parser/**
)
# this is the "object library" target: compiles the sources only once
add_library(world_obj OBJECT ${lib_world_src})

# shared libraries need PIC
set_property(TARGET world_obj PROPERTY POSITION_INDEPENDENT_CODE 1)

# shared and static libraries built from the same object files
add_library(world SHARED $<TARGET_OBJECTS:world_obj>)
target_link_libraries(world core)
add_library(world_static STATIC $<TARGET_OBJECTS:world_obj>)
target_link_libraries(world_static core_static)

# # LIB PARSER
# file(
# 	GLOB_RECURSE
# 	lib_parser_src
# 	${INCLUDE_DIR}/parser/**
# )
# # this is the "object library" target: compiles the sources only once
# add_library(parser_obj OBJECT ${lib_parser_src})

# # shared libraries need PIC
# set_property(TARGET parser_obj PROPERTY POSITION_INDEPENDENT_CODE 1)

# # shared and static libraries built from the same object files
# add_library(parser SHARED $<TARGET_OBJECTS:parser_obj>)
# target_link_libraries(parser world)
# add_library(parser_static STATIC $<TARGET_OBJECTS:parser_obj>)
# target_link_libraries(parser_static world_static)

# LIB PLANNING (=DynamicProgramming)
file(
	GLOB_RECURSE
	lib_planning_src
	${INCLUDE_DIR}/algorithms/planning/**
	${INCLUDE_DIR}/algorithms/backward_induction.cpp
	${INCLUDE_DIR}/algorithms/alpha_star.cpp
	${INCLUDE_DIR}/algorithms/bayesian_game_solver.cpp
)
# this is the "object library" target: compiles the sources only once
add_library(planning_obj OBJECT ${lib_planning_src})

# shared libraries need PIC
set_property(TARGET planning_obj PROPERTY POSITION_INDEPENDENT_CODE 1)

# shared and static libraries built from the same object files
add_library(planning SHARED $<TARGET_OBJECTS:planning_obj>)
target_link_libraries(planning core value_function world fmt)
add_library(planning_static STATIC $<TARGET_OBJECTS:planning_obj>)
target_link_libraries(planning_static core_static value_function_static world_static fmt)

# LIB REINFORCEMENT LEARNING
file(
	GLOB_RECURSE
	lib_rl_src
	${INCLUDE_DIR}/utils/rl/**
	${INCLUDE_DIR}/algorithms/q_learning.cpp
	${INCLUDE_DIR}/algorithms/sarsa.cpp
)
# this is the "object library" target: compiles the sources only once
add_library(rl_obj OBJECT ${lib_rl_src})

# shared libraries need PIC
set_property(TARGET rl_obj PROPERTY POSITION_INDEPENDENT_CODE 1)

# shared and static libraries built from the same object files
add_library(rl SHARED $<TARGET_OBJECTS:rl_obj>)
target_link_libraries(rl core value_function world fmt)
add_library(rl_static STATIC $<TARGET_OBJECTS:rl_obj>)
target_link_libraries(rl_static core_static value_function_static world_static fmt)
# LIB SDMS
file(
	GLOB_RECURSE
	lib_sdms_src
	${INCLUDE_DIR}/algorithms.cpp
)

target_link_libraries(value_function planning_static rl_static)
target_link_libraries(value_function_static planning_static rl_static)


# this is the "object library" target: compiles the sources only once
add_library(sdms_obj OBJECT ${lib_sdms_src})

# shared libraries need PIC
set_property(TARGET sdms_obj PROPERTY POSITION_INDEPENDENT_CODE 1)

# shared and static libraries built from the same object files
add_library(${LIB_SDMS} SHARED $<TARGET_OBJECTS:sdms_obj>)
target_link_libraries(${LIB_SDMS} value_function world planning rl)
add_library(${LIB_SDMS_STATIC} STATIC $<TARGET_OBJECTS:sdms_obj>)
target_link_libraries(${LIB_SDMS_STATIC} value_function_static world_static planning_static rl_static)
install(TARGETS ${LIB_SDMS} DESTINATION lib)
install(TARGETS ${LIB_SDMS_STATIC} DESTINATION lib)

# file(COPY ${SDMS_LIB_DIR}/libtb2.so DESTINATION /usr/lib)
# install(FILES ${SDMS_LIB_DIR}/libtb2.so DESTINATION /usr/lib)
# install(FILES ${SDMS_LIB_DIR}/libtb2.so DESTINATION lib)

########### BUILD PROGRAMS ###########

# Build main program
set(SDMS_PROGRAM_NAME "SDMStudio")

set(TB2_LIBRARIES tb2) #${SDMS_LIB_DIR}/libtb2.so)
add_executable(${SDMS_PROGRAM_NAME} main.cpp) 
target_link_libraries(${SDMS_PROGRAM_NAME} dl ${Boost_LIBRARIES} ${TORCH_LIBRARIES} ${TB2_LIBRARIES} ${LIB_SDMS_STATIC})

########### INSTALL #################
# Install programs
install(DIRECTORY ${INCLUDE_DIR} DESTINATION include)
install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/${SDMS_PROGRAM_NAME} DESTINATION bin)

# Install documentation
install(DIRECTORY ${SDMS_MAN_DIR} DESTINATION share)

# Install data
install(DIRECTORY ${SDMS_WORLD_DIR} DESTINATION share/sdms)

# Install config
install(DIRECTORY ${SDMS_CONFIG_DIR} DESTINATION share/sdms)

add_subdirectory(examples)
add_subdirectory(programs)
