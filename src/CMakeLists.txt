# Find packages
find_package(Boost COMPONENTS system filesystem program_options serialization iostreams REQUIRED)
find_package(Torch REQUIRED)
find_package(CPLEX)

# Check TORCH dependancy and include directories
if(TORCH_FOUND)
	include_directories(${TORCH_INCLUDE_DIRS})
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")
	add_definitions("-DWITH_TORCH")
endif(TORCH_FOUND)
	
	# Check CPLEX dependancy and include directories
if(CPLEX_FOUND)
	include_directories(${CPLEX_INCLUDE_DIRS})
	add_definitions("-DWITH_CPLEX")
	add_definitions("-DIL_STD")
endif(CPLEX_FOUND)

set(TB2_LIBRARIES sdms_tb2) #${SDMS_LIB_DIR}/libtb2.so)
	
# Add boost definitions
add_definitions (-DBOOST)
add_definitions (-DDENSE)
	
# Add TOULBAR2 definitions
add_definitions (-DLONGDOUBLE_PROB -DLONGLONG_COST -DWCSPFORMATONLY)

# Add include directories
include_directories(${SDMS_INCLUDE_DIRS}
					${Boost_INCLUDE_DIRS}
					${TOULBAR_INCLUDE_DIR}
					${SPIRIT_INCLUDE_DIR}
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
add_library(sdm_core SHARED $<TARGET_OBJECTS:core_obj>)
add_library(sdm_core_static STATIC $<TARGET_OBJECTS:core_obj>)


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
add_library(sdm_world SHARED $<TARGET_OBJECTS:core_obj> $<TARGET_OBJECTS:world_obj>)
target_link_libraries(sdm_world -L${SDMS_LIB_DIR} boost_iostreams ${Boost_LIBRARIES})
add_library(sdm_world_static STATIC $<TARGET_OBJECTS:core_obj> $<TARGET_OBJECTS:world_obj>)
target_link_libraries(sdm_world_static -L${SDMS_LIB_DIR} boost_iostreams ${Boost_LIBRARIES})

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
add_library(sdm_value_function SHARED $<TARGET_OBJECTS:core_obj> $<TARGET_OBJECTS:value_function_obj>)
target_link_libraries(sdm_value_function -L${SDMS_LIB_DIR} boost_iostreams ${TORCH_LIBRARIES} ${Boost_LIBRARIES} ${TB2_LIBRARIES} ${CPLEX_LIBRARIES})
add_library(sdm_value_function_static STATIC $<TARGET_OBJECTS:core_obj> $<TARGET_OBJECTS:value_function_obj>)
target_link_libraries(sdm_value_function_static -L${SDMS_LIB_DIR} boost_iostreams ${TORCH_LIBRARIES} ${Boost_LIBRARIES} ${TB2_LIBRARIES} ${CPLEX_LIBRARIES})

# LIB PLANNING (=DynamicProgramming)
file(
	GLOB_RECURSE
	lib_planning_src
	${INCLUDE_DIR}/algorithms/planning/**
)
# this is the "object library" target: compiles the sources only once
add_library(planning_obj OBJECT ${lib_planning_src})

# shared libraries need PIC
set_property(TARGET planning_obj PROPERTY POSITION_INDEPENDENT_CODE 1)

# shared and static libraries built from the same object files
add_library(sdm_planning SHARED $<TARGET_OBJECTS:core_obj> $<TARGET_OBJECTS:value_function_obj>  $<TARGET_OBJECTS:world_obj> $<TARGET_OBJECTS:planning_obj>)
target_link_libraries(sdm_planning -L${SDMS_LIB_DIR} boost_iostreams dl ${TORCH_LIBRARIES} ${Boost_LIBRARIES} ${TB2_LIBRARIES} ${CPLEX_LIBRARIES} fmt)
add_library(sdm_planning_static STATIC $<TARGET_OBJECTS:core_obj> $<TARGET_OBJECTS:value_function_obj> $<TARGET_OBJECTS:world_obj> $<TARGET_OBJECTS:planning_obj>)
target_link_libraries(sdm_planning_static -L${SDMS_LIB_DIR} boost_iostreams dl ${TORCH_LIBRARIES} ${Boost_LIBRARIES} ${TB2_LIBRARIES} ${CPLEX_LIBRARIES} fmt)

# LIB REINFORCEMENT LEARNING
file(
	GLOB_RECURSE
	lib_rl_src
	${INCLUDE_DIR}/utils/rl/**
	${INCLUDE_DIR}/algorithms/rl/**
)
# this is the "object library" target: compiles the sources only once
add_library(rl_obj OBJECT ${lib_rl_src})

# shared libraries need PIC
set_property(TARGET rl_obj PROPERTY POSITION_INDEPENDENT_CODE 1)

# shared and static libraries built from the same object files
add_library(sdm_rl SHARED $<TARGET_OBJECTS:core_obj> $<TARGET_OBJECTS:value_function_obj>  $<TARGET_OBJECTS:world_obj>  $<TARGET_OBJECTS:rl_obj>)
target_link_libraries(sdm_rl -L${SDMS_LIB_DIR} boost_iostreams dl ${TORCH_LIBRARIES} ${Boost_LIBRARIES} ${TB2_LIBRARIES} ${CPLEX_LIBRARIES} fmt)
add_library(sdm_rl_static STATIC $<TARGET_OBJECTS:core_obj> $<TARGET_OBJECTS:value_function_obj>  $<TARGET_OBJECTS:world_obj>  $<TARGET_OBJECTS:rl_obj>)
target_link_libraries(sdm_rl_static -L${SDMS_LIB_DIR} boost_iostreams dl ${TORCH_LIBRARIES} ${Boost_LIBRARIES} ${TB2_LIBRARIES} ${CPLEX_LIBRARIES} fmt)
# LIB SDMS
file(
	GLOB_RECURSE
	lib_sdm_src
	${INCLUDE_DIR}/algorithms.cpp
)

# target_link_libraries(sdm_value_function sdm_planning_static sdm_rl_static)
# target_link_libraries(sdm_value_function_static sdm_planning_static sdm_rl_static)


# this is the "object library" target: compiles the sources only once
add_library(sdm_obj OBJECT ${lib_sdm_src})

# shared libraries need PIC
set_property(TARGET sdm_obj PROPERTY POSITION_INDEPENDENT_CODE 1)

# shared and static libraries built from the same object files
add_library(${LIB_SDMS} SHARED $<TARGET_OBJECTS:sdm_obj> $<TARGET_OBJECTS:core_obj> $<TARGET_OBJECTS:world_obj> $<TARGET_OBJECTS:value_function_obj> $<TARGET_OBJECTS:planning_obj> $<TARGET_OBJECTS:rl_obj>)
target_link_libraries(${LIB_SDMS} -L${SDMS_LIB_DIR} boost_iostreams dl ${TORCH_LIBRARIES} ${Boost_LIBRARIES} ${TB2_LIBRARIES} ${CPLEX_LIBRARIES} fmt)

add_library(${LIB_SDMS_STATIC} STATIC $<TARGET_OBJECTS:sdm_obj> $<TARGET_OBJECTS:core_obj> $<TARGET_OBJECTS:world_obj> $<TARGET_OBJECTS:value_function_obj> $<TARGET_OBJECTS:planning_obj> $<TARGET_OBJECTS:rl_obj>)
target_link_libraries(${LIB_SDMS_STATIC} -L${SDMS_LIB_DIR} boost_iostreams dl ${TORCH_LIBRARIES} ${Boost_LIBRARIES} ${TB2_LIBRARIES} ${CPLEX_LIBRARIES} fmt)

install(TARGETS sdm_core DESTINATION lib)
install(TARGETS sdm_core_static DESTINATION lib)
install(TARGETS sdm_world DESTINATION lib)
install(TARGETS sdm_world_static DESTINATION lib)
install(TARGETS sdm_value_function DESTINATION lib)
install(TARGETS sdm_value_function_static DESTINATION lib)
install(TARGETS sdm_planning DESTINATION lib)
install(TARGETS sdm_planning_static DESTINATION lib)
install(TARGETS sdm_rl DESTINATION lib)
install(TARGETS sdm_rl_static DESTINATION lib)
install(TARGETS ${LIB_SDMS} DESTINATION lib)
install(TARGETS ${LIB_SDMS_STATIC} DESTINATION lib)

# file(COPY ${SDMS_LIB_DIR}/libtb2.so DESTINATION /usr/lib)
# install(FILES ${SDMS_LIB_DIR}/libtb2.so DESTINATION /usr/lib)
# install(FILES ${SDMS_LIB_DIR}/libtb2.so DESTINATION lib)ct 

########### BUILD PROGRAMS ###########

# Build main program
set(SDMS_PROGRAM_NAME ${CMAKE_PROJECT_NAME})

add_executable(${SDMS_PROGRAM_NAME} main.cpp) 
target_link_libraries(${SDMS_PROGRAM_NAME} ${LIB_SDMS})

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
