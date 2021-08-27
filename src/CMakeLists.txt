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

if(GLOBAL)
	# LIB CORE
	file(
		GLOB_RECURSE
		lib_core
		${INCLUDE_DIR}/tools.cpp
		${INCLUDE_DIR}/common.cpp
		${INCLUDE_DIR}/public/**
		${INCLUDE_DIR}/utils/struct/**
		${INCLUDE_DIR}/utils/linear_algebra/**
		${INCLUDE_DIR}/core/**
	)
	add_library(core SHARED ${lib_core})

	file(
		GLOB_RECURSE
		lib_value_function
		${INCLUDE_DIR}/utils/value_function/**
		${INCLUDE_DIR}/utils/linear_programming/**
	)
	add_library(value_function SHARED ${lib_value_function})
	target_link_libraries(value_function core ${CPLEX_LIBRARIES})

	# LIB WORLD
	file(
		GLOB_RECURSE
		lib_world
		${INCLUDE_DIR}/world/**
	)
	add_library(world SHARED ${lib_world})
	target_link_libraries(world core)

	# LIB PARSER
	file(
		GLOB_RECURSE
		lib_parser
		${INCLUDE_DIR}/parser/**
	)
	add_library(parser SHARED ${lib_parser})
	target_link_libraries(parser world)

	# LIB HSVI
	file(
		GLOB_RECURSE
		planning
		${INCLUDE_DIR}/algorithms/hsvi.cpp
		${INCLUDE_DIR}/algorithms/value_iteration.cpp
		${INCLUDE_DIR}/algorithms/backward_induction.cpp
		${INCLUDE_DIR}/algorithms/alpha_star.cpp
	)
	add_library(planning SHARED ${planning})
	target_link_libraries(planning world core fmt)


	# LIB HSVI
	file(
		GLOB_RECURSE
		lib_rl
		${INCLUDE_DIR}/utils/rl/**
		${INCLUDE_DIR}/algorithms/q_learning.cpp
	)
	add_library(rl SHARED ${lib_rl})
	target_link_libraries(rl world core fmt)

	# LIB SDMS
	file(
		GLOB_RECURSE
		lib_sdm
		${INCLUDE_DIR}/algorithms.cpp
	)
	add_library(${LIB_SDMS} SHARED ${lib_sdm})
	target_link_libraries(${LIB_SDMS} value_function parser planning rl)

	install(TARGETS ${LIB_SDMS} DESTINATION lib)
endif()

########### BUILD PROGRAMS ###########

# Build main program
set(SDMS_PROGRAM_NAME "SDMStudio")

add_executable(${SDMS_PROGRAM_NAME} main.cpp) 
target_link_libraries(${SDMS_PROGRAM_NAME} -L${SDMS_LIB_DIR} dl ${Boost_LIBRARIES} ${TORCH_LIBRARIES} ${SDMS_LIB_DIR}/libtb2.so ${LIB_SDMS})

########### INSTALL #################
# Install programs
install(DIRECTORY ${INCLUDE_DIR} DESTINATION include)
install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/${SDMS_PROGRAM_NAME} DESTINATION bin)

# Install documentation
install(DIRECTORY ${SDMS_MAN_DIR} DESTINATION share)
# Install data
install(DIRECTORY ${SDMS_WORLD_DIR} DESTINATION share/sdms)

add_subdirectory(examples)
add_subdirectory(programs)
