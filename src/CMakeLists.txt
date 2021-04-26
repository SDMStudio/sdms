# Find packages
find_package(Boost COMPONENTS system filesystem program_options iostreams REQUIRED)
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
	message(${CPLEX_INCLUDE_DIRS})
	add_definitions("-DWITH_CPLEX")
	add_definitions("-DIL_STD")
endif()

# Add boost definitions
add_definitions (-DBOOST)
add_definitions (-DDENSE)

# Add include directories
include_directories(${SDMS_INCLUDE_DIRS}
					${Boost_INCLUDE_DIRS}
					)
	
# Build libsdm (contains all sdms sources)
file(
	GLOB_RECURSE
	libsdm_srcs
	${SOURCE_DIR}/*.cpp
)
add_library(${LIB_SDMS} SHARED ${libsdm_srcs})
install(TARGETS ${LIB_SDMS} DESTINATION lib)

########### BUILD PROGRAMS ###########

# Build main program
set(SDMS_PROGRAM_NAME "SDMStudio")

add_executable(${SDMS_PROGRAM_NAME} main.cpp) 
target_link_libraries(${SDMS_PROGRAM_NAME} fmt dl ${TORCH_LIBRARIES} ${Boost_LIBRARIES} ${CPLEX_LIBRARIES} ${LIB_SDMS})

########### INSTALL #################
install(DIRECTORY ${INCLUDE_DIR} DESTINATION include)
install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/${SDMS_PROGRAM_NAME} DESTINATION bin)

install(DIRECTORY ${SDMS_MAN_DIR} DESTINATION share)

add_subdirectory(examples)
add_subdirectory(algos)

## OLD
###############################################
########### BUILD SDMS LIBRARIES ############## 
###############################################

# # Build libsdm_core (contains all tools for construction and parsing)
# file(
# 	GLOB_RECURSE
# 	libsdm_core_srcs
# 	${SDM_SOURCE_DIR}/sdm/core/*.cpp
# 	${SDM_SOURCE_DIR}/sdm/parsing/*.cpp
# )
# add_library(sdm_world SHARED ${libsdm_world_srcs})
# install(TARGETS sdm_world DESTINATION lib)

# # Build libsdm_world (contains all tools for world construction and parsing)
# file(
# 	GLOB_RECURSE
# 	libsdm_world_srcs
# 	${SDM_SOURCE_DIR}/sdm/world/*.cpp
# 	${SDM_SOURCE_DIR}/sdm/parsing/*.cpp
# )
# add_library(sdm_world SHARED ${libsdm_world_srcs})
# install(TARGETS sdm_world DESTINATION lib)