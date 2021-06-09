
######################################
### PARTIE A DAVID : NE PAS CHANGER ##
######################################

# Build documentation
if(DAVID)
    message(STATUS "Build David quicktest.")
	#### This section can be uncommented and rewrite in order to compile only a part of sdms (quicktest.cpp file)

	# LIB CORE
	file(
		GLOB_RECURSE
		lib_core
		${INCLUDE_DIR}/core/*
		${INCLUDE_DIR}/utils/struct/iterator/*
		${INCLUDE_DIR}/common.cpp
		${INCLUDE_DIR}/tools.cpp
	)
	add_library(core SHARED ${lib_core})

	# LIB WORLD
	file(
		GLOB_RECURSE
		lib_world
		${INCLUDE_DIR}/world/mdp.cpp
		${INCLUDE_DIR}/world/mmdp.cpp
	)
	add_library(world SHARED ${lib_world})
	target_link_libraries(world core)

	# LIB HSVI
	file(
		GLOB_RECURSE
		lib_hsvi
		${INCLUDE_DIR}/algorithms/hsvi.cpp
		${INCLUDE_DIR}/world/solvable_by_mdp.cpp
		${INCLUDE_DIR}/tools.cpp
		${INCLUDE_DIR}/utils/value_function/value_function.cpp
		${INCLUDE_DIR}/utils/value_function/base_value_function.cpp
	)
	add_library(hsvi SHARED ${lib_hsvi})
	target_link_libraries(hsvi world fmt)
	include_directories(${SDMS_INCLUDE_DIRS})

	add_executable(quicktest_david quicktest_david.cpp)
	target_link_libraries(quicktest_david
		dl
		${Boost_LIBRARIES}
		${TORCH_LIBRARIES}
		${CPLEX_LIBRARIES}
		hsvi
	)
endif()


#######################################
### PARTIE A JEROME : NE PAS CHANGER ##
#######################################

# Build documentation
if(JEROME)
    message(STATUS "Build Jerome quicktest.")
	#### This section can be uncommented and rewrite in order to compile only a part of sdms (quicktest.cpp file)

	# LIB CORE
	file(
		GLOB_RECURSE
		lib_core
		${INCLUDE_DIR}/core/*
		${INCLUDE_DIR}/utils/struct/iterator/*
		${INCLUDE_DIR}/common.cpp
		${INCLUDE_DIR}/tools.cpp
	)
	add_library(core SHARED ${lib_core})

	# LIB WORLD
	file(
		GLOB_RECURSE
		lib_world
		${INCLUDE_DIR}/world/mdp.cpp
		${INCLUDE_DIR}/world/mmdp.cpp
		${INCLUDE_DIR}/world/serialized_mmdp.cpp
	)
	add_library(world SHARED ${lib_world})
	target_link_libraries(world core)

	# LIB HSVI
	file(
		GLOB_RECURSE
		lib_hsvi
		${INCLUDE_DIR}/algorithms/hsvi.cpp
		${INCLUDE_DIR}/world/solvable_by_mdp.cpp
		${INCLUDE_DIR}/tools.cpp
		${INCLUDE_DIR}/utils/value_function/value_function.cpp
		${INCLUDE_DIR}/utils/value_function/base_value_function.cpp
	)
	add_library(hsvi SHARED ${lib_hsvi})
	target_link_libraries(hsvi world fmt)
	include_directories(${SDMS_INCLUDE_DIRS})

	add_executable(quicktest_jerome quicktest_jerome.cpp)
	target_link_libraries(quicktest_jerome
		dl
		${Boost_LIBRARIES}
		${TORCH_LIBRARIES}
		${CPLEX_LIBRARIES}
		hsvi
	)
endif()