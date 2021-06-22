
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

	file(
		GLOB_RECURSE
		lib_value_function
		${INCLUDE_DIR}/utils/value_function/base_value_function.cpp
		${INCLUDE_DIR}/utils/value_function/value_function.cpp
		${INCLUDE_DIR}/utils/value_function/tabular_value_function.cpp
		${INCLUDE_DIR}/utils/value_function/backup/tabular_backup.cpp
	)
	add_library(value_function SHARED ${lib_value_function})
	target_link_libraries(value_function core)

	# LIB WORLD
	file(
		GLOB_RECURSE
		lib_world
		${INCLUDE_DIR}/world/mdp.cpp
		${INCLUDE_DIR}/world/mmdp.cpp
		${INCLUDE_DIR}/world/pomdp.cpp
		${INCLUDE_DIR}/world/mpomdp.cpp
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
		lib_hsvi
		${INCLUDE_DIR}/algorithms/hsvi.cpp
		${INCLUDE_DIR}/world/solvable_by_mdp.cpp
		${INCLUDE_DIR}/world/belief_mdp.cpp
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
		parser
		hsvi
		value_function
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
		${INCLUDE_DIR}/world/mpomdp.cpp
		${INCLUDE_DIR}/world/pomdp.cpp
		${INCLUDE_DIR}/parser/**
		${INCLUDE_DIR}/world/serialized_mpomdp.cpp
	)
	add_library(world SHARED ${lib_world})
	target_link_libraries(world core)

	# LIB HSVI
	file(
		GLOB_RECURSE
		lib_hsvi
		${INCLUDE_DIR}/algorithms/hsvi.cpp
		${INCLUDE_DIR}/world/solvable_by_mdp.cpp
		${INCLUDE_DIR}/world/belief_mdp.cpp
		${INCLUDE_DIR}/utils/value_function/value_function.cpp
		${INCLUDE_DIR}/utils/value_function/base_value_function.cpp
		${INCLUDE_DIR}/utils/value_function/tabular_value_function.cpp
		${INCLUDE_DIR}/utils/value_function/point_set_value_function.cpp
		${INCLUDE_DIR}/utils/value_function/hyperplan_value_function.cpp

		${INCLUDE_DIR}/utils/value_function/initializer/**
		${INCLUDE_DIR}/utils/value_function/backup/**
		${INCLUDE_DIR}/world/gym_interface.cpp


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
#######################################
### PARTIE A BARIS : NE PAS CHANGER ##
#######################################

# Build documentation
if(BARIS)
    message(STATUS "Build Baris quicktest.")
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
		${INCLUDE_DIR}/world/mpomdp.cpp
		${INCLUDE_DIR}/world/pomdp.cpp
		${INCLUDE_DIR}/parser/**


	)
	add_library(world SHARED ${lib_world})
	target_link_libraries(world core)

	# LIB HSVI
	file(
		GLOB_RECURSE
		lib_hsvi
		${INCLUDE_DIR}/algorithms/q_learning.cpp
		${INCLUDE_DIR}/world/gym_interface.cpp
		${INCLUDE_DIR}/utils/value_function/value_function.cpp
		${INCLUDE_DIR}/utils/value_function/base_value_function.cpp
		${INCLUDE_DIR}/utils/value_function/tabular_qvalue_function.cpp
		${INCLUDE_DIR}/utils/value_function/qvalue_function.cpp
		${INCLUDE_DIR}/utils/value_function/initializer/**
		${INCLUDE_DIR}/utils/value_function/backup/**

	)
	add_library(hsvi SHARED ${lib_hsvi})
	target_link_libraries(hsvi world fmt)
	include_directories(${SDMS_INCLUDE_DIRS})

	add_executable(quicktest_baris quicktest_baris.cpp)
	target_link_libraries(quicktest_baris
		dl
		${Boost_LIBRARIES}
		${TORCH_LIBRARIES}
		${CPLEX_LIBRARIES}
		hsvi
	)

endif()

# Build documentation
if(QLEARNING)
    message(STATUS "Build Baris quicktest.")
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
		${INCLUDE_DIR}/world/mpomdp.cpp
		${INCLUDE_DIR}/world/pomdp.cpp
		${INCLUDE_DIR}/parser/**


	)
	add_library(world SHARED ${lib_world})
	target_link_libraries(world core)

	# LIB HSVI
	file(
		GLOB_RECURSE
		lib_hsvi
		${INCLUDE_DIR}/algorithms/q_learning.cpp
		${INCLUDE_DIR}/world/gym_interface.cpp
		${INCLUDE_DIR}/utils/value_function/base_value_function.cpp
		${INCLUDE_DIR}/utils/value_function/qvalue_function.cpp
		${INCLUDE_DIR}/utils/value_function/tabular_qvalue_function.cpp
		${INCLUDE_DIR}/utils/value_function/initializer/initializer.cpp
		${INCLUDE_DIR}/utils/value_function/backup/tabular_qvalue_backup.cpp

	)
	add_library(hsvi SHARED ${lib_hsvi})
	target_link_libraries(hsvi world fmt)
	include_directories(${SDMS_INCLUDE_DIRS})

	add_executable(quicktest_qlearning quicktest_qlearning.cpp)
	target_link_libraries(quicktest_qlearning
		dl
		${Boost_LIBRARIES}
		${TORCH_LIBRARIES}
		${CPLEX_LIBRARIES}
		hsvi
	)

endif()