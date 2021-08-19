
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
		lib_hsvi
		${INCLUDE_DIR}/algorithms/hsvi.cpp
		${INCLUDE_DIR}/algorithms/q_learning.cpp
		${INCLUDE_DIR}/world/solvable_by_mdp.cpp
		${INCLUDE_DIR}/world/belief_mdp.cpp
		${INCLUDE_DIR}/world/occupancy_mdp.cpp
	)
	add_library(hsvi SHARED ${lib_hsvi})
	target_link_libraries(hsvi world core fmt)

	# LIB SDMS
	file(
		GLOB_RECURSE
		lib_sdm
		${INCLUDE_DIR}/algorithms.cpp
	)
	add_library(sdm SHARED ${lib_sdm})
	target_link_libraries(sdm value_function parser hsvi)

	include_directories(${SDMS_INCLUDE_DIRS})

	add_executable(quicktest_david quicktest_david.cpp)
	target_link_libraries(quicktest_david
		dl
		${Boost_LIBRARIES}
		${TORCH_LIBRARIES}
		sdm
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
		${INCLUDE_DIR}/common.cpp
		${INCLUDE_DIR}/tools.cpp
		${INCLUDE_DIR}/public/**
		${INCLUDE_DIR}/core/**

		${INCLUDE_DIR}/utils/struct/**
		${INCLUDE_DIR}/utils/linear_algebra/**
	)
	add_library(core SHARED ${lib_core})

	# LIB WORLD
	file(
		GLOB_RECURSE
		lib_world
		${INCLUDE_DIR}/parser/**
		${INCLUDE_DIR}/world/**

	)
	add_library(world SHARED ${lib_world})
	target_link_libraries(world core)

	# LIB HSVI
	file(
		GLOB_RECURSE
		lib_hsvi
		${INCLUDE_DIR}/algorithms.cpp
		${INCLUDE_DIR}/algorithms/hsvi.cpp
		${INCLUDE_DIR}/algorithms/q_learning.cpp

		${INCLUDE_DIR}/utils/value_function/**
		${INCLUDE_DIR}/utils/linear_programming/**

	)
	add_library(hsvi SHARED ${lib_hsvi})
	target_link_libraries(hsvi world fmt)
	include_directories(${SDMS_INCLUDE_DIRS})

	add_executable(quicktest_jerome quicktest_jerome.cpp)
	target_link_libraries(quicktest_jerome
		-L${SDMS_LIB_DIR} 
		dl
		${Boost_LIBRARIES}
		${TORCH_LIBRARIES}
		${CPLEX_LIBRARIES}
		${SDMS_LIB_DIR}/libtb2.so 
		#${LIB_SDMS}
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
		${INCLUDE_DIR}/algorithms/hsvi.cpp
		${INCLUDE_DIR}/world/belief_mdp.cpp
		${INCLUDE_DIR}/world/occupancy_mdp.cpp
		${INCLUDE_DIR}/world/private_hierarchical_occupancy_mdp.cpp
		${INCLUDE_DIR}/world/private_hierarchical_occupancy_mdp_with_history.cpp
		${INCLUDE_DIR}/world/solvable_by_mdp.cpp
		${INCLUDE_DIR}/world/networked_distributed_pomdp.cpp
		${INCLUDE_DIR}/utils/value_function/base_value_function.cpp
		${INCLUDE_DIR}/utils/value_function/qvalue_function.cpp
		${INCLUDE_DIR}/utils/value_function/value_function.cpp
		${INCLUDE_DIR}/utils/value_function/tabular_qvalue_function.cpp
		${INCLUDE_DIR}/utils/value_function/hierarchical_qvalue_function.cpp
		${INCLUDE_DIR}/utils/value_function/initializer/initializer.tpp
		${INCLUDE_DIR}/utils/value_function/backup/tabular_qvalue_backup.cpp
		${INCLUDE_DIR}/utils/value_function/backup/extensive_qvalue_backup.cpp
		${INCLUDE_DIR}/utils/rl/experience_memory.cpp
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
    message(STATUS "Build Qlearning quicktest.")
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
		${INCLUDE_DIR}/algorithms/q_learning.tpp
		${INCLUDE_DIR}/algorithms/hsvi.cpp
		${INCLUDE_DIR}/world/belief_mdp.cpp
		${INCLUDE_DIR}/world/occupancy_mdp.cpp
		${INCLUDE_DIR}/world/hierarchical_occupancy_mdp.cpp
		${INCLUDE_DIR}/world/private_hierarchical_occupancy_mdp.cpp
		${INCLUDE_DIR}/world/private_hierarchical_occupancy_mdp_with_history.cpp
		${INCLUDE_DIR}/world/hierarchical_occupancy_mdp_with_history.cpp
		${INCLUDE_DIR}/world/solvable_by_mdp.cpp
		${INCLUDE_DIR}/world/networked_distributed_pomdp.cpp
		${INCLUDE_DIR}/utils/value_function/base_value_function.cpp
		${INCLUDE_DIR}/utils/value_function/qvalue_function.tpp
		${INCLUDE_DIR}/utils/value_function/value_function.cpp
		${INCLUDE_DIR}/utils/value_function/tabular_qvalue_function.tpp
		${INCLUDE_DIR}/utils/value_function/extensive_qvalue_function.cpp
		${INCLUDE_DIR}/utils/value_function/light_extensive_qvalue_function.cpp
		${INCLUDE_DIR}/utils/value_function/initializer/initializer.tpp
		${INCLUDE_DIR}/utils/value_function/backup/tabular_qvalue_backup.cpp
		${INCLUDE_DIR}/utils/value_function/backup/simple_hierarchical_qvalue_backup.cpp
		${INCLUDE_DIR}/utils/value_function/backup/hierarchical_qvalue_backup.cpp
		${INCLUDE_DIR}/utils/value_function/backup/extensive_qvalue_backup.cpp
		${INCLUDE_DIR}/utils/value_function/backup/light_extensive_qvalue_backup.cpp
		${INCLUDE_DIR}/utils/rl/experience_memory.cpp
		${INCLUDE_DIR}/world/hierarchical_mpomdp.cpp
		${INCLUDE_DIR}/world/transformed_mpomdp.cpp
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

	add_executable(ex-hierarchical-mpomdp ex-hierarchical-mpomdp.cpp)
	target_link_libraries(ex-hierarchical-mpomdp
		dl
		${Boost_LIBRARIES}
		${TORCH_LIBRARIES}
		${CPLEX_LIBRARIES}
		hsvi
	)

	add_executable(test-perf-qlearning test-perf-qlearning.cpp)
	target_link_libraries(test-perf-qlearning
		dl
		${Boost_LIBRARIES}
		${TORCH_LIBRARIES}
		${CPLEX_LIBRARIES}
		hsvi
	)

endif()