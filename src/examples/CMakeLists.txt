# Store *.cpp files in a liste
file(
	GLOB
	usage_examples
	*.cpp
)

# For each .cpp file (sub-programs), make the associated main program
foreach(f ${usage_examples})
  get_filename_component(exampleName ${f} NAME_WE) 
  add_executable (${exampleName} ${f}) 
  target_link_libraries(${exampleName} fmt dl ${TORCH_LIBRARIES} ${Boost_LIBRARIES} ${CPLEX_LIBRARIES} ${LIB_SDMS})
  install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/${exampleName}
    DESTINATION bin/
    RENAME "sdms-${exampleName}")
endforeach(f)


#### This section can be uncommented and rewrite in order to compile only a part of sdms (quicktest.cpp file)
file(
	GLOB_RECURSE
	# lib_world
	# ${INCLUDE_DIR}/*
	# ${INCLUDE_DIR}/utils/struct/*
	${INCLUDE_DIR}/core/action/*
	# ${INCLUDE_DIR}/common.cpp
	# ${INCLUDE_DIR}/tools.cpp
)
# add_library(lib_world SHARED ${lib_world})
include_directories(${SDMS_INCLUDE_DIRS})

add_executable(quicktest tmp/quicktest.cpp)
target_link_libraries(quicktest
fmt
dl
${Boost_LIBRARIES}
${TORCH_LIBRARIES}
${CPLEX_LIBRARIES}
# lib_world
)



# Build documentation
if(DAVID)
    message(STATUS "Build David quicktest.")
	#### This section can be uncommented and rewrite in order to compile only a part of sdms (quicktest.cpp file)

	# LIB CORE
	file(
		GLOB_RECURSE
		lib_core
		${INCLUDE_DIR}/core/*
		${INCLUDE_DIR}/common.cpp
		${INCLUDE_DIR}/tools.cpp
	)
	add_library(core SHARED ${lib_core})

	# LIB WORLD
	file(
		GLOB_RECURSE
		lib_world
		${INCLUDE_DIR}/world/base/base_mdp.cpp
	)
	add_library(world SHARED ${lib_world})
	target_link_libraries(world core)

	# LIB HSVI
	file(
		GLOB_RECURSE
		lib_hsvi
		# ${INCLUDE_DIR}/algorithms/hsvi.cpp
		# ${INCLUDE_DIR}/world/mdp.cpp
		${INCLUDE_DIR}/utils/value_function/value_function.cpp
		${INCLUDE_DIR}/utils/value_function/base_value_function.cpp
	)
	add_library(hsvi SHARED ${lib_hsvi})
	target_link_libraries(hsvi world fmt)
	include_directories(${SDMS_INCLUDE_DIRS})

	add_executable(quicktest_david tmp/quicktest_david.cpp)
	target_link_libraries(quicktest_david
		dl
		${Boost_LIBRARIES}
		${TORCH_LIBRARIES}
		${CPLEX_LIBRARIES}
		hsvi
	)
endif()

install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/quicktest
    DESTINATION bin/
)