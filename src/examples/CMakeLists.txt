file(
	GLOB
	usage_examples
	*.cpp
)

foreach(f ${usage_examples})
  get_filename_component(exampleName ${f} NAME_WE) 
  add_executable (${exampleName} ${f}) 
  target_link_libraries(${exampleName} fmt ${TORCH_LIBRARIES} ${Boost_LIBRARIES} ${LIB_SDMS})
  install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/${exampleName}
    DESTINATION bin/
    RENAME "sdms-${exampleName}")
endforeach(f)


#### This section can be uncommented and rewrite in order to only partially compile sdms

file(
	GLOB_RECURSE
	lib_world
	# ${INCLUDE_DIR}/*
	${INCLUDE_DIR}/utils/struct/*
	# ${INCLUDE_DIR}/core/*
	${INCLUDE_DIR}/common.cpp
	${INCLUDE_DIR}/tools.cpp
)
add_library(lib_world SHARED ${lib_world})
include_directories(${SDMS_INCLUDE_DIRS})

add_executable(quicktest tmp/quicktest.cpp)
target_link_libraries(quicktest
  lib_world
  fmt
  ${Boost_LIBRARIES}
  ${TORCH_LIBRARIES}
)

install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/quicktest
    DESTINATION bin/
    )