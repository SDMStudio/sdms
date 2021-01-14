file(
	GLOB
	usage_examples
	*.cpp
)

foreach(f ${usage_examples})
  get_filename_component(exampleName ${f} NAME_WE) 
  add_executable (${exampleName} ${f}) 
  target_link_libraries(${exampleName} ${LIB_SDMS} ${Boost_LIBRARIES} ${TORCH_LIBRARIES})
  install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/${exampleName}
    DESTINATION bin/examples/
    RENAME "sdms-${exampleName}")
endforeach(f)


# file(
# 	GLOB_RECURSE
# 	lib_struct
# 	${SOURCE_DIR}/utils/struct/*.cpp
# )
# add_library(lib_struct SHARED ${lib_struct})
include_directories(${SDMS_INCLUDE_DIRS})

file(
	GLOB_RECURSE
	SRC_FILES
	${SOURCE_DIR}/*.cpp
)
message(${SRC_FILES})
add_library(algebraLib SHARED ${SRC_FILES})

add_executable(quicktest tmp/quicktest.cpp)
target_link_libraries(quicktest
  algebraLib
  ${Boost_LIBRARIES}
  ${TORCH_LIBRARIES}
)

install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/quicktest
    DESTINATION bin/
    )