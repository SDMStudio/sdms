file(
	GLOB
	usage_examples
	*.cpp
)

foreach(f ${usage_examples})
  get_filename_component(exampleName ${f} NAME_WE) 
  add_executable (${exampleName} ${f}) 
  target_link_libraries(${exampleName} ${TORCH_LIBRARIES} ${Boost_LIBRARIES} ${LIB_SDMS})
  install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/${exampleName}
    DESTINATION bin/
    RENAME "sdms-${exampleName}")
endforeach(f)


#### This section can be uncommented and rewrite in order to only partially compile sdms

# file(
# 	GLOB_RECURSE
# 	lib_struct
# 	${SOURCE_DIR}/utils/struct/*.cpp
# )
# add_library(lib_struct SHARED ${lib_struct})
# include_directories(${SDMS_INCLUDE_DIRS})

# add_executable(simple tmp/simple.cpp)
# target_link_libraries(simple
# ${TORCH_LIBRARIES}
# ${Boost_LIBRARIES}
# ${LIB_SDMS}
# )

# install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/quicktest
#     DESTINATION bin/
#     )