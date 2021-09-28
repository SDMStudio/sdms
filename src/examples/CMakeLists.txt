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
  target_link_libraries(${exampleName} dl ${Boost_LIBRARIES} ${TORCH_LIBRARIES} ${TB2_LIBRARIES} ${LIB_SDMS})
  install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/${exampleName}
    DESTINATION bin/
    RENAME "sdms-${exampleName}")
endforeach(f)


### This section can be uncommented and rewrite in order to compile only a part of sdms (quicktest.cpp file)
add_executable(quicktest tmp/quicktest.cpp)
target_link_libraries(quicktest
fmt
dl
${Boost_LIBRARIES}
${TORCH_LIBRARIES}
${SDMS_LIB_DIR}/libtb2.so
${CPLEX_LIBRARIES}
${LIB_SDMS}
)

install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/quicktest
    DESTINATION bin/
)

add_subdirectory(tmp)