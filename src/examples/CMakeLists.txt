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
  target_link_libraries(${exampleName} ${LIB_SDMS})
  # install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/${exampleName}
  #   DESTINATION bin/
  #   RENAME "sdms-${exampleName}")
endforeach(f)