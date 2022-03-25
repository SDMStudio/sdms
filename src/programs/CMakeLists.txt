# Store *.cpp files in a liste
file(
	GLOB
	main_programs
	*.cpp
)

# For each .cpp file (sub-programs), make the associated main program
foreach(f ${main_programs})
  get_filename_component(programName ${f} NAME_WE) 
  add_executable (${programName} ${f}) 
  target_link_libraries(${programName} -L${SDMS_LIB_DIR} dl ${Boost_LIBRARIES} ${TORCH_LIBRARIES} ${TB2_LIBRARIES} ${LIB_SDMS_STATIC})
  install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/${programName}
    DESTINATION bin/
    RENAME "sdms-${programName}")
endforeach(f)
