file(
	GLOB
	usage_examples
	*.cpp
)

foreach(f ${usage_examples})
  get_filename_component(exampleName ${f} NAME_WE) 
  add_executable (${exampleName} ${f}) 
  target_link_libraries(${exampleName} ${LIB_NAME} ${Boost_SYSTEM_LIBRARIES} ${Boost_FILESYSTEM_LIBRARIES} ${Boost_IOSTREAMS_LIBRARIES} ${Boost_PROGRAM_OPTIONS_LIBRARIES})
  install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/${exampleName}
    DESTINATION bin/
    RENAME ${CMAKE_PROJECT_NAME}-${exampleName})
endforeach(f)

