file(
	GLOB
  algos
	*.cpp
)

foreach(f ${algos})
  get_filename_component(algoName ${f} NAME_WE) 
  add_executable (${algoName} ${f}) 
  target_link_libraries(${algoName} ${TORCH_LIBRARIES} ${Boost_LIBRARIES} ${LIB_SDMS})
  install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/${algoName}
    DESTINATION bin/
    RENAME ${CMAKE_PROJECT_NAME}-${algoName}
    COMPONENT ${algoName})
endforeach(f)