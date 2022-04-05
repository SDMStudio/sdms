find_package (Boost COMPONENTS system filesystem unit_test_framework iostreams REQUIRED)
find_package(Torch REQUIRED)


add_definitions (-DBOOST -DBOOST_TEST_DYN_LINK)

# Store *.cpp files in a liste
file(
	GLOB
	test_files
	*.cpp
)

# For each .cpp file (sub-programs), make the associated main program
foreach(f ${test_files})
  get_filename_component(testName ${f} NAME_WE) 
  add_executable (${testName} ${f}) 
  target_link_libraries(${testName} ${LIB_SDMS})
  add_test( NAME ${testName} 
            COMMAND ${testName}
          )
endforeach(f)