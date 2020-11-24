find_package (Boost COMPONENTS system filesystem unit_test_framework iostreams REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/include
					${Boost_INCLUDE_DIRS}
					)
          
add_definitions (-DBOOST -DDENSE -DBOOST_TEST_DYN_LINK)

add_executable (test_world test_world.cpp) 
target_link_libraries(test_world
  ${LIB_NAME}
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
  ${Boost_IOSTREAMS_LIBRARY}
  ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
  )
