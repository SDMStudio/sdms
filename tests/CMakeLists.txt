find_package (Boost COMPONENTS system filesystem unit_test_framework iostreams REQUIRED)
find_package(Torch REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

include_directories(${CMAKE_SOURCE_DIR}/include
          ${Boost_INCLUDE_DIRS}
					${TORCH_INCLUDE_DIRS}
					)
          
add_definitions (-DBOOST -DDENSE -DBOOST_TEST_DYN_LINK)

add_executable (test_world test_world.cpp) 
target_link_libraries(test_world
  ${LIB_NAME}
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
  ${Boost_IOSTREAMS_LIBRARY}
  ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
  ${TORCH_LIBRARIES}
  )

add_executable (test_space test_space.cpp) 
target_link_libraries(test_space
  ${LIB_NAME}
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
  ${Boost_IOSTREAMS_LIBRARY}
  ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
  ${TORCH_LIBRARIES}
  )
