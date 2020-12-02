find_package (Boost COMPONENTS system filesystem program_options iostreams REQUIRED)
find_package(Torch REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

add_definitions (-DBOOST)
add_definitions (-DTORCH)
add_definitions (-DDENSE)
add_definitions (-DIL_STD)

include_directories(${CMAKE_SOURCE_DIR}/include
					${Boost_INCLUDE_DIRS}
					${TORCH_INCLUDE_DIRS}
					)

file(
	GLOB_RECURSE
	lib_srcs
	${LIB_NAME}/*.cpp
)

# Build library
add_library(${LIB_NAME} SHARED ${lib_srcs})

# Build main program
add_executable (SDMStudio main.cpp) 
target_link_libraries (SDMStudio ${LIB_NAME} ${Boost_SYSTEM_LIBRARIES} ${Boost_FILESYSTEM_LIBRARIES} ${Boost_IOSTREAMS_LIBRARIES} ${Boost_PROGRAM_OPTIONS_LIBRARIES} ${TORCH_LIBRARIES})

# Install in lib include and bin directories  
install(TARGETS ${LIB_NAME} DESTINATION lib)
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/sdm DESTINATION include)
install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/SDMStudio DESTINATION bin)

add_subdirectory(examples)