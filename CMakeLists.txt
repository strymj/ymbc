cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(ymbc)

# set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
# set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# set the build type
set(CMAKE_BUILD_TYPE Release)
# confirmation messages
message(STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")
message(STATUS "CMAKE_C_FLAGS: ${CMAKE_C_FLAGS}")
message(STATUS "CMAKE_C_FLAGS_RELEASE: ${CMAKE_C_FLAGS_RELEASE}")
message(STATUS "CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")
message(STATUS "CMAKE_CXX_FLAGS_RELEASE: ${CMAKE_CXX_FLAGS_RELEASE}")

#include_directories(/usr/local/include/mylib1 /usr/local/include/mylib2)
#link_directories(/usr/local/lib/mylib1 /usr/local/lib/mylib2)
#add_definitions(-DMYDEFINITION1 -DMYDEFINITION2)

# set the executable name, sources, libraries
include_directories(src)
add_library(ymbc
	src/ymbc.cpp)
add_executable(kadai1
	src/kadai1.cpp)
target_link_libraries(kadai1
	ymbc
	ypspur)
