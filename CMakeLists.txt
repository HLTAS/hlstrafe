cmake_minimum_required (VERSION 3.1)
project (hlstrafe)

set (CMAKE_CXX_STANDARD 11)
include_directories (include)

add_subdirectory(src)

if (BUILD_TESTING)
	enable_testing()
	add_subdirectory(test)
endif ()
