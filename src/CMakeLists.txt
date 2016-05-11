include_directories (${BOOST_ROOT})
include_directories (${CMAKE_SOURCE_DIR}/hltas/src)

set (SOURCE_FILES
	hlstrafe.cpp
	vec.cpp)

add_library (hlstrafe ${SOURCE_FILES})
