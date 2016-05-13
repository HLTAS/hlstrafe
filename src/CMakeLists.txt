include_directories (${BOOST_ROOT})
include_directories (${CMAKE_SOURCE_DIR}/hltas/src)

set (SOURCE_FILES
	anglefuncs.cpp
	autofuncs.cpp
	hlstrafe.cpp
	prediction.cpp
	strafefuncs.cpp
	vec.cpp)

add_library (hlstrafe ${SOURCE_FILES})
