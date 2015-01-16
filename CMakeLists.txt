cmake_minimum_required (VERSION 3.1)
project (hlstrafe)

set (CMAKE_CXX_STANDARD 11)

include_directories (hltas/src)

add_library (hlstrafe src/hlstrafe.cpp)
