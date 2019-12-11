cmake_minimum_required (VERSION 3.1)
project (hlstrafe)

set (CMAKE_CXX_STANDARD 14)

add_library (hlstrafe src/hlstrafe.cpp)
target_include_directories (hlstrafe PUBLIC hltas/src)
