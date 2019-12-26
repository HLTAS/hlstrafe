cmake_minimum_required(VERSION 3.1)
project(hlstrafe)

add_subdirectory(hltas)

add_library(hlstrafe src/hlstrafe.cpp)
set_property(TARGET hlstrafe PROPERTY CXX_STANDARD 14)
target_include_directories(hlstrafe PUBLIC src)
target_link_libraries(hlstrafe hltas-cpp)
