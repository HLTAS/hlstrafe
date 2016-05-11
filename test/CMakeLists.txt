add_executable(vec-test vec-test.cpp)
target_link_libraries(vec-test hlstrafe)

add_test(COMMAND vec-test)
