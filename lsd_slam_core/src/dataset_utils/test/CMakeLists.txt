enable_testing()

file(GLOB TEST_SOURCES "test/*.cc")

set(TEST_SOURCES
  utils_test.cc
  asl/dataset_test.cc)

add_executable(${PROJECT_NAME}_test ${TEST_SOURCES} ./gtest/gtest-all.cc)
target_link_libraries(${PROJECT_NAME}_test pthread ${PROJECT_NAME})
set_target_properties(${PROJECT_NAME}_test PROPERTIES COMPILE_FLAGS "-std=c++11 -Wno-deprecated-register")

add_test(NAME ${PROJECT_NAME}_test COMMAND ${PROJECT_NAME}_test --gtest_color=yes)

# Copy test data.
file(COPY ./data DESTINATION ${CMAKE_BINARY_DIR})
