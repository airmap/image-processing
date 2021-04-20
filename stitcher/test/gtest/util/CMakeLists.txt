cmake_minimum_required(VERSION 3.6)

add_library(
    util INTERFACE
)

target_link_libraries(
    util INTERFACE
    ${OpenCV_LIBS}
    gtest
)

target_include_directories(
    util INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/test/gtest/util
)

add_executable(matCompareTests ${CMAKE_CURRENT_SOURCE_DIR}/test/gtest/util/test/mat_compare.cpp)
target_link_libraries(matCompareTests gtest gtest_main util)
add_test(matCompareTests matCompareTests)
