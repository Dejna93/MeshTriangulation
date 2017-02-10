include_directories(${googletest_SOURCE_DIR}/include ${googletest_SOURCE_DIR})

add_executable(runBasicTest *.cpp)

target_link_libraries(runBasicTest gtest gtest_main)
target_link_libraries(runBasicTest src)