# GoogleTest: each TEST is its own ctest test (gtest_discover_tests).
find_package(GTest REQUIRED)
include(GoogleTest)
add_executable(tests pole_test.cpp)
target_link_libraries(tests PRIVATE pole_estimator GTest::gtest_main)
gtest_discover_tests(tests)
