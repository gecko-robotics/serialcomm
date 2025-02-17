find_package(GTest REQUIRED)
if(GTest_FOUND)
  message(STATUS "=> Found gtest")
else()
  message(STATUS "*** Didn't find gtest")
endif()

enable_testing()

list(APPEND gtests
  main-gtest
)

# run all tests
add_executable(test-all ${gtests}.cpp)
target_link_libraries(test-all GTest::gtest_main ${PROJECT_NAME})
include(GoogleTest)
gtest_discover_tests(test-all)