
enable_testing()

list(APPEND gtests
    main-gtest
)

# run all tests
add_executable(test-all ${gtests})
target_link_libraries(test-all GTest::gtest_main ${PROJECT_NAME})
include(GoogleTest)
gtest_discover_tests(test-all)

# message(STATUS "gci::gTests ----------------------")
# foreach(name ${gci-gtests})
#     message(STATUS " -> ${name}")
#     add_executable(${name} ${name}.cpp)
#     target_link_libraries(${name}
#         GTest::gtest_main
#         ${PROJECT_NAME})
#     add_test(NAME ${name} COMMAND ${name})
# endforeach()

# include(GoogleTest)
# gtest_discover_tests(${gci-gtests})