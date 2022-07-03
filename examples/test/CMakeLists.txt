
add_executable(serial-test main.cpp)
target_link_libraries(serial-test
    PUBLIC
        serialcomm
)
