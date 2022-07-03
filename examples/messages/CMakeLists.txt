
add_executable(message-test main.cpp)
target_link_libraries(message-test
    PUBLIC
        serialcomm
)
