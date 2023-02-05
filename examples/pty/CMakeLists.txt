
add_executable(pty pty.cpp)
target_link_libraries(pty
    PUBLIC
        serialcomm
)
