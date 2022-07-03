
add_executable(serial-struct main.cpp)
target_link_libraries(serial-struct
    PUBLIC
        serialcomm
)
