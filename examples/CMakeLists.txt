
list(APPEND examples
    serial-struct
    mockserial-writer
    mockserial-reader
)

# message(STATUS "Examples ----------------------")
foreach(app ${examples})
    add_executable(${app} ${app}.cpp)
    target_link_libraries(${app} PRIVATE ${PROJECT_NAME})
endforeach()