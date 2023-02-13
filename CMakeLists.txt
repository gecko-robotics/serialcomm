cmake_minimum_required(VERSION 3.20)
PROJECT(serialcomm VERSION "2023.02.10")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

cmake_host_system_information(RESULT HOST QUERY HOSTNAME)
cmake_host_system_information(RESULT OSN QUERY OS_NAME)
cmake_host_system_information(RESULT OS_VERSION QUERY OS_RELEASE)
cmake_host_system_information(RESULT PROC QUERY PROCESSOR_DESCRIPTION)

message(STATUS "-------------------------------------")
message(STATUS "  Project: ${PROJECT_NAME}")
message(STATUS "  Version: ${CMAKE_PROJECT_VERSION}")
message(STATUS "  C++ ${CMAKE_CXX_STANDARD}")
message(STATUS "-------------------------------------")
message(STATUS " ${HOST}")
message(STATUS " ${OSN}: ${OS_VERSION}")
message(STATUS " ${PROC}")
message(STATUS "-------------------------------------")


# Build library ----------------------------------------------
add_library(${PROJECT_NAME}
    SHARED
        src/wire.cpp
        src/serial.cpp
)
target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

option(BUILD_EXAMPLES "Build examples" ON)
message(STATUS "Building serialcomm examples is ${BUILD_EXAMPLES}")
if (BUILD_EXAMPLES)
    add_subdirectory(examples/test)
    add_subdirectory(examples/structs)
    add_subdirectory(examples/messages)
endif()

option(BUILD_GTESTS "build gtests" ON)
message(STATUS "Building serialcomm gtests is ${BUILD_GTESTS}")
if (BUILD_GTESTS)
    # add_subdirectory(gtests)
endif()
