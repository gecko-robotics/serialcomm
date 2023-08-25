cmake_minimum_required(VERSION 3.20)
PROJECT(serialcomm VERSION "2023.02.10")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(PROJECT_IS_TOP_LEVEL)
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

    set(BUILD_EXAMPLES ON)
    set(BUILD_GTESTS ON)

    # GTest -----------------
    include(FetchContent)

    FetchContent_Declare(gtest
        GIT_REPOSITORY "https://github.com/google/googletest"
        GIT_TAG "origin/main"
    )

    FetchContent_MakeAvailable(gtest)

else()
    message(STATUS "-> ${PROJECT_NAME} is submodule")
    set(BUILD_EXAMPLES OFF)
    set(BUILD_GTESTS OFF)
endif()


# Build library ----------------------------------------------
# add_library(${PROJECT_NAME}
#     SHARED
#         src/wire_linux.cpp
#         src/wire_apple.cpp
#         src/serial.cpp
# )
# target_include_directories(${PROJECT_NAME}
#     PUBLIC
#         $<INSTALL_INTERFACE:include>
#         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# )


add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE src/)

# ----------------------------------------------
message(STATUS "Building ${PROJECT_NAME} examples is ${BUILD_EXAMPLES}")
if (BUILD_EXAMPLES)
    add_subdirectory(examples/test)
    add_subdirectory(examples/structs)
    # add_subdirectory(examples/messages)
endif()

# ----------------------------------------------
message(STATUS "Building ${PROJECT_NAME} gtests is ${BUILD_GTESTS}")
if (BUILD_GTESTS)
    add_subdirectory(gtests)
endif()
