cmake_minimum_required(VERSION 3.10)
PROJECT(mv VERSION "0.0.1")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

cmake_host_system_information(RESULT HOST QUERY HOSTNAME)
# cmake_host_system_information(RESULT CORES QUERY NUMBER_OF_LOGICAL_CORES)
cmake_host_system_information(RESULT OSN QUERY OS_NAME)
cmake_host_system_information(RESULT OS_VERSION QUERY OS_RELEASE)
cmake_host_system_information(RESULT PROC QUERY PROCESSOR_DESCRIPTION)

message(STATUS "-------------------------------------")
message(STATUS "  Project: ${PROJECT_NAME}")
message(STATUS "  C++ ${CMAKE_CXX_STANDARD}")
message(STATUS "-------------------------------------")
message(STATUS " ${HOST}")
message(STATUS " ${OSN}: ${OS_VERSION}")
message(STATUS " ${PROC}")
message(STATUS "-------------------------------------")

# configure_file(${path to configure file 'config.h.in'}
# include_directories(${PROJECT_BINARY_BIN}) // this allows the 'config.h' file to be used throughout the program

# set(CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
# set(CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VERSION_MINOR})
# set(CPACK_PACKAGE_VERSION_PATCH ${PROJECT_VERSION_PATCH})
# include(CPack)

# Boost -----------------------------------------------------------
# find_package(Boost REQUIRED system)

# Threads ---------------------------------------------------------
# find_package(Threads REQUIRED)

# Pi ---------------------------------------------------------------
# Need to figure out if we need to link against pigpio or not
# if (PI)
#     set(OTHER "pigpio rt")
# else ()
#     set(OTHER "")
# endif (PI)

# Build AX-12 library ----------------------------------------------
set(LIB
    src/SerialPort.cpp
    src/protocol1.cpp
    src/protocol2.cpp
    src/ax12.cpp
)
add_library(${PROJECT_NAME} ${LIB})
target_include_directories(${PROJECT_NAME}
    PUBLIC
        src/
        # ${Boost_INCLUDE_DIRS}
)
# target_link_libraries(${PROJECT_NAME}
#     PUBLIC
#         # ${Boost_SYSTEM_LIBRARY}
#         ${Boost_LIBRARIES}
# )

# Examples -------------------------------------------------------
add_subdirectory(examples)
# add_executable(ax12-ex examples/ax12-ex.cpp)
# target_link_libraries(ax12-ex
#     PUBLIC
#         ${PROJECT_NAME}
#         # ${CMAKE_THREAD_LIBS_INIT}
#         # ${OTHER}
# )
# target_compile_definitions(main PRIVATE RPI=1)

# Tests --------------------------------------------------------------
option(TESTS "gecko google tests" ON)
if(${TESTS})
    add_subdirectory(test)
endif()
