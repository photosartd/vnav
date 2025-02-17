cmake_minimum_required(VERSION 3.10)

# Create an executable for this week's task
add_executable(week_2 main.cpp)

# Include tests only if the directory exists
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests)
    add_subdirectory(tests)
endif()