find_package(GTest REQUIRED)

set(TEST_BINARY ${PROJECT_NAME}_test)

add_executable(${TEST_BINARY}
               tests.cpp)
# if needed
# target_sources(${TEST_BINARY} PRIVATE ${CMAKE_SOURCE_DIR}/src/homework_6.cpp)               
target_link_libraries(${TEST_BINARY} PRIVATE GTest::Main)

gtest_discover_tests(${TEST_BINARY} WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/week2/tests)