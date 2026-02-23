cmake_minimum_required(VERSION 3.20)
project("assignment1" "CXX")

# Task 1
add_executable("${PROJECT_NAME}_task1" "src/task1.cpp")
set_target_properties("${PROJECT_NAME}_task1" PROPERTIES OUTPUT_NAME "task1")
target_compile_features("${PROJECT_NAME}_task1" PUBLIC "cxx_std_17")
target_include_directories("${PROJECT_NAME}_task1" PRIVATE "${CMAKE_SOURCE_DIR}/turtlebot3/include/")
target_link_directories("${PROJECT_NAME}_task1" PRIVATE "turtlebot3/")
target_link_libraries("${PROJECT_NAME}_task1" PRIVATE "turtlebot3")

# Task 2
add_executable("${PROJECT_NAME}_task2" "src/task2.cpp")
set_target_properties("${PROJECT_NAME}_task2" PROPERTIES OUTPUT_NAME "task2")
target_compile_features("${PROJECT_NAME}_task2" PUBLIC "cxx_std_17")
target_include_directories("${PROJECT_NAME}_task2" PRIVATE "${CMAKE_SOURCE_DIR}/turtlebot3/include/")
target_link_directories("${PROJECT_NAME}_task2" PRIVATE "turtlebot3/")
target_link_libraries("${PROJECT_NAME}_task2" PRIVATE "turtlebot3")

# Task 3
add_executable("${PROJECT_NAME}_task3" "src/task3.cpp")
set_target_properties("${PROJECT_NAME}_task3" PROPERTIES OUTPUT_NAME "task3")
target_compile_features("${PROJECT_NAME}_task3" PUBLIC "cxx_std_17")
target_include_directories("${PROJECT_NAME}_task3" PRIVATE "${CMAKE_SOURCE_DIR}/turtlebot3/include/")
target_link_directories("${PROJECT_NAME}_task3" PRIVATE "turtlebot3/")
target_link_libraries("${PROJECT_NAME}_task3" PRIVATE "turtlebot3")