cmake_minimum_required(VERSION 3.20)
project("assignment1" "CXX")

# Task 1
add_executable("task1" "src/task1.cpp")
target_compile_features("task1" PUBLIC "cxx_std_17")
target_include_directories("task1" PRIVATE "${CMAKE_SOURCE_DIR}/turtlebot3/include/")
target_link_directories("task1" PRIVATE "turtlebot3/")
target_link_libraries("task1" PRIVATE "turtlebot3")

# Task 2
add_executable("task2" "src/task2.cpp")
target_compile_features("task2" PUBLIC "cxx_std_17")
target_include_directories("task2" PRIVATE "${CMAKE_SOURCE_DIR}/turtlebot3/include/")
target_link_directories("task2" PRIVATE "turtlebot3/")
target_link_libraries("task2" PRIVATE "turtlebot3")

# Task 3
add_executable("task3" "src/task3.cpp")
target_compile_features("task3" PUBLIC "cxx_std_17")
target_include_directories("task3" PRIVATE "${CMAKE_SOURCE_DIR}/turtlebot3/include/")
target_link_directories("task3" PRIVATE "turtlebot3/")
target_link_libraries("task3" PRIVATE "turtlebot3")