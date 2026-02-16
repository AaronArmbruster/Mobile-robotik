cmake_minimum_required(VERSION 3.20)
project("turtlebot3" "CXX")

add_library(${PROJECT_NAME} STATIC "src/dynamixel_sdk_wrapper.cpp")
target_compile_features(${PROJECT_NAME} PUBLIC "cxx_std_17")
target_include_directories(${PROJECT_NAME} PUBLIC "include/")

if (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
  target_link_libraries(${PROJECT_NAME} PUBLIC "dxl_x64_cpp")
else ()
  target_link_libraries(${PROJECT_NAME} PUBLIC "dxl_sbc_cpp")
endif ()