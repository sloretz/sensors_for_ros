cmake_minimum_required(VERSION 3.13)

project(hello_android)

find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)

set(CMAKE_VERBOSE_MAKEFILE ON)

add_library(gui STATIC gui.cc)
target_link_libraries(gui PUBLIC
    log
    EGL
    GLESv1_CM)

add_library(android-ros SHARED android_entry_point.cpp)
target_link_libraries(android-ros PRIVATE
    android
    log
    gui)
target_link_options(android-ros PRIVATE -u ANativeActivity_onCreate)

install(TARGETS android-ros DESTINATION lib)
