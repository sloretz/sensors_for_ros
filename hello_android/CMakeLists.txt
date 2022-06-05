cmake_minimum_required(VERSION 3.13)

project(hello_android)

find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)

set(CMAKE_VERBOSE_MAKEFILE ON)

add_library(events STATIC
    events.cc)
target_compile_features(events PUBLIC cxx_std_17)

add_library(jvm STATIC
    jvm.cc)
target_link_libraries(jvm PUBLIC
    android)

add_library(sensors STATIC
    sensors.cc)
target_link_libraries(sensors PUBLIC
    events
    rclcpp::rclcpp)
target_link_libraries(sensors PRIVATE
    jvm)

add_library(ros STATIC
    ros_interface.cc
)
target_link_libraries(ros PUBLIC
    rclcpp::rclcpp
    events)

add_library(gui STATIC
    gui.cc
    DearImGui/imgui.cpp
    DearImGui/imgui_demo.cpp
    DearImGui/imgui_draw.cpp
    DearImGui/imgui_tables.cpp
    DearImGui/imgui_widgets.cpp
    DearImGui/backends/imgui_impl_android.cpp
    DearImGui/backends/imgui_impl_opengl3.cpp)
target_include_directories(gui PRIVATE
    DearImGui
    DearImGui/backends)
target_link_libraries(gui PUBLIC
    log
    EGL
    GLESv3
    events)

add_library(android-ros SHARED android_entry_point.cpp)
target_link_libraries(android-ros PRIVATE
    android
    log
    gui
    events
    ros
    sensors)
target_link_options(android-ros PRIVATE -u ANativeActivity_onCreate)

install(TARGETS android-ros DESTINATION lib)
