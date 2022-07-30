cmake_minimum_required(VERSION 3.13)

project(hello_android)

find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(CMAKE_VERBOSE_MAKEFILE ON)

add_library(jvm STATIC
    jvm.cc)
target_link_libraries(jvm PUBLIC
    android)

add_library(ros STATIC
    ros_interface.cc
)
target_link_libraries(ros PUBLIC
    rclcpp::rclcpp
    ${sensor_msgs_TARGETS})

add_library(sensors STATIC
    sensors.cc)
target_link_libraries(sensors PUBLIC
    ros)
target_link_libraries(sensors PRIVATE
    jvm)

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
    GLESv3)

add_library(android-ros SHARED android_entry_point.cpp)
target_link_libraries(android-ros PRIVATE
    android
    log
    gui
    ros
    sensors)
target_link_options(android-ros PRIVATE -u ANativeActivity_onCreate)

install(TARGETS android-ros DESTINATION lib)
