cmake_minimum_required(VERSION 3.13)

project(hello_android)

find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)

set(CMAKE_VERBOSE_MAKEFILE ON)

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
    gui)
target_link_options(android-ros PRIVATE -u ANativeActivity_onCreate)

install(TARGETS android-ros DESTINATION lib)
