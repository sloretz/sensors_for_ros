cmake_minimum_required(VERSION 3.13)

project(hello_android)

find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)

set(CMAKE_VERBOSE_MAKEFILE ON)

add_library(native_app_glue STATIC
    ${ANDROID_NDK}/sources/android/native_app_glue/android_native_app_glue.c)


add_library(color_listener STATIC color_listener.cpp)
target_link_libraries(color_listener PUBLIC
  # rclcpp_lifecycle::rclcpp_lifecycle
  rclcpp::rclcpp
  ${std_msgs_TARGETS})


add_library(native-activity SHARED main.cpp)

target_include_directories(native-activity PRIVATE
    ${ANDROID_NDK}/sources/android/native_app_glue)

# add lib dependencies
target_link_libraries(native-activity PRIVATE
    android
    native_app_glue
    EGL
    GLESv1_CM
    log
    color_listener
    rclcpp::rclcpp
    ${std_msgs_TARGETS})

# TODO(sloretz) use target_link_options in CMake 3.13
set_property(TARGET native-activity APPEND_STRING PROPERTY LINK_FLAGS " -u ANativeActivity_onCreate")

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

install(TARGETS native-activity DESTINATION lib)
