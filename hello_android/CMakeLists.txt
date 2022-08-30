cmake_minimum_required(VERSION 3.13)

project(hello_android)

find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(CMAKE_VERBOSE_MAKEFILE ON)

add_library(imgui STATIC
    DearImGui/imgui.cpp
    DearImGui/imgui_demo.cpp
    DearImGui/imgui_draw.cpp
    DearImGui/imgui_tables.cpp
    DearImGui/imgui_widgets.cpp
    DearImGui/backends/imgui_impl_android.cpp
    DearImGui/backends/imgui_impl_opengl3.cpp)
target_include_directories(imgui PUBLIC
    DearImGui
    DearImGui/backends)
target_link_libraries(imgui PUBLIC
    EGL
    GLESv3)

add_library(android-ros SHARED
  
  android_entry_point.cc
  controllers/accelerometer_sensor_controller.cc
  controllers/barometer_sensor_controller.cc
  controllers/gyroscope_sensor_controller.cc
  controllers/illuminance_sensor_controller.cc
  controllers/ros_domain_id_controller.cc
  controllers/sensor_list_controller.cc
  gui.cc
  jvm.cc
  ros_interface.cc
  sensor.cc
  sensor_descriptor.cc
  sensors.cc
  sensors/accelerometer_sensor.cc
  sensors/barometer_sensor.cc
  sensors/gyroscope_sensor.cc
  sensors/illuminance_sensor.cc
)
target_link_libraries(android-ros PRIVATE
    android
    log
    imgui
    rclcpp::rclcpp
    ${geometry_msgs_TARGETS}
    ${sensor_msgs_TARGETS})
target_link_options(android-ros PRIVATE -u ANativeActivity_onCreate)
target_include_directories(android-ros PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

install(TARGETS android-ros DESTINATION lib)
