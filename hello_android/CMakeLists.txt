cmake_minimum_required(VERSION 3.5)

project(hello_android)

add_library(native_app_glue STATIC
    ${ANDROID_NDK}/sources/android/native_app_glue/android_native_app_glue.c)

add_library(native-activity SHARED main.cpp)

target_include_directories(native-activity PRIVATE
    ${ANDROID_NDK}/sources/android/native_app_glue)

# add lib dependencies
target_link_libraries(native-activity PRIVATE
    android
    native_app_glue
    EGL
    GLESv1_CM
    log)
# TODO(sloretz) use target_link_options in CMake 3.13
set_property(TARGET native-activity APPEND_STRING PROPERTY LINK_FLAGS " -u ANativeActivity_onCreate")


# Destination matches where it will be installed in the APK
install(TARGETS native-activity
  DESTINATION lib/${ANDROID_ABI})
