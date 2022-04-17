cmake_minimum_required(VERSION 3.5)

project(hello_android)

# build native_app_glue as a static lib
# set(${CMAKE_C_FLAGS}, "${CMAKE_C_FLAGS}")
add_library(native_app_glue STATIC
    ${ANDROID_NDK}/sources/android/native_app_glue/android_native_app_glue.c)

# now build app's shared lib
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -Wall -Werror")

# Export ANativeActivity_onCreate(),
# Refer to: https://github.com/android-ndk/ndk/issues/381.
# set(CMAKE_SHARED_LINKER_FLAGS
#   "${CMAKE_SHARED_LINKER_FLAGS} -u ANativeActivity_onCreate")


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

# Configure the manifest
set(APP_TITLE "Hello Android")  # TODO this can be in a resource file instead
set(NAMESPACE "loretz.shane")
set(MIN_SDK_VERSION 30)
set(TARGET_SDK_VERSION 30)
set(LIB_NAME "native-activity")
configure_file(
  AndroidManifest.xml.in
  AndroidManifest.xml
  @ONLY)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/AndroidManifest.xml
  DESTINATION
  stuff_for_apk)

# Destination matches where it will be installed in the APK
install(TARGETS native-activity
  DESTINATION lib/${ANDROID_ABI})


# Need to compile resource files ?
#   aapt2 compile \
#   project_root/module_root/src/main/res/drawable/Image1.png \
#   project_root/module_root/src/main/res/drawable/Image2.png \
#   -o compiled_resources/


