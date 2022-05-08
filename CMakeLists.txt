cmake_minimum_required(VERSION 3.12)

project(anroid_ros-superbuild)

include(ExternalProject)

set(minimum_target_sdk 30)

if(NOT ANDROID_HOME)
  message(FATAL_ERROR "ANDROID_HOME must be given")
endif()
if(NOT ANDROID_HOME)
  message(FATAL_ERROR "ANDROID_HOME must be given")
endif()

if(NOT PATH_TO_KEYSTORE)
  set(PATH_TO_KEYSTORE "$ENV{HOME}/.android/debug.keystore")
endif()
message(STATUS "Given keystore ${PATH_TO_KEYSTORE}")

file(GLOB android_toolchain
  LIST_DIRECTORIES false
  "${ANDROID_HOME}/ndk/*/build/cmake/android.toolchain.cmake"
  )
if(NOT android_toolchain)
  message(FATAL_ERROR "Failed to find android toolchain file - is ANDROID_HOME correct?")
endif()

file(GLOB android_aapt2
  LIST_DIRECTORIES false
  "${ANDROID_HOME}/build-tools/*/aapt2"
  )
if(NOT android_aapt2)
  message(FATAL_ERROR "Failed to find aapt2 - is ANDROID_HOME correct?")
endif()

file(GLOB android_jar
  LIST_DIRECTORIES false
  "${ANDROID_HOME}/platforms/android-${minimum_target_sdk}/android.jar"
  )
if(NOT android_jar)
  message(FATAL_ERROR "Failed to find android.jar - is ANDROID_HOME correct?")
endif()

file(GLOB android_zipalign
  LIST_DIRECTORIES false
  "${ANDROID_HOME}/build-tools/*/zipalign"
  )
if(NOT android_zipalign)
  message(FATAL_ERROR "Failed to find zipalign - is ANDROID_HOME correct?")
endif()

file(GLOB android_apksigner
  LIST_DIRECTORIES false
  "${ANDROID_HOME}/build-tools/*/apksigner"
  )
if(NOT android_apksigner)
  message(FATAL_ERROR "Failed to find apksigner - is ANDROID_HOME correct?")
endif()

set(ANDROID_ABI arm64-v8a)

set(android_cmake_args 
  -DCMAKE_TOOLCHAIN_FILE=${android_toolchain}
  -DANDROID_ABI=${ANDROID_ABI}
  -DANDROID_PLATFORM=android-${minimum_target_sdk}
  )

# Make place to install Python packages we depend on
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/deps/_python_)

include(dependencies.cmake)

build_native_dependencies()

build_crosscompile_dependencies()

##################################
# Android specific projects
##################################


set(cmake_with_env "${CMAKE_COMMAND}" -E
  env
  "PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/deps/_python_"
  "AMENT_PREFIX_PATH=${CMAKE_CURRENT_BINARY_DIR}/deps"
  "${CMAKE_COMMAND}")

ExternalProject_Add(hello_android
  BUILD_ALWAYS ON
  DOWNLOAD_COMMAND ""
  CMAKE_COMMAND ${cmake_with_env}
  BUILD_COMMAND ${cmake_with_env} --build .
  SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/hello_android"
  DEPENDS
  deps-rclcpp
  CMAKE_ARGS
  ${android_cmake_args}
  "-DCMAKE_FIND_ROOT_PATH=${CMAKE_CURRENT_BINARY_DIR}/deps"
  -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/hello_android
  # -DCMAKE_VERBOSE_MAKEFILE=ON
  )

# Configure the manifest
set(NAMESPACE "loretz.shane")
set(MIN_SDK_VERSION 30)
set(TARGET_SDK_VERSION 30)
set(LIB_NAME "native-activity")
configure_file(
  AndroidManifest.xml.in
  AndroidManifest.xml
  @ONLY)

# Make a place for compiled resources to go
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/compiled_resources)

# Compile resources
add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/compiled_resources/values_strings.arsc.flat
  COMMENT "compiling res/values/strings.xml"
  COMMAND ${android_aapt2} compile
  ${CMAKE_CURRENT_SOURCE_DIR}/res/values/strings.xml
  -o ${CMAKE_CURRENT_BINARY_DIR}/compiled_resources
  )
add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/compiled_resources/mipmap-hdpi_ic_launcher.png.flat
  COMMENT "compiling res/mipmap-hdpi/ic_launcher.png"
  COMMAND ${android_aapt2} compile
  ${CMAKE_CURRENT_SOURCE_DIR}/res/mipmap-hdpi/ic_launcher.png
  -o ${CMAKE_CURRENT_BINARY_DIR}/compiled_resources
  )
add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/compiled_resources/mipmap-mdpi_ic_launcher.png.flat
  COMMENT "compiling res/mipmap-mdpi/ic_launcher.png"
  COMMAND ${android_aapt2} compile
  ${CMAKE_CURRENT_SOURCE_DIR}/res/mipmap-mdpi/ic_launcher.png
  -o ${CMAKE_CURRENT_BINARY_DIR}/compiled_resources
  )
add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/compiled_resources/mipmap-xhdpi_ic_launcher.png.flat
  COMMENT "compiling res/mipmap-xhdpi/ic_launcher.png"
  COMMAND ${android_aapt2} compile
  ${CMAKE_CURRENT_SOURCE_DIR}/res/mipmap-xhdpi/ic_launcher.png
  -o ${CMAKE_CURRENT_BINARY_DIR}/compiled_resources
  )
add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/compiled_resources/mipmap-xxhdpi_ic_launcher.png.flat
  COMMENT "compiling res/mipmap-xxhdpi/ic_launcher.png"
  COMMAND ${android_aapt2} compile
  ${CMAKE_CURRENT_SOURCE_DIR}/res/mipmap-xxhdpi/ic_launcher.png
  -o ${CMAKE_CURRENT_BINARY_DIR}/compiled_resources
  )

# Link resources into an APK that doesn't have the libs yet
add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/android_ros-nolibs.apk
  COMMENT "linking android_ros-nolibs.apk"
  DEPENDS
  ${CMAKE_CURRENT_BINARY_DIR}/compiled_resources/values_strings.arsc.flat
  ${CMAKE_CURRENT_BINARY_DIR}/compiled_resources/mipmap-hdpi_ic_launcher.png.flat
  ${CMAKE_CURRENT_BINARY_DIR}/compiled_resources/mipmap-mdpi_ic_launcher.png.flat
  ${CMAKE_CURRENT_BINARY_DIR}/compiled_resources/mipmap-xhdpi_ic_launcher.png.flat
  ${CMAKE_CURRENT_BINARY_DIR}/compiled_resources/mipmap-xxhdpi_ic_launcher.png.flat
  COMMAND ${android_aapt2} link
  -o "${CMAKE_CURRENT_BINARY_DIR}/android_ros-nolibs.apk"
  -I "${android_jar}"
  --manifest "${CMAKE_CURRENT_BINARY_DIR}/AndroidManifest.xml"
  -R
  ${CMAKE_CURRENT_BINARY_DIR}/compiled_resources/values_strings.arsc.flat
  ${CMAKE_CURRENT_BINARY_DIR}/compiled_resources/mipmap-hdpi_ic_launcher.png.flat
  ${CMAKE_CURRENT_BINARY_DIR}/compiled_resources/mipmap-mdpi_ic_launcher.png.flat
  ${CMAKE_CURRENT_BINARY_DIR}/compiled_resources/mipmap-xhdpi_ic_launcher.png.flat
  ${CMAKE_CURRENT_BINARY_DIR}/compiled_resources/mipmap-xxhdpi_ic_launcher.png.flat
  --auto-add-overlay
  )

##############################
# Add compiled libs to the APK

file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib/${ANDROID_ABI})

# I'm sure I don't need all these, but I don't know how to run ldd on different arch
set(deps_libs
  libactionlib_msgs__rosidl_generator_c.so
  libactionlib_msgs__rosidl_typesupport_cpp.so
  libactionlib_msgs__rosidl_typesupport_c.so
  libactionlib_msgs__rosidl_typesupport_introspection_cpp.so
  libactionlib_msgs__rosidl_typesupport_introspection_c.so
  libament_index_cpp.so
  libbuiltin_interfaces__rosidl_generator_c.so
  libbuiltin_interfaces__rosidl_typesupport_cpp.so
  libbuiltin_interfaces__rosidl_typesupport_c.so
  libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.so
  libbuiltin_interfaces__rosidl_typesupport_introspection_c.so
  libcomposition_interfaces__rosidl_generator_c.so
  libcomposition_interfaces__rosidl_typesupport_cpp.so
  libcomposition_interfaces__rosidl_typesupport_c.so
  libcomposition_interfaces__rosidl_typesupport_introspection_cpp.so
  libcomposition_interfaces__rosidl_typesupport_introspection_c.so
  libddsc.so
  libdiagnostic_msgs__rosidl_generator_c.so
  libdiagnostic_msgs__rosidl_typesupport_cpp.so
  libdiagnostic_msgs__rosidl_typesupport_c.so
  libdiagnostic_msgs__rosidl_typesupport_introspection_cpp.so
  libdiagnostic_msgs__rosidl_typesupport_introspection_c.so
  libgeometry_msgs__rosidl_generator_c.so
  libgeometry_msgs__rosidl_typesupport_cpp.so
  libgeometry_msgs__rosidl_typesupport_c.so
  libgeometry_msgs__rosidl_typesupport_introspection_cpp.so
  libgeometry_msgs__rosidl_typesupport_introspection_c.so
  liblibstatistics_collector.so
  liblifecycle_msgs__rosidl_generator_c.so
  liblifecycle_msgs__rosidl_typesupport_cpp.so
  liblifecycle_msgs__rosidl_typesupport_c.so
  liblifecycle_msgs__rosidl_typesupport_introspection_cpp.so
  liblifecycle_msgs__rosidl_typesupport_introspection_c.so
  libnav_msgs__rosidl_generator_c.so
  libnav_msgs__rosidl_typesupport_cpp.so
  libnav_msgs__rosidl_typesupport_c.so
  libnav_msgs__rosidl_typesupport_introspection_cpp.so
  libnav_msgs__rosidl_typesupport_introspection_c.so
  librclcpp_lifecycle.so
  librclcpp.so
  librcl_interfaces__rosidl_generator_c.so
  librcl_interfaces__rosidl_typesupport_cpp.so
  librcl_interfaces__rosidl_typesupport_c.so
  librcl_interfaces__rosidl_typesupport_introspection_cpp.so
  librcl_interfaces__rosidl_typesupport_introspection_c.so
  librcl_lifecycle.so
  librcl_logging_interface.so
  librcl_logging_android.so
  librcl_logging_noop.so
  librcl_logging_rcutils.so
  librcl_logging_spdlog.so
  librcl.so
  librcl_yaml_param_parser.so
  librcpputils.so
  librcutils.so
  librmw_cyclonedds_cpp.so
  librmw_dds_common__rosidl_generator_c.so
  librmw_dds_common__rosidl_typesupport_cpp.so
  librmw_dds_common__rosidl_typesupport_c.so
  librmw_dds_common__rosidl_typesupport_introspection_cpp.so
  librmw_dds_common__rosidl_typesupport_introspection_c.so
  librmw_dds_common.so
  librmw.so
  librosgraph_msgs__rosidl_generator_c.so
  librosgraph_msgs__rosidl_typesupport_cpp.so
  librosgraph_msgs__rosidl_typesupport_c.so
  librosgraph_msgs__rosidl_typesupport_introspection_cpp.so
  librosgraph_msgs__rosidl_typesupport_introspection_c.so
  librosidl_runtime_c.so
  librosidl_typesupport_cpp.so
  librosidl_typesupport_c.so
  librosidl_typesupport_introspection_cpp.so
  librosidl_typesupport_introspection_c.so
  libsensor_msgs__rosidl_generator_c.so
  libsensor_msgs__rosidl_typesupport_cpp.so
  libsensor_msgs__rosidl_typesupport_c.so
  libsensor_msgs__rosidl_typesupport_introspection_cpp.so
  libsensor_msgs__rosidl_typesupport_introspection_c.so
  libshape_msgs__rosidl_generator_c.so
  libshape_msgs__rosidl_typesupport_cpp.so
  libshape_msgs__rosidl_typesupport_c.so
  libshape_msgs__rosidl_typesupport_introspection_cpp.so
  libshape_msgs__rosidl_typesupport_introspection_c.so
  libspdlog.so
  libstatistics_msgs__rosidl_generator_c.so
  libstatistics_msgs__rosidl_typesupport_cpp.so
  libstatistics_msgs__rosidl_typesupport_c.so
  libstatistics_msgs__rosidl_typesupport_introspection_cpp.so
  libstatistics_msgs__rosidl_typesupport_introspection_c.so
  libstd_msgs__rosidl_generator_c.so
  libstd_msgs__rosidl_typesupport_cpp.so
  libstd_msgs__rosidl_typesupport_c.so
  libstd_msgs__rosidl_typesupport_introspection_cpp.so
  libstd_msgs__rosidl_typesupport_introspection_c.so
  libstd_srvs__rosidl_generator_c.so
  libstd_srvs__rosidl_typesupport_cpp.so
  libstd_srvs__rosidl_typesupport_c.so
  libstd_srvs__rosidl_typesupport_introspection_cpp.so
  libstd_srvs__rosidl_typesupport_introspection_c.so
  libstereo_msgs__rosidl_generator_c.so
  libstereo_msgs__rosidl_typesupport_cpp.so
  libstereo_msgs__rosidl_typesupport_c.so
  libstereo_msgs__rosidl_typesupport_introspection_cpp.so
  libstereo_msgs__rosidl_typesupport_introspection_c.so
  libtracetools.so
  libtracetools_status.so
  libtrajectory_msgs__rosidl_generator_c.so
  libtrajectory_msgs__rosidl_typesupport_cpp.so
  libtrajectory_msgs__rosidl_typesupport_c.so
  libtrajectory_msgs__rosidl_typesupport_introspection_cpp.so
  libtrajectory_msgs__rosidl_typesupport_introspection_c.so
  libvisualization_msgs__rosidl_generator_c.so
  libvisualization_msgs__rosidl_typesupport_cpp.so
  libvisualization_msgs__rosidl_typesupport_c.so
  libvisualization_msgs__rosidl_typesupport_introspection_cpp.so
  libvisualization_msgs__rosidl_typesupport_introspection_c.so
  libyaml.so)

# Copy library to ABI specific folder
foreach(lib ${deps_libs})
  set(destination "${CMAKE_CURRENT_BINARY_DIR}/lib/${ANDROID_ABI}/${lib}")
  list(APPEND deps_libs_zippable "${destination}")

  add_custom_target("copy_${lib}"
    ALL
    BYPRODUCTS "${destination}"
    COMMENT "Copying ${lib} to folder for zipping"
    DEPENDS
    deps-ament_index_python
    deps-ament_package
    deps-domain_coordinator
    deps-rosidl_cli
    deps-ament_cmake_core
    deps-cyclonedds
    deps-ament_cmake_export_include_directories
    deps-ament_cmake_export_dependencies
    deps-ament_cmake_export_definitions
    deps-ament_cmake_export_link_flags
    deps-ament_cmake_export_targets
    deps-ament_cmake_gmock
    deps-ament_cmake_export_interfaces
    deps-ament_cmake_export_libraries
    deps-ament_cmake_include_directories
    deps-ament_cmake_pytest
    deps-ament_cmake_gtest
    deps-ament_cmake_libraries
    deps-ament_cmake_gen_version_h
    deps-ament_cmake_nose
    deps-ament_cmake_python
    deps-ament_cmake_target_dependencies
    deps-ament_cmake_version
    deps-ament_cmake
    deps-ament_cmake_google_benchmark
    deps-ament_cmake_test
    deps-ament_cmake_ros
    deps-rosidl_adapter
    deps-ament_cmake_auto
    deps-libyaml_vendor
    deps-rosidl_cmake
    deps-common_interfaces
    deps-ament_index_cpp
    deps-rmw_implementation_cmake
    deps-rosidl_parser
    deps-rosidl_default_generators
    deps-rosidl_typesupport_introspection_c
    deps-spdlog_vendor
    deps-rosidl_generator_c
    deps-rosidl_typesupport_interface
    deps-rosidl_generator_cpp
    deps-rosidl_default_runtime
    deps-rcutils
    deps-tracetools
    deps-rosidl_runtime_c
    deps-rcpputils
    deps-rcl_logging_interface
    deps-rmw
    deps-rosidl_runtime_cpp
    deps-rosidl_typesupport_c
    deps-rcl_logging_android
    deps-rcl_logging_noop
    deps-rcl_logging_rcutils
    deps-rcl_logging_spdlog
    deps-rosidl_typesupport_cpp
    deps-rcl_yaml_param_parser
    deps-rosidl_typesupport_introspection_cpp
    deps-rmw_dds_common
    deps-lifecycle_msgs
    deps-std_srvs
    deps-builtin_interfaces
    deps-rosgraph_msgs
    deps-rmw_cyclonedds_cpp
    deps-std_msgs
    deps-statistics_msgs
    deps-rcl_interfaces
    deps-rmw_implementation
    deps-geometry_msgs
    deps-composition_interfaces
    deps-actionlib_msgs
    deps-rcl
    deps-trajectory_msgs
    deps-diagnostic_msgs
    deps-sensor_msgs
    deps-nav_msgs
    deps-shape_msgs
    deps-rcl_lifecycle
    deps-libstatistics_collector
    deps-visualization_msgs
    deps-stereo_msgs
    deps-rclcpp
    deps-rclcpp_lifecycle
    WORKING_DIRECTORY
    ${CMAKE_CURRENT_BINARY_DIR}/deps/
    COMMAND
    ${CMAKE_COMMAND} -E copy_if_different
    "lib/${lib}"
    "${CMAKE_CURRENT_BINARY_DIR}/lib/${ANDROID_ABI}/"
    )
endforeach()

# Copy hello_android's library to ABI specific folder
add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/lib/${ANDROID_ABI}/libnative-activity.so
  COMMENT "Copying hello_android's libs to folder for zipping"
  DEPENDS
  hello_android
  WORKING_DIRECTORY
  ${CMAKE_CURRENT_BINARY_DIR}/hello_android
  COMMAND
  ${CMAKE_COMMAND} -E copy_if_different
  "lib/libnative-activity.so"
  "${CMAKE_CURRENT_BINARY_DIR}/lib/${ANDROID_ABI}/"
  )

# Add all shared libraries to APK
add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/android_ros-unaligned-unsigned.apk
  COMMENT "Adding libs to android_ros-nolibs.apk to make android_ros-unaligned-unsigned.apk"
  DEPENDS
  ${deps_libs_zippable}
  ${CMAKE_CURRENT_BINARY_DIR}/lib/${ANDROID_ABI}/libnative-activity.so
  ${CMAKE_CURRENT_BINARY_DIR}/android_ros-nolibs.apk
  WORKING_DIRECTORY
  "${CMAKE_CURRENT_BINARY_DIR}"
  COMMAND zip -D0ru 
  ${CMAKE_CURRENT_BINARY_DIR}/android_ros-nolibs.apk
  --output-file ${CMAKE_CURRENT_BINARY_DIR}/android_ros-unaligned-unsigned.apk
  lib/
  )

#############################
# Align to 4KiB page boundary
add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/android_ros-unsigned.apk
  COMMENT "Aligning make android_ros-unaligned-unsigned.apk to produce android_ros-unsigned.apk"
  DEPENDS
  ${CMAKE_CURRENT_BINARY_DIR}/android_ros-unaligned-unsigned.apk
  COMMAND ${android_zipalign} -p -f -v 4
  ${CMAKE_CURRENT_BINARY_DIR}/android_ros-unaligned-unsigned.apk
  ${CMAKE_CURRENT_BINARY_DIR}/android_ros-unsigned.apk
  )

# Sign the APK
if(NOT DEFINED $ENV{ANDROID_KEY_PASS})
  set(ENV{ANDROID_KEY_PASS} android)
endif()
if(NOT DEFINED $ENV{ANDROID_KEYSTORE_PASS})
  set(ENV{ANDROID_KEYSTORE_PASS} android)
endif()

add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/android_ros.apk
  COMMENT "Signing android_ros-unsigned.apk to make android_ros.apk"
  DEPENDS
  ${CMAKE_CURRENT_BINARY_DIR}/android_ros-unsigned.apk
  COMMAND "${CMAKE_COMMAND}" -E env
  "ANDROID_KEY_PASS=$ENV{ANDROID_KEY_PASS}"
  "ANDROID_KEYSTORE_PASS=$ENV{ANDROID_KEYSTORE_PASS}"
  "${android_apksigner}" sign
  --ks "${PATH_TO_KEYSTORE}"
  --ks-pass env:ANDROID_KEYSTORE_PASS
  --key-pass env:ANDROID_KEY_PASS
  --out "${CMAKE_CURRENT_BINARY_DIR}/android_ros.apk"
  "${CMAKE_CURRENT_BINARY_DIR}/android_ros-unsigned.apk"
  )

add_custom_target(apk ALL
  DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/android_ros.apk)
