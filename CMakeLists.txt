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
  CMAKE_COMMAND ${cmake_with_env} -DCMAKE_BUILD_TYPE=RELEASE  # TODO(sloretz) better release/debug switching
  BUILD_COMMAND ${cmake_with_env} --build .
  SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/hello_android"
  DEPENDS
  deps-rclcpp
  deps-rclcpp_lifecycle
  deps-std_msgs
  deps-sensor_msgs
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
set(LIB_NAME "android-ros")
configure_file(
  AndroidManifest.xml.in
  AndroidManifest.xml
  @ONLY)

# Make a place for compiled resources to go
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/compiled_resources)
# Make a list of compiled resources for copying
set(aapt2_compiled_resources)
macro(aapt2_compile input output)
add_custom_command(
  OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/compiled_resources/${output}"
  COMMENT "compiling ${input} -> ${output}"
  COMMAND ${android_aapt2} compile
  "${CMAKE_CURRENT_SOURCE_DIR}/${input}"
  -o "${CMAKE_CURRENT_BINARY_DIR}/compiled_resources"
  )
  list(APPEND aapt2_compiled_resources "${CMAKE_CURRENT_BINARY_DIR}/compiled_resources/${output}")
endmacro()

# Compile resources
aapt2_compile(res/values/strings.xml values_strings.arsc.flat)
aapt2_compile(res/mipmap-hdpi/ic_launcher.png mipmap-hdpi_ic_launcher.png.flat)
aapt2_compile(res/mipmap-mdpi/ic_launcher.png mipmap-mdpi_ic_launcher.png.flat)
aapt2_compile(res/mipmap-xhdpi/ic_launcher.png mipmap-xhdpi_ic_launcher.png.flat)
aapt2_compile(res/mipmap-xxhdpi/ic_launcher.png mipmap-xxhdpi_ic_launcher.png.flat)

# Link resources into an APK that doesn't have the libs yet
add_custom_command(
  OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/android_ros-nolibs.zip"
  COMMENT "linking android_ros-nolibs.zip"
  DEPENDS
  ${aapt2_compiled_resources}
  COMMAND ${android_aapt2} link
  -o "${CMAKE_CURRENT_BINARY_DIR}/android_ros-nolibs.zip"
  -I "${android_jar}"
  --manifest "${CMAKE_CURRENT_BINARY_DIR}/AndroidManifest.xml"
  -R
  ${aapt2_compiled_resources}
  --auto-add-overlay
  )

##############################
# Add compiled libs to the APK

file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib/${ANDROID_ABI})

# Copy cross-compiled libraries to a folder to include in the zip
set(copied_android_libs)
macro(copy_android_lib from_target lib_path)
  if(NOT TARGET ${from_target})
    message(FATAL_ERROR "${from_target} isn't a target. There must be a typo")
  endif()

  get_filename_component(libname "${lib_path}" NAME)
  add_custom_command(
    OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/lib/${ANDROID_ABI}/${libname}"
    COMMENT "Copying ${lib_path} to lib/${ANDROID_ABI}/${libname}"
    DEPENDS
    "${from_target}"
    "${lib_path}"
    COMMAND
    ${CMAKE_COMMAND} -E copy_if_different
    "${lib_path}"
    "${CMAKE_CURRENT_BINARY_DIR}/lib/${ANDROID_ABI}/"
  )
  list(APPEND copied_android_libs "${CMAKE_CURRENT_BINARY_DIR}/lib/${ANDROID_ABI}/${libname}")
endmacro()

copy_android_lib(hello_android "hello_android/lib/libandroid-ros.so")
copy_android_lib(deps-actionlib_msgs "deps/lib/libactionlib_msgs__rosidl_generator_c.so")
copy_android_lib(deps-actionlib_msgs "deps/lib/libactionlib_msgs__rosidl_typesupport_cpp.so")
copy_android_lib(deps-actionlib_msgs "deps/lib/libactionlib_msgs__rosidl_typesupport_c.so")
copy_android_lib(deps-actionlib_msgs "deps/lib/libactionlib_msgs__rosidl_typesupport_introspection_cpp.so")
copy_android_lib(deps-actionlib_msgs "deps/lib/libactionlib_msgs__rosidl_typesupport_introspection_c.so")
copy_android_lib(deps-ament_index_cpp "deps/lib/libament_index_cpp.so")
copy_android_lib(deps-builtin_interfaces "deps/lib/libbuiltin_interfaces__rosidl_generator_c.so")
copy_android_lib(deps-builtin_interfaces "deps/lib/libbuiltin_interfaces__rosidl_typesupport_cpp.so")
copy_android_lib(deps-builtin_interfaces "deps/lib/libbuiltin_interfaces__rosidl_typesupport_c.so")
copy_android_lib(deps-builtin_interfaces "deps/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.so")
copy_android_lib(deps-builtin_interfaces "deps/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_c.so")
copy_android_lib(deps-composition_interfaces "deps/lib/libcomposition_interfaces__rosidl_generator_c.so")
copy_android_lib(deps-composition_interfaces "deps/lib/libcomposition_interfaces__rosidl_typesupport_cpp.so")
copy_android_lib(deps-composition_interfaces "deps/lib/libcomposition_interfaces__rosidl_typesupport_c.so")
copy_android_lib(deps-composition_interfaces "deps/lib/libcomposition_interfaces__rosidl_typesupport_introspection_cpp.so")
copy_android_lib(deps-composition_interfaces "deps/lib/libcomposition_interfaces__rosidl_typesupport_introspection_c.so")
copy_android_lib(deps-cyclonedds "deps/lib/libddsc.so")
copy_android_lib(deps-diagnostic_msgs "deps/lib/libdiagnostic_msgs__rosidl_generator_c.so")
copy_android_lib(deps-diagnostic_msgs "deps/lib/libdiagnostic_msgs__rosidl_typesupport_cpp.so")
copy_android_lib(deps-diagnostic_msgs "deps/lib/libdiagnostic_msgs__rosidl_typesupport_c.so")
copy_android_lib(deps-diagnostic_msgs "deps/lib/libdiagnostic_msgs__rosidl_typesupport_introspection_cpp.so")
copy_android_lib(deps-diagnostic_msgs "deps/lib/libdiagnostic_msgs__rosidl_typesupport_introspection_c.so")
copy_android_lib(deps-geometry_msgs "deps/lib/libgeometry_msgs__rosidl_generator_c.so")
copy_android_lib(deps-geometry_msgs "deps/lib/libgeometry_msgs__rosidl_typesupport_cpp.so")
copy_android_lib(deps-geometry_msgs "deps/lib/libgeometry_msgs__rosidl_typesupport_c.so")
copy_android_lib(deps-geometry_msgs "deps/lib/libgeometry_msgs__rosidl_typesupport_introspection_cpp.so")
copy_android_lib(deps-geometry_msgs "deps/lib/libgeometry_msgs__rosidl_typesupport_introspection_c.so")
copy_android_lib(deps-libstatistics_collector "deps/lib/liblibstatistics_collector.so")
copy_android_lib(deps-lifecycle_msgs "deps/lib/liblifecycle_msgs__rosidl_generator_c.so")
copy_android_lib(deps-lifecycle_msgs "deps/lib/liblifecycle_msgs__rosidl_typesupport_cpp.so")
copy_android_lib(deps-lifecycle_msgs "deps/lib/liblifecycle_msgs__rosidl_typesupport_c.so")
copy_android_lib(deps-lifecycle_msgs "deps/lib/liblifecycle_msgs__rosidl_typesupport_introspection_cpp.so")
copy_android_lib(deps-lifecycle_msgs "deps/lib/liblifecycle_msgs__rosidl_typesupport_introspection_c.so")
copy_android_lib(deps-nav_msgs "deps/lib/libnav_msgs__rosidl_generator_c.so")
copy_android_lib(deps-nav_msgs "deps/lib/libnav_msgs__rosidl_typesupport_cpp.so")
copy_android_lib(deps-nav_msgs "deps/lib/libnav_msgs__rosidl_typesupport_c.so")
copy_android_lib(deps-nav_msgs "deps/lib/libnav_msgs__rosidl_typesupport_introspection_cpp.so")
copy_android_lib(deps-nav_msgs "deps/lib/libnav_msgs__rosidl_typesupport_introspection_c.so")
copy_android_lib(deps-rclcpp_lifecycle "deps/lib/librclcpp_lifecycle.so")
copy_android_lib(deps-rclcpp "deps/lib/librclcpp.so")
copy_android_lib(deps-rcl_interfaces "deps/lib/librcl_interfaces__rosidl_generator_c.so")
copy_android_lib(deps-rcl_interfaces "deps/lib/librcl_interfaces__rosidl_typesupport_cpp.so")
copy_android_lib(deps-rcl_interfaces "deps/lib/librcl_interfaces__rosidl_typesupport_c.so")
copy_android_lib(deps-rcl_interfaces "deps/lib/librcl_interfaces__rosidl_typesupport_introspection_cpp.so")
copy_android_lib(deps-rcl_interfaces "deps/lib/librcl_interfaces__rosidl_typesupport_introspection_c.so")
copy_android_lib(deps-rcl_lifecycle "deps/lib/librcl_lifecycle.so")
copy_android_lib(deps-rcl_logging_interface "deps/lib/librcl_logging_interface.so")
copy_android_lib(deps-rcl_logging_android "deps/lib/librcl_logging_android.so")
copy_android_lib(deps-rcl "deps/lib/librcl.so")
copy_android_lib(deps-rcl_yaml_param_parser "deps/lib/librcl_yaml_param_parser.so")
copy_android_lib(deps-rcpputils "deps/lib/librcpputils.so")
copy_android_lib(deps-rcutils "deps/lib/librcutils.so")
copy_android_lib(deps-rmw_cyclonedds_cpp "deps/lib/librmw_cyclonedds_cpp.so")
copy_android_lib(deps-rmw_dds_common "deps/lib/librmw_dds_common__rosidl_generator_c.so")
copy_android_lib(deps-rmw_dds_common "deps/lib/librmw_dds_common__rosidl_typesupport_cpp.so")
copy_android_lib(deps-rmw_dds_common "deps/lib/librmw_dds_common__rosidl_typesupport_c.so")
copy_android_lib(deps-rmw_dds_common "deps/lib/librmw_dds_common__rosidl_typesupport_introspection_cpp.so")
copy_android_lib(deps-rmw_dds_common "deps/lib/librmw_dds_common__rosidl_typesupport_introspection_c.so")
copy_android_lib(deps-rmw_dds_common "deps/lib/librmw_dds_common.so")
copy_android_lib(deps-rmw "deps/lib/librmw.so")
copy_android_lib(deps-rosgraph_msgs "deps/lib/librosgraph_msgs__rosidl_generator_c.so")
copy_android_lib(deps-rosgraph_msgs "deps/lib/librosgraph_msgs__rosidl_typesupport_cpp.so")
copy_android_lib(deps-rosgraph_msgs "deps/lib/librosgraph_msgs__rosidl_typesupport_c.so")
copy_android_lib(deps-rosgraph_msgs "deps/lib/librosgraph_msgs__rosidl_typesupport_introspection_cpp.so")
copy_android_lib(deps-rosgraph_msgs "deps/lib/librosgraph_msgs__rosidl_typesupport_introspection_c.so")
copy_android_lib(deps-rosidl_runtime_c "deps/lib/librosidl_runtime_c.so")
copy_android_lib(deps-rosidl_typesupport_cpp "deps/lib/librosidl_typesupport_cpp.so")
copy_android_lib(deps-rosidl_typesupport_c "deps/lib/librosidl_typesupport_c.so")
copy_android_lib(deps-rosidl_typesupport_introspection_cpp "deps/lib/librosidl_typesupport_introspection_cpp.so")
copy_android_lib(deps-rosidl_typesupport_introspection_c "deps/lib/librosidl_typesupport_introspection_c.so")
copy_android_lib(deps-sensor_msgs "deps/lib/libsensor_msgs__rosidl_generator_c.so")
copy_android_lib(deps-sensor_msgs "deps/lib/libsensor_msgs__rosidl_typesupport_cpp.so")
copy_android_lib(deps-sensor_msgs "deps/lib/libsensor_msgs__rosidl_typesupport_c.so")
copy_android_lib(deps-sensor_msgs "deps/lib/libsensor_msgs__rosidl_typesupport_introspection_cpp.so")
copy_android_lib(deps-sensor_msgs "deps/lib/libsensor_msgs__rosidl_typesupport_introspection_c.so")
copy_android_lib(deps-shape_msgs "deps/lib/libshape_msgs__rosidl_generator_c.so")
copy_android_lib(deps-shape_msgs "deps/lib/libshape_msgs__rosidl_typesupport_cpp.so")
copy_android_lib(deps-shape_msgs "deps/lib/libshape_msgs__rosidl_typesupport_c.so")
copy_android_lib(deps-shape_msgs "deps/lib/libshape_msgs__rosidl_typesupport_introspection_cpp.so")
copy_android_lib(deps-shape_msgs "deps/lib/libshape_msgs__rosidl_typesupport_introspection_c.so")
copy_android_lib(deps-statistics_msgs "deps/lib/libstatistics_msgs__rosidl_generator_c.so")
copy_android_lib(deps-statistics_msgs "deps/lib/libstatistics_msgs__rosidl_typesupport_cpp.so")
copy_android_lib(deps-statistics_msgs "deps/lib/libstatistics_msgs__rosidl_typesupport_c.so")
copy_android_lib(deps-statistics_msgs "deps/lib/libstatistics_msgs__rosidl_typesupport_introspection_cpp.so")
copy_android_lib(deps-statistics_msgs "deps/lib/libstatistics_msgs__rosidl_typesupport_introspection_c.so")
copy_android_lib(deps-std_msgs "deps/lib/libstd_msgs__rosidl_generator_c.so")
copy_android_lib(deps-std_msgs "deps/lib/libstd_msgs__rosidl_typesupport_cpp.so")
copy_android_lib(deps-std_msgs "deps/lib/libstd_msgs__rosidl_typesupport_c.so")
copy_android_lib(deps-std_msgs "deps/lib/libstd_msgs__rosidl_typesupport_introspection_cpp.so")
copy_android_lib(deps-std_msgs "deps/lib/libstd_msgs__rosidl_typesupport_introspection_c.so")
copy_android_lib(deps-std_srvs "deps/lib/libstd_srvs__rosidl_generator_c.so")
copy_android_lib(deps-std_srvs "deps/lib/libstd_srvs__rosidl_typesupport_cpp.so")
copy_android_lib(deps-std_srvs "deps/lib/libstd_srvs__rosidl_typesupport_c.so")
copy_android_lib(deps-std_srvs "deps/lib/libstd_srvs__rosidl_typesupport_introspection_cpp.so")
copy_android_lib(deps-std_srvs "deps/lib/libstd_srvs__rosidl_typesupport_introspection_c.so")
copy_android_lib(deps-stereo_msgs "deps/lib/libstereo_msgs__rosidl_generator_c.so")
copy_android_lib(deps-stereo_msgs "deps/lib/libstereo_msgs__rosidl_typesupport_cpp.so")
copy_android_lib(deps-stereo_msgs "deps/lib/libstereo_msgs__rosidl_typesupport_c.so")
copy_android_lib(deps-stereo_msgs "deps/lib/libstereo_msgs__rosidl_typesupport_introspection_cpp.so")
copy_android_lib(deps-stereo_msgs "deps/lib/libstereo_msgs__rosidl_typesupport_introspection_c.so")
copy_android_lib(deps-tracetools "deps/lib/libtracetools.so")
copy_android_lib(deps-tracetools  "deps/lib/libtracetools_status.so")
copy_android_lib(deps-trajectory_msgs "deps/lib/libtrajectory_msgs__rosidl_generator_c.so")
copy_android_lib(deps-trajectory_msgs "deps/lib/libtrajectory_msgs__rosidl_typesupport_cpp.so")
copy_android_lib(deps-trajectory_msgs "deps/lib/libtrajectory_msgs__rosidl_typesupport_c.so")
copy_android_lib(deps-trajectory_msgs "deps/lib/libtrajectory_msgs__rosidl_typesupport_introspection_cpp.so")
copy_android_lib(deps-trajectory_msgs "deps/lib/libtrajectory_msgs__rosidl_typesupport_introspection_c.so")
copy_android_lib(deps-visualization_msgs "deps/lib/libvisualization_msgs__rosidl_generator_c.so")
copy_android_lib(deps-visualization_msgs "deps/lib/libvisualization_msgs__rosidl_typesupport_cpp.so")
copy_android_lib(deps-visualization_msgs "deps/lib/libvisualization_msgs__rosidl_typesupport_c.so")
copy_android_lib(deps-visualization_msgs "deps/lib/libvisualization_msgs__rosidl_typesupport_introspection_cpp.so")
copy_android_lib(deps-visualization_msgs "deps/lib/libvisualization_msgs__rosidl_typesupport_introspection_c.so")
copy_android_lib(deps-libyaml_vendor "deps/lib/libyaml.so")

# Add all shared libraries to APK
add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/android_ros-unaligned-unsigned.zip
  COMMENT "Adding libs to android_ros-nolibs.zip to make android_ros-unaligned-unsigned.zip"
  DEPENDS
  ${copied_android_libs}
  ${CMAKE_CURRENT_BINARY_DIR}/android_ros-nolibs.zip
  WORKING_DIRECTORY
  "${CMAKE_CURRENT_BINARY_DIR}"
  COMMAND zip -D0ru 
  ${CMAKE_CURRENT_BINARY_DIR}/android_ros-nolibs.zip
  --output-file ${CMAKE_CURRENT_BINARY_DIR}/android_ros-unaligned-unsigned.zip
  lib/
  )

#############################
# Align to 4KiB page boundary
add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/android_ros-unsigned.zip
  COMMENT "Aligning make android_ros-unaligned-unsigned.zip to produce android_ros-unsigned.zip"
  DEPENDS
  ${CMAKE_CURRENT_BINARY_DIR}/android_ros-unaligned-unsigned.zip
  COMMAND ${android_zipalign} -p -f -v 4
  ${CMAKE_CURRENT_BINARY_DIR}/android_ros-unaligned-unsigned.zip
  ${CMAKE_CURRENT_BINARY_DIR}/android_ros-unsigned.zip
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
  COMMENT "Signing android_ros-unsigned.zip to make android_ros.apk"
  DEPENDS
  ${CMAKE_CURRENT_BINARY_DIR}/android_ros-unsigned.zip
  COMMAND "${CMAKE_COMMAND}" -E env
  "ANDROID_KEY_PASS=$ENV{ANDROID_KEY_PASS}"
  "ANDROID_KEYSTORE_PASS=$ENV{ANDROID_KEYSTORE_PASS}"
  "${android_apksigner}" sign
  --ks "${PATH_TO_KEYSTORE}"
  --ks-pass env:ANDROID_KEYSTORE_PASS
  --key-pass env:ANDROID_KEY_PASS
  --out "${CMAKE_CURRENT_BINARY_DIR}/android_ros.apk"
  "${CMAKE_CURRENT_BINARY_DIR}/android_ros-unsigned.zip"
  )

add_custom_target(apk ALL
  DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/android_ros.apk)
