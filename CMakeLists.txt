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

set(android_cmake_args 
  -DCMAKE_TOOLCHAIN_FILE=${android_toolchain}
  -DANDROID_ABI=arm64-v8a
  -DANDROID_PLATFORM=android-${minimum_target_sdk}
)

# Make place to install Python packages we depend on
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/deps/_python_)

# A list of CMAKE args for find_packaging the location of each dep
set(deps_cmake_DIR_args)

ExternalProject_Add(deps-ament_package
  DOWNLOAD_COMMAND ""
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  TEST_COMMAND ""
  SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/deps/ament_package"
  INSTALL_COMMAND
    pip install
      -t "${CMAKE_CURRENT_BINARY_DIR}/deps/_python_"
      --no-deps
      "${CMAKE_CURRENT_SOURCE_DIR}/deps/ament_package"
)
ExternalProject_Add_StepTargets(deps-ament_package "install")

ExternalProject_Add(deps-ament_cmake_core
  DOWNLOAD_COMMAND ""
  CMAKE_COMMAND "${CMAKE_COMMAND}" -E env PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/deps/_python_ "${CMAKE_COMMAND}"
  DEPENDS
    deps-ament_package-install
  SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/deps/ament_cmake/ament_cmake_core"
  CMAKE_ARGS
    ${android_cmake_args}
    -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/deps/ament_cmake_core
    -DBUILD_TESTING=OFF
)
ExternalProject_Add_StepTargets(deps-ament_cmake_core "install")
list(APPEND deps_cmake_DIR_args "-Dament_cmake_core_DIR=${CMAKE_CURRENT_BINARY_DIR}/deps/ament_cmake_core/share/ament_cmake_core/cmake")

ExternalProject_Add(deps-ament_cmake_export_libraries
  DOWNLOAD_COMMAND ""
  CMAKE_COMMAND "${CMAKE_COMMAND}" -E
    env PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/deps/_python_
    "${CMAKE_COMMAND}"
  DEPENDS
    deps-ament_cmake_core-install
  SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/deps/ament_cmake/ament_cmake_export_libraries"
  CMAKE_ARGS
    ${android_cmake_args}
    ${deps_cmake_DIR_args}
    -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/deps/ament_cmake_export_libraries
    -DBUILD_TESTING=OFF
)
ExternalProject_Add_StepTargets(deps-ament_cmake_export_libraries "install")
list(APPEND deps_cmake_DIR_args "-Dament_cmake_export_libraries_DIR=${CMAKE_CURRENT_BINARY_DIR}/deps/ament_cmake_export_libraries/share/ament_cmake_export_libraries/cmake")

ExternalProject_Add(deps-ament_cmake_export_definitions
  DOWNLOAD_COMMAND ""
  CMAKE_COMMAND "${CMAKE_COMMAND}" -E
    env PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/deps/_python_
    "${CMAKE_COMMAND}"
  DEPENDS
    deps-ament_cmake_core-install
  SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/deps/ament_cmake/ament_cmake_export_definitions"
  CMAKE_ARGS
    ${android_cmake_args}
    -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/deps/ament_cmake_export_definitions
    ${deps_cmake_DIR_args}
    -DBUILD_TESTING=OFF
)
ExternalProject_Add_StepTargets(deps-ament_cmake_export_definitions "install")
list(APPEND deps_cmake_DIR_args "-Dament_cmake_export_definitions_DIR=${CMAKE_CURRENT_BINARY_DIR}/deps/ament_cmake_export_definitions/share/ament_cmake_export_definitions/cmake")

ExternalProject_Add(deps-ament_cmake_export_include_directories
  DOWNLOAD_COMMAND ""
  CMAKE_COMMAND "${CMAKE_COMMAND}" -E
    env PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/deps/_python_
    "${CMAKE_COMMAND}"
  DEPENDS
    deps-ament_cmake_core-install
  SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/deps/ament_cmake/ament_cmake_export_include_directories"
  CMAKE_ARGS
    ${android_cmake_args}
    -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/deps/ament_cmake_export_include_directories
    ${deps_cmake_DIR_args}
    -DBUILD_TESTING=OFF
)
ExternalProject_Add_StepTargets(deps-ament_cmake_export_include_directories "install")
list(APPEND deps_cmake_DIR_args "-Dament_cmake_export_include_directories_DIR=${CMAKE_CURRENT_BINARY_DIR}/deps/ament_cmake_export_include_directories/share/ament_cmake_export_include_directories/cmake")

ExternalProject_Add(deps-ament_cmake_export_link_flags
  DOWNLOAD_COMMAND ""
  CMAKE_COMMAND "${CMAKE_COMMAND}" -E
    env PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/deps/_python_
    "${CMAKE_COMMAND}"
  DEPENDS
    deps-ament_cmake_core-install
  SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/deps/ament_cmake/ament_cmake_export_link_flags"
  CMAKE_ARGS
    ${android_cmake_args}
    -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/deps/ament_cmake_export_link_flags
    ${deps_cmake_DIR_args}
    -DBUILD_TESTING=OFF
)
ExternalProject_Add_StepTargets(deps-ament_cmake_export_link_flags "install")
list(APPEND deps_cmake_DIR_args "-Dament_cmake_export_link_flags_DIR=${CMAKE_CURRENT_BINARY_DIR}/deps/ament_cmake_export_link_flags/share/ament_cmake_export_link_flags/cmake")

ExternalProject_Add(deps-ament_cmake_include_directories
  DOWNLOAD_COMMAND ""
  CMAKE_COMMAND "${CMAKE_COMMAND}" -E
    env PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/deps/_python_
    "${CMAKE_COMMAND}"
  DEPENDS
    deps-ament_cmake_core-install
  SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/deps/ament_cmake/ament_cmake_include_directories"
  CMAKE_ARGS
    ${android_cmake_args}
    -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/deps/ament_cmake_include_directories
    ${deps_cmake_DIR_args}
    -DBUILD_TESTING=OFF
)
ExternalProject_Add_StepTargets(deps-ament_cmake_include_directories "install")
list(APPEND deps_cmake_DIR_args "-Dament_cmake_include_directories_DIR=${CMAKE_CURRENT_BINARY_DIR}/deps/ament_cmake_include_directories/share/ament_cmake_include_directories/cmake")

ExternalProject_Add(deps-ament_cmake_libraries
  DOWNLOAD_COMMAND ""
  CMAKE_COMMAND "${CMAKE_COMMAND}" -E
    env PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/deps/_python_
    "${CMAKE_COMMAND}"
  DEPENDS
    deps-ament_cmake_core-install
  SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/deps/ament_cmake/ament_cmake_libraries"
  CMAKE_ARGS
    ${android_cmake_args}
    -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/deps/ament_cmake_libraries
    ${deps_cmake_DIR_args}
    -DBUILD_TESTING=OFF
)
ExternalProject_Add_StepTargets(deps-ament_cmake_libraries "install")
list(APPEND deps_cmake_DIR_args "-Dament_cmake_libraries_DIR=${CMAKE_CURRENT_BINARY_DIR}/deps/ament_cmake_libraries/share/ament_cmake_libraries/cmake")

ExternalProject_Add(deps-ament_cmake_version
  DOWNLOAD_COMMAND ""
  CMAKE_COMMAND "${CMAKE_COMMAND}" -E
    env PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/deps/_python_
    "${CMAKE_COMMAND}"
  DEPENDS
    deps-ament_cmake_core-install
  SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/deps/ament_cmake/ament_cmake_version"
  CMAKE_ARGS
    ${android_cmake_args}
    -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/deps/ament_cmake_version
    ${deps_cmake_DIR_args}
    -DBUILD_TESTING=OFF
)
ExternalProject_Add_StepTargets(deps-ament_cmake_version "install")
list(APPEND deps_cmake_DIR_args "-Dament_cmake_version_DIR=${CMAKE_CURRENT_BINARY_DIR}/deps/ament_cmake_version/share/ament_cmake_version/cmake")

ExternalProject_Add(deps-ament_cmake_python
  DOWNLOAD_COMMAND ""
  CMAKE_COMMAND "${CMAKE_COMMAND}" -E
    env PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/deps/_python_
    "${CMAKE_COMMAND}"
  DEPENDS
    deps-ament_cmake_core-install
  SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/deps/ament_cmake/ament_cmake_python"
  CMAKE_ARGS
    ${android_cmake_args}
    -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/deps/ament_cmake_python
    ${deps_cmake_DIR_args}
    -DBUILD_TESTING=OFF
)
ExternalProject_Add_StepTargets(deps-ament_cmake_python "install")
list(APPEND deps_cmake_DIR_args "-Dament_cmake_python_DIR=${CMAKE_CURRENT_BINARY_DIR}/deps/ament_cmake_python/share/ament_cmake_python/cmake")

ExternalProject_Add(deps-ament_cmake_gen_version_h
  DOWNLOAD_COMMAND ""
  CMAKE_COMMAND "${CMAKE_COMMAND}" -E
    env PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/deps/_python_
    "${CMAKE_COMMAND}"
  DEPENDS
    deps-ament_cmake_core-install
  SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/deps/ament_cmake/ament_cmake_gen_version_h"
  CMAKE_ARGS
    ${android_cmake_args}
    -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/deps/ament_cmake_gen_version_h
    ${deps_cmake_DIR_args}
    -DBUILD_TESTING=OFF
)
ExternalProject_Add_StepTargets(deps-ament_cmake_gen_version_h "install")
list(APPEND deps_cmake_DIR_args "-Dament_cmake_gen_version_h_DIR=${CMAKE_CURRENT_BINARY_DIR}/deps/ament_cmake_gen_version_h/share/ament_cmake_gen_version_h/cmake")

ExternalProject_Add(deps-ament_cmake_export_targets
  DOWNLOAD_COMMAND ""
  CMAKE_COMMAND "${CMAKE_COMMAND}" -E
    env PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/deps/_python_
    "${CMAKE_COMMAND}"
  DEPENDS
    deps-ament_cmake_core-install
    deps-ament_cmake_export_libraries-install
  SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/deps/ament_cmake/ament_cmake_export_targets"
  CMAKE_ARGS
    ${android_cmake_args}
    -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/deps/ament_cmake_export_targets
    ${deps_cmake_DIR_args}
    -DBUILD_TESTING=OFF
)
ExternalProject_Add_StepTargets(deps-ament_cmake_export_targets "install")
list(APPEND deps_cmake_DIR_args "-Dament_cmake_export_targets_DIR=${CMAKE_CURRENT_BINARY_DIR}/deps/ament_cmake_export_targets/share/ament_cmake_export_targets/cmake")

ExternalProject_Add(deps-ament_cmake_export_interfaces
  DOWNLOAD_COMMAND ""
  CMAKE_COMMAND "${CMAKE_COMMAND}" -E
    env PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/deps/_python_
    "${CMAKE_COMMAND}"
  DEPENDS
    deps-ament_cmake_core-install
    deps-ament_cmake_export_libraries-install
  SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/deps/ament_cmake/ament_cmake_export_interfaces"
  CMAKE_ARGS
    ${android_cmake_args}
    -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/deps/ament_cmake_export_interfaces
    ${deps_cmake_DIR_args}
    -DBUILD_TESTING=OFF
)
ExternalProject_Add_StepTargets(deps-ament_cmake_export_interfaces "install")
list(APPEND deps_cmake_DIR_args "-Dament_cmake_export_interfaces_DIR=${CMAKE_CURRENT_BINARY_DIR}/deps/ament_cmake_export_interfaces/share/ament_cmake_export_interfaces/cmake")

ExternalProject_Add(deps-ament_cmake_target_dependencies
  DOWNLOAD_COMMAND ""
  CMAKE_COMMAND "${CMAKE_COMMAND}" -E
    env PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/deps/_python_
    "${CMAKE_COMMAND}"
  DEPENDS
    deps-ament_cmake_core-install
    deps-ament_cmake_include_directories-install
  SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/deps/ament_cmake/ament_cmake_target_dependencies"
  CMAKE_ARGS
    ${android_cmake_args}
    -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/deps/ament_cmake_target_dependencies
    ${deps_cmake_DIR_args}
    -DBUILD_TESTING=OFF
)
ExternalProject_Add_StepTargets(deps-ament_cmake_target_dependencies "install")
list(APPEND deps_cmake_DIR_args "-Dament_cmake_target_dependencies_DIR=${CMAKE_CURRENT_BINARY_DIR}/deps/ament_cmake_target_dependencies/share/ament_cmake_target_dependencies/cmake")

ExternalProject_Add(deps-ament_cmake_export_dependencies
  DOWNLOAD_COMMAND ""
  CMAKE_COMMAND "${CMAKE_COMMAND}" -E
    env PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/deps/_python_
    "${CMAKE_COMMAND}"
  DEPENDS
    deps-ament_cmake_core-install
    deps-ament_cmake_libraries-install
  SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/deps/ament_cmake/ament_cmake_export_dependencies"
  CMAKE_ARGS
    ${android_cmake_args}
    -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/deps/ament_cmake_export_dependencies
    ${deps_cmake_DIR_args}
    -DBUILD_TESTING=OFF
)
ExternalProject_Add_StepTargets(deps-ament_cmake_export_dependencies "install")
list(APPEND deps_cmake_DIR_args "-Dament_cmake_export_dependencies_DIR=${CMAKE_CURRENT_BINARY_DIR}/deps/ament_cmake_export_dependencies/share/ament_cmake_export_dependencies/cmake")

ExternalProject_Add(deps-ament_cmake_test
  DOWNLOAD_COMMAND ""
  CMAKE_COMMAND "${CMAKE_COMMAND}" -E
    env PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/deps/_python_
    "${CMAKE_COMMAND}"
  DEPENDS
    deps-ament_cmake_core-install
    deps-ament_cmake_python-install
  SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/deps/ament_cmake/ament_cmake_test"
  CMAKE_ARGS
    ${android_cmake_args}
    -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/deps/ament_cmake_test
    ${deps_cmake_DIR_args}
    -DBUILD_TESTING=OFF
)
ExternalProject_Add_StepTargets(deps-ament_cmake_test "install")
list(APPEND deps_cmake_DIR_args "-Dament_cmake_test_DIR=${CMAKE_CURRENT_BINARY_DIR}/deps/ament_cmake_test/share/ament_cmake_test/cmake")

ExternalProject_Add(deps-ament_cmake
  DOWNLOAD_COMMAND ""
  CMAKE_COMMAND "${CMAKE_COMMAND}" -E
    env PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/deps/_python_
    "${CMAKE_COMMAND}"
  DEPENDS
    deps-ament_package-install
    deps-ament_cmake_core-install
    deps-ament_cmake_export_definitions-install
    deps-ament_cmake_export_include_directories-install
    deps-ament_cmake_export_libraries-install
    deps-ament_cmake_export_link_flags-install
    deps-ament_cmake_include_directories-install
    deps-ament_cmake_libraries-install
    deps-ament_cmake_version-install
    deps-ament_cmake_python-install
    deps-ament_cmake_gen_version_h-install
    deps-ament_cmake_export_targets-install
    deps-ament_cmake_export_interfaces-install
    deps-ament_cmake_target_dependencies-install
    deps-ament_cmake_export_dependencies-install
    deps-ament_cmake_test-install
  SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/deps/ament_cmake/ament_cmake"
  CMAKE_ARGS
    ${android_cmake_args}
    -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/deps/ament_cmake
    ${deps_cmake_DIR_args}
    -DBUILD_TESTING=OFF
)
ExternalProject_Add_StepTargets(deps-ament_cmake "install")
list(APPEND deps_cmake_DIR_args "-Dament_cmake_DIR=${CMAKE_CURRENT_BINARY_DIR}/deps/ament_cmake/share/ament_cmake/cmake")

ExternalProject_Add(deps-ament_cmake_ros
  DOWNLOAD_COMMAND ""
  CMAKE_COMMAND "${CMAKE_COMMAND}" -E
    env PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/deps/_python_
    "${CMAKE_COMMAND}"
  DEPENDS
    deps-ament_cmake-install
  SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/deps/ament_cmake_ros/ament_cmake_ros"
  CMAKE_ARGS
    ${android_cmake_args}
    -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/deps/ament_cmake_ros
    ${deps_cmake_DIR_args}
    -DBUILD_TESTING=OFF
)
ExternalProject_Add_StepTargets(deps-ament_cmake_ros "install")



##################################
# Android specific projects
##################################

ExternalProject_Add(hello_android
  DOWNLOAD_COMMAND ""
  SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/hello_android"
  CMAKE_ARGS
    ${android_cmake_args}
    -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/hello_android
)
# Create a hello_anrdoid-install target so we can use it's output to make an APK
ExternalProject_Add_StepTargets(hello_android "install")

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

# Add compiled libs to the APK
add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/android_ros-unaligned-unsigned.apk
  COMMENT "Adding libs to android_ros-nolibs.apk to make android_ros-unaligned-unsigned.apk"
  DEPENDS
    hello_android-install
      ${CMAKE_CURRENT_BINARY_DIR}/android_ros-nolibs.apk
  WORKING_DIRECTORY
    ${CMAKE_CURRENT_BINARY_DIR}/hello_android
  COMMAND zip -D0ru 
    ${CMAKE_CURRENT_BINARY_DIR}/android_ros-nolibs.apk
    --output-file ${CMAKE_CURRENT_BINARY_DIR}/android_ros-unaligned-unsigned.apk
    lib/
)

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
