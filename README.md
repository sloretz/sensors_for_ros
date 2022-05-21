# ROS on Android

Build ROS on android using a CMake superbuild and native activities - no Java or Kotlin.

## Building Installing and running

Build the software

```
mkdir build
cd build
cmake ../ -DANDROID_HOME=/home/sloretz/android-sdk/
make -j`nproc`
```

Install the APK in the build directory onto a device.

```
adb install -r android_ros.apk
```

Use logcat to view the logs from the app
```
adb logcat
```

The main activity can be started directly from the CLI
```
adb shell am start -n loretz.shane/android.app.NativeActivity
```

Getting stack traces

```
adb logcat | ~/android-sdk/ndk/21.3.6528147/ndk-stack -sym lib/arm64-v8a/
```

# Lessons

## Native dependencies
bison

## Can't build iceoryx

Android [intentionally doesn't impelement the necessary APIs](https://stackoverflow.com/a/38637565).

```
[ 74%] Performing build step for 'deps-iceoryx_hoofs'
[  2%] Building CXX object platform/CMakeFiles/iceoryx_platform.dir/unix/source/file.cpp.o
[  4%] Building CXX object platform/CMakeFiles/iceoryx_platform.dir/unix/source/fnctl.cpp.o
[  6%] Building CXX object platform/CMakeFiles/iceoryx_platform.dir/unix/source/grp.cpp.o
[  8%] Building CXX object platform/CMakeFiles/iceoryx_platform.dir/unix/source/mman.cpp.o
/home/sloretz/android_ros/deps/iceoryx/iceoryx_hoofs/platform/unix/source/mman.cpp:22:12: error: use of undeclared identifier 'shm_open'
    return shm_open(name, oflag, mode);
           ^
/home/sloretz/android_ros/deps/iceoryx/iceoryx_hoofs/platform/unix/source/mman.cpp:28:12: error: use of undeclared identifier 'shm_unlink'
    return shm_unlink(name);
```

## Cyclonedds needs native ddsconf tool

In the 0.8.x release it has a tool [`ddsconf` that needs to exist on the native machine](https://github.com/eclipse-cyclonedds/cyclonedds/blob/93fdedcf0c99988147b04c22e13cb24dd2e8f74f/src/tools/ddsconf/CMakeLists.txt#L58-L60).
I think this means two projects: one building the ddsconf executable locally.

## Cyclonedds Native and Cross Compiled need matching ENABLE_SSL settings

They take a CMake argument `-DENABLE_SSL=OFF` to turn off SSL support.
If the native build supports SSL, but the cross compiled does not, then compile erros result because the native ddsconf tool assumes SSL support

```

[ 43%] Building C object src/core/CMakeFiles/ddsc.dir/defconfig.c.o
/home/sloretz/android_ros/build/deps-cyclonedds-prefix/src/deps-cyclonedds-build/src/core/defconfig.c:83:8: error: no member named 'ssl_verify' in 'struct ddsi_config'
  cfg->ssl_verify = INT32_C (1);
  ~~~  ^
/home/sloretz/android_ros/build/deps-cyclonedds-prefix/src/deps-cyclonedds-build/src/core/defconfig.c:84:8: error: no member named 'ssl_verify_client' in 'struct ddsi_config'
  cfg->ssl_verify_client = INT32_C (1);
  ~~~  ^
/home/sloretz/android_ros/build/deps-cyclonedds-prefix/src/deps-cyclonedds-build/src/core/defconfig.c:85:8: error: no member named 'ssl_keystore' in 'struct ddsi_config'
  cfg->ssl_keystore = "keystore";
  ~~~  ^
/home/sloretz/android_ros/build/deps-cyclonedds-prefix/src/deps-cyclonedds-build/src/core/defconfig.c:86:8: error: no member named 'ssl_key_pass' in 'struct ddsi_config'
  cfg->ssl_key_pass = "secret";
  ~~~  ^
/home/sloretz/android_ros/build/deps-cyclonedds-prefix/src/deps-cyclonedds-build/src/core/defconfig.c:87:8: error: no member named 'ssl_ciphers' in 'struct ddsi_config'
  cfg->ssl_ciphers = "ALL:!ADH:!LOW:!EXP:!MD5:@STRENGTH";
  ~~~  ^
/home/sloretz/android_ros/build/deps-cyclonedds-prefix/src/deps-cyclonedds-build/src/core/defconfig.c:88:8: error: no member named 'ssl_rand_file' in 'struct ddsi_config'
  cfg->ssl_rand_file = "";
  ~~~  ^
/home/sloretz/android_ros/build/deps-cyclonedds-prefix/src/deps-cyclonedds-build/src/core/defconfig.c:89:8: error: no member named 'ssl_min_version' in 'struct ddsi_config'
  cfg->ssl_min_version.major = 1;
  ~~~  ^
/home/sloretz/android_ros/build/deps-cyclonedds-prefix/src/deps-cyclonedds-build/src/core/defconfig.c:90:8: error: no member named 'ssl_min_version' in 'struct ddsi_config'
  cfg->ssl_min_version.minor = 3;
  ~~~  ^
```

## catkin_pkg won't give transitive build dependencies

The `catkin_pkg.topological_order.topological_order` function does seem to order by exported dependencies, but the exported package objects don't have that information built in.
I had to make the `generate_ros_superbuild.py` script transitively lookup exported dependencies so`make -jN` on the superbuild would work.

## Vendor package is inconvenient when scripting a superbuild

`rcl_yaml_param_parser` can't find `yamlConfig.cmake` because of the way I've written `dep_build` to install packages to separate folders.
CMake can find a package with `<prefix>/<name>*/(cmake|CMake)/`, but the `<name>` here is `libyaml_vendor` instead of `yaml`.

I worked around it by passing `-Dyaml_DIR` to all downstream packages.

## Typesupports are found using the ament index

```
CMake Error at /home/sloretz/android_ros/build/deps/rosidl_typesupport_c/share/rosidl_typesupport_c/cmake/get_used_typesupports.cmake:35 (message):
  No 'rosidl_typesupport_c' found
Call Stack (most recent call first):
  /home/sloretz/android_ros/build/deps/rosidl_typesupport_c/share/rosidl_typesupport_c/cmake/rosidl_typesupport_c-extras.cmake:8 (get_used_typesupports)
  /home/sloretz/android_ros/build/deps/rosidl_typesupport_c/share/rosidl_typesupport_c/cmake/rosidl_typesupport_cConfig.cmake:41 (include)
  /home/sloretz/android_ros/build/deps/rosidl_default_generators/share/rosidl_default_generators/cmake/rosidl_default_generators-extras.cmake:21 (find_package)
  /home/sloretz/android_ros/build/deps/rosidl_default_generators/share/rosidl_default_generators/cmake/rosidl_default_generatorsConfig.cmake:41 (include)
  CMakeLists.txt:14 (find_package)
```

I'll work around it by setting `AMENT_PREFIX_PATH` in dep-build.
I could also work around this by using a "merged" install space.

## Need PYTHONPATH to be set for CMake projects

I'm installing Python packages with PIP into `deps/_python_`, but CMake packages are installed to `deps/${name}`
How do I update PYTHONPATH for CMake packages that install Python modules?
Ament feature to override python install dir?
It looks like [`PYTHON_INSTALL_DIR`](https://github.com/ament/ament_cmake/blob/b84cf9e6f2a61d8f9fc5a90c02dc2b5cb63e7f76/ament_cmake_python/ament_cmake_python-extras.cmake#L44) will do what I want.

```
CMake Error at /home/sloretz/android_ros/build/deps/rosidl_adapter/share/rosidl_adapter/cmake/rosidl_adapt_interfaces.cmake:59 (message):
  execute_process(/usr/bin/python3.8 -m rosidl_adapter --package-name
  builtin_interfaces --arguments-file
  /home/sloretz/android_ros/build/deps-builtin_interfaces-prefix/src/deps-builtin_interfaces-build/rosidl_adapter__arguments__builtin_interfaces.json
  --output-dir
  /home/sloretz/android_ros/build/deps-builtin_interfaces-prefix/src/deps-builtin_interfaces-build/rosidl_adapter/builtin_interfaces
  --output-file
  /home/sloretz/android_ros/build/deps-builtin_interfaces-prefix/src/deps-builtin_interfaces-build/rosidl_adapter/builtin_interfaces.idls)
  returned error code 1:

  /usr/bin/python3.8: No module named rosidl_adapter

Call Stack (most recent call first):
  /home/sloretz/android_ros/build/deps/rosidl_cmake/share/rosidl_cmake/cmake/rosidl_generate_interfaces.cmake:130 (rosidl_adapt_interfaces)
  CMakeLists.txt:16 (rosidl_generate_interfaces)
```

## rosidl_generator_c expects relative PYTHON_INSTALL_DIR

```
CMake Error at /home/sloretz/android_ros/build/deps/rosidl_generator_c/share/rosidl_generator_c/cmake/rosidl_generator_c_generate_interfaces.cmake:69 (message):
  Target dependency
  '/home/sloretz/android_ros/build/deps/rosidl_generator_c/share/rosidl_generator_c/cmake/../../..//home/sloretz/android_ros/build/deps/_python_/rosidl_generator_c/__init__.py'
  does not exist
Call Stack (most recent call first):
  /home/sloretz/android_ros/build/deps/ament_cmake_core/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake:48 (include)
  /home/sloretz/android_ros/build/deps/rosidl_cmake/share/rosidl_cmake/cmake/rosidl_generate_interfaces.cmake:286 (ament_execute_extensions)
  CMakeLists.txt:16 (rosidl_generate_interfaces)
```

## rosidl generators use add_custom_command, but PYTHONPATH not available to them

I'm not sure how the rosidl generator inputs are supposed to work.
`add_custom_command` runs at build time, so the PYTHONPATH given to the CMake invocation
is not necessarily available to the make invocation.

I'll work around this by making `dep_build` set `PYTHONPATH` in the build command.

Maybe the rosidl generators should pass the ENV at CMake time to the Python process?

```
[  5%] Generating C code for ROS interfaces SLORETZ /home/sloretz/android_ros/build/deps/_python_
[ 11%] Generating C++ code for ROS interfaces
['/home/sloretz/android_ros/build/deps/rosidl_generator_c/lib/rosidl_generator_c', '/usr/lib/python38.zip', '/usr/lib/python3.8', '/usr/lib/python3.8/lib-dynload', '/usr/local/lib/python3.8/dist-packages', '/usr/lib/python3/dist-packages']
['/home/sloretz/android_ros/build/deps/rosidl_generator_c/share/rosidl_generator_c/cmake/../../../lib/rosidl_generator_c/rosidl_generator_c', '--generator-arguments-file', '/home/sloretz/android_ros/build/deps-builtin_interfaces-prefix/src/deps-builtin_interfaces-build/rosidl_generator_c__arguments.json']
Traceback (most recent call last):
  File "/home/sloretz/android_ros/build/deps/rosidl_generator_c/share/rosidl_generator_c/cmake/../../../lib/rosidl_generator_c/rosidl_generator_c", line 11, in <module>
    from rosidl_generator_c import generate_c
ModuleNotFoundError: No module named 'rosidl_generator_c'
CMakeFiles/builtin_interfaces__rosidl_generator_c.dir/build.make:87: recipe for target 'rosidl_generator_c/builtin_interfaces/msg/duration.h' failed
```

## rosidl_typesupport_cpp sneakily depends on rosidl_typesupport_introspection_c

`rosidl_typesupport_c` when found tries to find other typesupports that use it.
If not, it has a fatal error.
The only package in this workspace that uses it is `rosidl_typesupport_introspection_c`.
This means `rosidl_typesupport_cpp` can't `find_package(rosidl_typesupport_c` until after `rosidl_typesupport_introspection_c` is built.

```
CMake Error at /home/sloretz/android_ros/build/deps/rosidl_typesupport_c/share/rosidl_typesupport_c/cmake/get_used_typesupports.cmake:35 (message):
  No 'rosidl_typesupport_c' found
Call Stack (most recent call first):
  /home/sloretz/android_ros/build/deps/rosidl_typesupport_c/share/rosidl_typesupport_c/cmake/rosidl_typesupport_c-extras.cmake:8 (get_used_typesupports)
  /home/sloretz/android_ros/build/deps/rosidl_typesupport_c/share/rosidl_typesupport_c/cmake/rosidl_typesupport_cConfig.cmake:41 (include)
  CMakeLists.txt:20 (find_package)


-- Configuring incomplete, errors occurred!
See also "/home/sloretz/android_ros/build/deps-rosidl_typesupport_cpp-prefix/src/deps-rosidl_typesupport_cpp-build/CMakeFiles/CMakeOutput.log".
```

## Failed to configure logging

Might need an android specific logger.
I'll use the noop logging for now.
It looks like the CMake variable `RCL_LOGGING_IMPLEMENTATION` is the one I need to set.


```
05-07 15:15:43.986  2980  3006 E libc++abi: terminating with uncaught exception of type rclcpp::exceptions::RCLError: failed to configure logging: Failed to get logging directory, at /home/sloretz/android_ros/deps/rcl_logging/rcl_logging_spdlog/src/rcl_logging_spdlog.cpp:83
```

## rmw implementation invalid - can't create node


```
05-07 15:29:28.641  3452  3486 E libc++abi: terminating with uncaught exception of type rclcpp::exceptions::RCLError: failed to initialize rcl node: rcl node's rmw handle is invalid, at /home/sloretz/android_ros/deps/rcl/rcl/src/rcl/node.c:416
```

This was caused by not having the permissions needed to access the network.
Adding those permissions to the manifest fixed it.

```xml
    <uses-permission android:name="android.permission.INTERNET" />
    <uses-permission android:name="android.permission.ACCESS_NETWORK_STATE" />
```
