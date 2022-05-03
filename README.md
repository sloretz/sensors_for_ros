# ROS on Android

Build ROS on android using a CMake superbuild and native activities - no Java or Kotlin.

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
