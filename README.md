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
