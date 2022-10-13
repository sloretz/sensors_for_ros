include(dep_build.cmake)

macro(build_native_dependencies)
  # dep_build(cyclonedds CMAKE
  #   NATIVE
  #   SOURCE_DIR "deps/cyclonedds"
  #   CMAKE_ARGS "-DBUILD_DDSCONF=ON"
  #   # TODO(sloretz) is SSL required for sros2? if so, figure out how to enable
  #   "-DENABLE_SSL=OFF")
endmacro()

macro(build_crosscompile_dependencies)
  set(extra_cmake_args ${android_cmake_args})
  # list(APPEND extra_cmake_args -DBUILD_SHARED_LIBS=OFF)
  list(APPEND extra_cmake_args -DCMAKE_BUILD_TYPE=RELEASE)

  dep_build(ament_index_python PIP
    SOURCE_DIR "deps/ament_index/ament_index_python"
    DEPENDENCIES )
  
  dep_build(ament_package PIP
    SOURCE_DIR "deps/ament_package")
  
  dep_build(ament_cmake_core CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_core"
    DEPENDENCIES ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(ament_cmake_export_definitions CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_export_definitions"
    DEPENDENCIES ament_cmake_core ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(ament_cmake_export_dependencies CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_export_dependencies"
    DEPENDENCIES ament_cmake_core ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(ament_cmake CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake"
    DEPENDENCIES ament_cmake_core ament_cmake_export_dependencies ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(ament_cmake_export_include_directories CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_export_include_directories"
    DEPENDENCIES ament_cmake_core ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(ament_cmake_export_interfaces CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_export_interfaces"
    DEPENDENCIES ament_cmake_core ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(ament_cmake_export_libraries CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_export_libraries"
    DEPENDENCIES ament_cmake_core ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(ament_cmake_export_link_flags CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_export_link_flags"
    DEPENDENCIES ament_cmake_core ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(ament_cmake_export_targets CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_export_targets"
    DEPENDENCIES ament_cmake_core ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(ament_cmake_gmock CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_gmock"
    DEPENDENCIES ament_cmake_core ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(ament_cmake_gtest CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_gtest"
    DEPENDENCIES ament_cmake_core ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(ament_cmake_gen_version_h CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_gen_version_h"
    DEPENDENCIES ament_cmake_core ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(ament_cmake_include_directories CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_include_directories"
    DEPENDENCIES ament_cmake_core ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(ament_cmake_libraries CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_libraries"
    DEPENDENCIES ament_cmake_core ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(ament_cmake_nose CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_nose"
    DEPENDENCIES ament_cmake_core ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(ament_cmake_pytest CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_pytest"
    DEPENDENCIES ament_cmake_core ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(ament_cmake_python CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_python"
    DEPENDENCIES ament_cmake_core ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(ament_cmake_google_benchmark CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_google_benchmark"
    DEPENDENCIES ament_cmake_core ament_cmake_python ament_cmake_export_dependencies ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(ament_cmake_target_dependencies CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_target_dependencies"
    DEPENDENCIES ament_cmake_core ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(ament_cmake_test CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_test"
    DEPENDENCIES ament_cmake_core ament_cmake_python ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(ament_cmake_version CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_version"
    DEPENDENCIES ament_cmake_core ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(ament_cmake_auto CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_auto"
    DEPENDENCIES ament_cmake_export_include_directories ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces ament_cmake_export_libraries ament_cmake_core ament_cmake_version ament_cmake_test ament_cmake_python ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gtest ament_cmake_gen_version_h ament_cmake ament_cmake_export_definitions ament_cmake_export_targets ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(ament_index_cpp CMAKE
    SOURCE_DIR "deps/ament_index/ament_index_cpp"
    DEPENDENCIES ament_cmake_export_include_directories ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces ament_cmake_export_libraries ament_cmake_core ament_cmake_version ament_cmake_test ament_cmake_python ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h ament_cmake ament_cmake_export_definitions ament_cmake_export_targets ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(common_interfaces CMAKE
    SOURCE_DIR "deps/common_interfaces/common_interfaces"
    DEPENDENCIES ament_cmake_export_include_directories ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces ament_cmake_export_libraries ament_cmake_core ament_cmake_version ament_cmake_test ament_cmake_python ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h ament_cmake ament_cmake_export_definitions ament_cmake_export_targets ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(domain_coordinator PIP
    SOURCE_DIR "deps/ament_cmake_ros/domain_coordinator"
    DEPENDENCIES )
  
  dep_build(ament_cmake_ros CMAKE
    SOURCE_DIR "deps/ament_cmake_ros/ament_cmake_ros"
    DEPENDENCIES ament_cmake_export_include_directories ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces ament_cmake_export_libraries ament_cmake_core ament_cmake_version ament_cmake_test ament_cmake_python ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h domain_coordinator ament_cmake ament_cmake_export_definitions ament_cmake_export_targets ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  # dep_build(cyclonedds CMAKE
  #   SOURCE_DIR "deps/cyclonedds"
  #   DEPENDENCIES native-cyclonedds
  #   CMAKE_ARGS
  #     # ${extra_cmake_args}  # Can't build this one statically
  #     ${android_cmake_args}
  #     # allow finding native ddsconf tool
  #     "-DCMAKE_PREFIX_PATH=${CMAKE_CURRENT_BINARY_DIR}/native-deps/native-cyclonedds"
  #     # TODO(sloretz) is SSL required for sros2? if so, figure out how to enable
  #     "-DENABLE_SSL=OFF")
  # Manually switching to FastDDS
  dep_build(fastrtps_cmake_module CMAKE
    SOURCE_DIR "deps/ros2/rosidl_typesupport_fastrtps/fastrtps_cmake_module"
    DEPENDENCIES ament_cmake_export_libraries ament_cmake_python ament_cmake_libraries ament_cmake_core ament_cmake_gen_version_h ament_cmake_include_directories ament_cmake_test python3-importlib-metadata ament_cmake_export_link_flags ament_cmake_version ament_cmake_export_interfaces ament_cmake_export_include_directories ament_cmake_export_dependencies ament_cmake ament_package ament_cmake_export_definitions ament_cmake_export_targets ament_cmake_target_dependencies
    CMAKE_ARGS ${android_cmake_args})
  dep_build(fastcdr CMAKE
    SOURCE_DIR "deps/eProsima/Fast-CDR"
    CMAKE_ARGS ${extra_cmake_args})
  dep_build(foonathan_memory_vendor CMAKE
    SOURCE_DIR "deps/eProsima/foonathan_memory_vendor"
    CMAKE_ARGS ${extra_cmake_args})
  dep_build(fastrtps CMAKE
    SOURCE_DIR "deps/eProsima/Fast-DDS"
    DEPENDENCIES
      fastcdr
      foonathan_memory_vendor
    CMAKE_ARGS ${extra_cmake_args}
    "-DTHIRDPARTY_Asio=ON"
    "-DTHIRDPARTY_android-ifaddrs=ON"
    "-DTHIRDPARTY_TinyXML2=ON"
    )
  dep_build(rmw_fastrtps_shared_cpp CMAKE
    SOURCE_DIR "deps/ros2/rmw_fastrtps/rmw_fastrtps_shared_cpp"
    DEPENDENCIES rosidl_typesupport_interface ament_cmake_export_libraries ament_cmake_python fastcdr rosidl_runtime_cpp ament_cmake_libraries ament_cmake_core rosidl_typesupport_introspection_c python3-empy rosidl_typesupport_introspection_cpp ament_cmake_gen_version_h ament_cmake_include_directories ament_cmake_test rcpputils ament_cmake_ros python3-importlib-metadata ament_cmake_export_link_flags ament_cmake_version foonathan_memory_vendor ament_cmake_export_interfaces tinyxml2 gtest ament_cmake_export_include_directories python3 rmw rosidl_cmake libatomic rmw_dds_common python3-pytest rcutils fastrtps ament_cmake_gtest ament_cmake_export_dependencies domain_coordinator ament_cmake_gmock ament_cmake_pytest rosidl_runtime_c tracetools ament_cmake google-mock gmock_vendor gtest_vendor ament_package ament_cmake_export_definitions python3-setuptools python3-catkin-pkg-modules ament_cmake_export_targets ament_cmake_target_dependencies fastrtps_cmake_module cmake libssl-dev python3-importlib-resources
    CMAKE_ARGS ${android_cmake_args})
  dep_build(rmw_fastrtps_dynamic_cpp CMAKE
    SOURCE_DIR "deps/ros2/rmw_fastrtps/rmw_fastrtps_dynamic_cpp"
    DEPENDENCIES rosidl_typesupport_interface ament_cmake_export_libraries ament_cmake_python fastcdr rosidl_runtime_cpp ament_cmake_libraries ament_cmake_core rosidl_typesupport_introspection_c python3-empy rosidl_cli rosidl_typesupport_introspection_cpp ament_cmake_gen_version_h ament_cmake_include_directories ament_cmake_test rcpputils ament_cmake_ros python3-importlib-metadata ament_cmake_export_link_flags ament_cmake_version foonathan_memory_vendor ament_cmake_export_interfaces tinyxml2 gtest ament_cmake_export_include_directories python3 rmw rosidl_cmake libatomic rmw_dds_common python3-pytest rcutils fastrtps ament_cmake_gtest ament_cmake_export_dependencies domain_coordinator rosidl_generator_c ament_cmake_gmock  rmw_fastrtps_shared_cpp rosidl_generator_cpp ament_cmake_pytest rosidl_runtime_c tracetools ament_index_python ament_cmake google-mock gmock_vendor gtest_vendor ament_package ament_cmake_export_definitions python3-setuptools rosidl_typesupport_fastrtps_cpp python3-catkin-pkg-modules ament_cmake_export_targets ament_cmake_target_dependencies fastrtps_cmake_module cmake libssl-dev python3-importlib-resources
    CMAKE_ARGS ${android_cmake_args})
  # End manually switching to FastDDS
  
  dep_build(rcutils CMAKE
    SOURCE_DIR "deps/rcutils"
    DEPENDENCIES ament_cmake_pytest ament_cmake_export_include_directories ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces ament_cmake_export_libraries ament_cmake_core ament_cmake_version ament_cmake_test ament_cmake_python ament_cmake_ros ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gtest ament_cmake_gen_version_h domain_coordinator ament_cmake ament_cmake_gmock ament_cmake_export_definitions ament_cmake_export_targets ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(rcpputils CMAKE
    SOURCE_DIR "deps/rcpputils"
    DEPENDENCIES ament_cmake_pytest ament_cmake_export_include_directories ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces ament_cmake_export_libraries ament_cmake_core ament_cmake_version ament_cmake_test ament_cmake_python rcutils ament_cmake_ros ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gtest ament_cmake_gen_version_h domain_coordinator ament_cmake ament_cmake_gmock ament_cmake_export_definitions ament_cmake_export_targets ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(libyaml_vendor CMAKE
    SOURCE_DIR "deps/libyaml_vendor"
    DEPENDENCIES ament_cmake_export_include_directories ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces ament_cmake_export_libraries ament_cmake_core ament_cmake_version ament_cmake_test ament_cmake_python ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h ament_cmake ament_cmake_export_definitions ament_cmake_export_targets ament_package
    CMAKE_ARGS ${extra_cmake_args})

  dep_build(rcl_logging_interface CMAKE
    SOURCE_DIR "deps/rcl_logging/rcl_logging_interface"
    DEPENDENCIES ament_cmake_pytest ament_cmake_export_include_directories ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces ament_cmake_export_libraries ament_cmake_core ament_cmake_version ament_cmake_test ament_cmake_python rcutils ament_cmake_ros ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gtest ament_cmake_gen_version_h domain_coordinator ament_cmake ament_cmake_gmock ament_cmake_export_definitions ament_cmake_export_targets ament_package
    CMAKE_ARGS ${extra_cmake_args})

  dep_build(rcl_logging_android CMAKE
    SOURCE_DIR "deps/rcl_logging_android"
    DEPENDENCIES ament_cmake_pytest ament_cmake_export_include_directories ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces ament_cmake_export_libraries ament_cmake_core ament_cmake_version ament_cmake_test ament_cmake_python rcutils ament_cmake_ros ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gtest ament_cmake_gen_version_h domain_coordinator ament_cmake ament_cmake_gmock ament_cmake_export_definitions ament_cmake_export_targets rcl_logging_interface ament_package
    CMAKE_ARGS ${extra_cmake_args})

  dep_build(rmw_implementation_cmake CMAKE
    SOURCE_DIR "deps/rmw/rmw_implementation_cmake"
    DEPENDENCIES ament_cmake_export_include_directories ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces ament_cmake_export_libraries ament_cmake_core ament_cmake_version ament_cmake_test ament_cmake_python ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h ament_cmake ament_cmake_export_definitions ament_cmake_export_targets ament_package
    CMAKE_ARGS ${extra_cmake_args})

  dep_build(rosidl_adapter CMAKE
    SOURCE_DIR "deps/rosidl/rosidl_adapter"
    DEPENDENCIES ament_cmake_export_include_directories ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces ament_cmake_export_libraries ament_cmake_core ament_cmake_version ament_cmake_test ament_cmake_python ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h ament_cmake ament_cmake_export_definitions ament_cmake_export_targets ament_package
    CMAKE_ARGS ${extra_cmake_args})

  dep_build(rosidl_cli PIP
    SOURCE_DIR "deps/rosidl/rosidl_cli"
    DEPENDENCIES )

  dep_build(rosidl_cmake CMAKE
    SOURCE_DIR "deps/rosidl/rosidl_cmake"
    DEPENDENCIES ament_cmake_export_include_directories ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces ament_cmake_export_libraries ament_cmake_core ament_cmake_version ament_cmake_test ament_cmake_python ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h ament_cmake ament_cmake_export_definitions ament_cmake_export_targets ament_package
    CMAKE_ARGS ${extra_cmake_args})

  dep_build(rosidl_parser CMAKE
    SOURCE_DIR "deps/rosidl/rosidl_parser"
    DEPENDENCIES ament_cmake_export_include_directories ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces ament_cmake_export_libraries ament_cmake_core ament_cmake_version ament_cmake_test ament_cmake_python ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h ament_cmake ament_cmake_export_definitions ament_cmake_export_targets ament_package
    CMAKE_ARGS ${extra_cmake_args})

  dep_build(rosidl_typesupport_interface CMAKE
    SOURCE_DIR "deps/rosidl/rosidl_typesupport_interface"
    DEPENDENCIES ament_cmake_export_include_directories ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces ament_cmake_export_libraries ament_cmake_core ament_cmake_version ament_cmake_test ament_cmake_python ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h ament_cmake ament_cmake_export_definitions ament_cmake_export_targets ament_package
    CMAKE_ARGS ${extra_cmake_args})

  dep_build(rosidl_runtime_c CMAKE
    SOURCE_DIR "deps/rosidl/rosidl_runtime_c"
    DEPENDENCIES rosidl_typesupport_interface ament_cmake_pytest ament_cmake_export_include_directories ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces ament_cmake_export_libraries ament_cmake_core ament_cmake_version ament_cmake_test ament_cmake_python rcutils ament_cmake_ros ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gtest ament_cmake_gen_version_h domain_coordinator ament_cmake ament_cmake_gmock ament_cmake_export_definitions ament_cmake_export_targets ament_package
    CMAKE_ARGS ${extra_cmake_args})

  dep_build(rmw CMAKE
    SOURCE_DIR "deps/rmw/rmw"
    DEPENDENCIES rosidl_typesupport_interface ament_cmake_pytest ament_cmake_export_include_directories ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces ament_cmake_export_libraries ament_cmake_core ament_cmake_version ament_cmake_test rcutils ament_cmake_python rosidl_runtime_c ament_cmake_ros ament_cmake_export_dependencies ament_cmake_gtest ament_cmake_target_dependencies ament_cmake_gen_version_h domain_coordinator ament_cmake ament_cmake_gmock ament_cmake_export_definitions ament_cmake_export_targets ament_package
    CMAKE_ARGS ${extra_cmake_args})

  dep_build(rcl_yaml_param_parser CMAKE
    SOURCE_DIR "deps/rcl/rcl_yaml_param_parser"
    DEPENDENCIES rosidl_typesupport_interface ament_cmake_pytest ament_cmake_export_include_directories ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces ament_cmake_export_libraries ament_cmake_core ament_cmake_version ament_cmake_test rcutils ament_cmake_python libyaml_vendor rosidl_runtime_c ament_cmake_ros ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gtest ament_cmake_gen_version_h domain_coordinator ament_cmake ament_cmake_gmock ament_cmake_export_definitions rmw ament_cmake_export_targets ament_package
    CMAKE_ARGS ${extra_cmake_args})

  dep_build(rosidl_generator_c CMAKE
    SOURCE_DIR "deps/rosidl/rosidl_generator_c"
    DEPENDENCIES ament_cmake_pytest ament_cmake_export_include_directories ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces ament_cmake_export_libraries ament_cmake_core ament_cmake_version ament_cmake_test ament_cmake_python ament_cmake_ros ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gtest ament_cmake_gen_version_h domain_coordinator ament_cmake ament_cmake_gmock ament_cmake_export_definitions ament_cmake_export_targets ament_package
    CMAKE_ARGS ${extra_cmake_args})

  dep_build(rosidl_runtime_cpp CMAKE
    SOURCE_DIR "deps/rosidl/rosidl_runtime_cpp"
    DEPENDENCIES rosidl_typesupport_interface ament_cmake_export_include_directories ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces ament_cmake_export_libraries ament_cmake_core ament_cmake_version ament_cmake_test rcutils ament_cmake_python rosidl_runtime_c ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h ament_cmake ament_cmake_export_definitions ament_cmake_export_targets ament_package
    CMAKE_ARGS ${extra_cmake_args})

  # Manually switching to Fast-DDS
  dep_build(rosidl_typesupport_fastrtps_cpp CMAKE
    SOURCE_DIR "deps/ros2/rosidl_typesupport_fastrtps/rosidl_typesupport_fastrtps_cpp"
    DEPENDENCIES rosidl_typesupport_interface ament_cmake_export_libraries ament_cmake_python fastcdr ament_cmake_libraries ament_cmake_core python3-empy rosidl_cli ament_cmake_gen_version_h ament_cmake_include_directories ament_cmake_test python3-importlib-metadata ament_cmake_export_link_flags ament_cmake_version ament_cmake_export_interfaces ament_cmake_export_include_directories rmw rosidl_cmake libatomic rcutils ament_cmake_export_dependencies rosidl_runtime_c ament_index_python ament_cmake ament_package ament_cmake_export_definitions python3-setuptools python3-catkin-pkg-modules ament_cmake_export_targets ament_cmake_target_dependencies fastrtps_cmake_module cmake python3-importlib-resources
    CMAKE_ARGS ${android_cmake_args})
  dep_build(rosidl_typesupport_fastrtps_c CMAKE
    SOURCE_DIR "deps/ros2/rosidl_typesupport_fastrtps/rosidl_typesupport_fastrtps_c"
    DEPENDENCIES rosidl_typesupport_interface ament_cmake_export_libraries ament_cmake_python fastcdr rosidl_runtime_cpp ament_cmake_libraries ament_cmake_core python3-empy rosidl_cli ament_cmake_gen_version_h ament_cmake_include_directories ament_cmake_test ament_cmake_ros python3-importlib-metadata ament_cmake_export_link_flags ament_cmake_version ament_cmake_export_interfaces ament_cmake_export_include_directories gtest python3 rmw rosidl_cmake libatomic python3-pytest rosidl_generator_c rcutils ament_cmake_gtest ament_cmake_export_dependencies domain_coordinator ament_cmake_gmock rosidl_generator_cpp ament_cmake_pytest rosidl_runtime_c ament_index_python ament_cmake google-mock gmock_vendor gtest_vendor ament_package ament_cmake_export_definitions python3-setuptools rosidl_typesupport_fastrtps_cpp python3-catkin-pkg-modules ament_cmake_export_targets ament_cmake_target_dependencies fastrtps_cmake_module cmake python3-importlib-resources
    CMAKE_ARGS ${android_cmake_args})
  # End manually switching to Fast-DDS

  dep_build(rosidl_generator_cpp CMAKE
    SOURCE_DIR "deps/rosidl/rosidl_generator_cpp"
    DEPENDENCIES ament_cmake_export_include_directories ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces ament_cmake_export_libraries ament_cmake_core ament_cmake_version ament_cmake_test ament_cmake_python ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h ament_cmake ament_cmake_export_definitions ament_cmake_export_targets ament_package
    CMAKE_ARGS ${extra_cmake_args})

  dep_build(rosidl_typesupport_introspection_c CMAKE
    SOURCE_DIR "deps/rosidl/rosidl_typesupport_introspection_c"
    DEPENDENCIES ament_cmake_pytest ament_cmake_export_include_directories ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces ament_cmake_export_libraries ament_cmake_core ament_cmake_version ament_cmake_test ament_cmake_python ament_cmake_ros ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gtest ament_cmake_gen_version_h domain_coordinator ament_cmake ament_cmake_gmock ament_cmake_export_definitions ament_cmake_export_targets ament_package
    CMAKE_ARGS ${extra_cmake_args})

  dep_build(rosidl_typesupport_introspection_cpp CMAKE
    SOURCE_DIR "deps/rosidl/rosidl_typesupport_introspection_cpp"
    DEPENDENCIES rosidl_typesupport_interface rosidl_runtime_cpp ament_cmake_pytest ament_cmake_export_include_directories ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces ament_cmake_export_libraries ament_cmake_core ament_cmake_version ament_cmake_test rcutils ament_cmake_python rosidl_runtime_c ament_cmake_ros ament_cmake_export_dependencies ament_cmake_gtest ament_cmake_target_dependencies ament_cmake_gen_version_h domain_coordinator ament_cmake ament_cmake_gmock ament_cmake_export_definitions rosidl_cmake rosidl_typesupport_introspection_c ament_cmake_export_targets ament_package
    CMAKE_ARGS ${extra_cmake_args})

  dep_build(rosidl_typesupport_c CMAKE
    SOURCE_DIR "deps/rosidl_typesupport/rosidl_typesupport_c"
    DEPENDENCIES rosidl_typesupport_interface ament_cmake_pytest ament_cmake_export_include_directories ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces ament_cmake_export_libraries ament_cmake_core ament_cmake_version ament_cmake_test rcutils ament_cmake_python rosidl_runtime_c ament_cmake_ros ament_cmake_export_dependencies ament_cmake_gtest ament_cmake_target_dependencies ament_cmake_gen_version_h domain_coordinator ament_cmake ament_cmake_gmock ament_cmake_export_definitions rosidl_cmake rosidl_typesupport_introspection_c ament_cmake_export_targets rcpputils ament_package
    CMAKE_ARGS ${extra_cmake_args})

  dep_build(rosidl_typesupport_cpp CMAKE
    SOURCE_DIR "deps/rosidl_typesupport/rosidl_typesupport_cpp"
    DEPENDENCIES rosidl_runtime_cpp rosidl_typesupport_interface ament_cmake_pytest ament_cmake_export_include_directories ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces ament_cmake_export_libraries ament_cmake_core rosidl_typesupport_introspection_cpp ament_cmake_version ament_cmake_test rcutils ament_cmake_python rosidl_runtime_c ament_cmake_ros ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h domain_coordinator ament_cmake_gtest ament_cmake ament_cmake_gmock ament_cmake_export_definitions rosidl_typesupport_c rosidl_cmake ament_cmake_export_targets rcpputils ament_package
    # Hack, need a typesupport using rosidl_typesupport_c to exist before cpp typesupport can be built
    # rosidl_typesupport_introspection_c
    CMAKE_ARGS ${extra_cmake_args})

  dep_build(rosidl_default_generators CMAKE
    SOURCE_DIR "deps/rosidl_defaults/rosidl_default_generators"
    DEPENDENCIES ament_cmake_export_include_directories ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces ament_cmake_export_libraries ament_cmake_core ament_cmake_version ament_cmake_test ament_cmake_python ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h ament_cmake ament_cmake_export_definitions ament_cmake_export_targets ament_package
    CMAKE_ARGS ${extra_cmake_args})

  dep_build(builtin_interfaces CMAKE
    SOURCE_DIR "deps/rcl_interfaces/builtin_interfaces"
    DEPENDENCIES rosidl_runtime_cpp rosidl_typesupport_interface ament_cmake_export_include_directories ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces ament_cmake_export_libraries ament_cmake_core rosidl_typesupport_introspection_cpp ament_cmake_version ament_cmake_test rcutils ament_cmake_python rosidl_runtime_c ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h ament_cmake ament_cmake_export_definitions rosidl_typesupport_c rosidl_cmake rosidl_typesupport_introspection_c rosidl_generator_cpp ament_cmake_export_targets rosidl_typesupport_cpp rcpputils rosidl_generator_c rosidl_default_generators ament_package
    fastcdr
    fastrtps
    fastrtps_cmake_module
    # rosidl_typesupport_fastrtps_c
    # rosidl_typesupport_fastrtps_cpp
    CMAKE_ARGS ${extra_cmake_args})

  dep_build(lifecycle_msgs CMAKE
    SOURCE_DIR "deps/rcl_interfaces/lifecycle_msgs"
    DEPENDENCIES rosidl_runtime_cpp rosidl_typesupport_interface ament_cmake_export_include_directories ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces ament_cmake_export_libraries ament_cmake_core rosidl_typesupport_introspection_cpp ament_cmake_version ament_cmake_test rcutils ament_cmake_python rosidl_runtime_c ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h ament_cmake ament_cmake_export_definitions rosidl_typesupport_c rosidl_cmake rosidl_typesupport_introspection_c rosidl_generator_cpp ament_cmake_export_targets rosidl_typesupport_cpp rcpputils rosidl_generator_c rosidl_default_generators ament_package
    fastcdr
    fastrtps
    fastrtps_cmake_module
    # rosidl_typesupport_fastrtps_c
    # rosidl_typesupport_fastrtps_cpp
    CMAKE_ARGS ${extra_cmake_args})

  dep_build(rmw_dds_common CMAKE
    SOURCE_DIR "deps/rmw_dds_common/rmw_dds_common"
    DEPENDENCIES rosidl_runtime_cpp rosidl_typesupport_interface ament_cmake_export_include_directories ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces ament_cmake_export_libraries ament_cmake_core ament_cmake_version rosidl_typesupport_introspection_cpp ament_cmake_test rcutils ament_cmake_python rosidl_runtime_c ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h ament_cmake ament_cmake_export_definitions rosidl_typesupport_c rosidl_cmake rosidl_typesupport_introspection_c rmw rosidl_generator_cpp ament_cmake_export_targets rosidl_typesupport_cpp rcpputils rosidl_generator_c rosidl_default_generators ament_package
    CMAKE_ARGS ${extra_cmake_args})

  dep_build(rosidl_default_runtime CMAKE
    SOURCE_DIR "deps/rosidl_defaults/rosidl_default_runtime"
    DEPENDENCIES ament_cmake_export_include_directories ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces ament_cmake_export_libraries ament_cmake_core ament_cmake_version ament_cmake_test ament_cmake_python ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h ament_cmake ament_cmake_export_definitions ament_cmake_export_targets ament_package
    CMAKE_ARGS ${extra_cmake_args})

  dep_build(rcl_interfaces CMAKE
    SOURCE_DIR "deps/rcl_interfaces/rcl_interfaces"
    DEPENDENCIES rosidl_runtime_cpp rosidl_typesupport_interface ament_cmake_export_include_directories ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces ament_cmake_export_libraries ament_cmake_core rosidl_typesupport_introspection_cpp ament_cmake_version ament_cmake_python ament_cmake_test rcutils rosidl_runtime_c ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h builtin_interfaces ament_cmake ament_cmake_export_definitions rosidl_typesupport_c rosidl_cmake rosidl_typesupport_introspection_c rosidl_generator_cpp ament_cmake_export_targets rosidl_typesupport_cpp rcpputils rosidl_generator_c rosidl_default_generators ament_package
    fastcdr
    fastrtps
    fastrtps_cmake_module
    # rosidl_typesupport_fastrtps_c
    # rosidl_typesupport_fastrtps_cpp
    CMAKE_ARGS ${extra_cmake_args})

  dep_build(composition_interfaces CMAKE
    SOURCE_DIR "deps/rcl_interfaces/composition_interfaces"
    DEPENDENCIES rosidl_runtime_cpp rosidl_typesupport_interface ament_cmake_export_include_directories ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces ament_cmake_export_libraries ament_cmake_core rosidl_typesupport_introspection_cpp ament_cmake_version ament_cmake_python ament_cmake_test rcutils rcl_interfaces rosidl_runtime_c ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h builtin_interfaces ament_cmake ament_cmake_export_definitions rosidl_typesupport_c rosidl_cmake rosidl_typesupport_introspection_c rosidl_generator_cpp ament_cmake_export_targets rosidl_typesupport_cpp rcpputils rosidl_generator_c rosidl_default_generators ament_package
    fastcdr
    fastrtps
    fastrtps_cmake_module
    # rosidl_typesupport_fastrtps_c
    # rosidl_typesupport_fastrtps_cpp
    CMAKE_ARGS ${extra_cmake_args})

  dep_build(rosgraph_msgs CMAKE
    SOURCE_DIR "deps/rcl_interfaces/rosgraph_msgs"
    DEPENDENCIES rosidl_runtime_cpp rosidl_typesupport_interface ament_cmake_export_include_directories ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces ament_cmake_export_libraries ament_cmake_core rosidl_typesupport_introspection_cpp ament_cmake_version ament_cmake_python ament_cmake_test rcutils rosidl_runtime_c ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h builtin_interfaces ament_cmake ament_cmake_export_definitions rosidl_typesupport_c rosidl_cmake rosidl_typesupport_introspection_c rosidl_generator_cpp ament_cmake_export_targets rosidl_typesupport_cpp rcpputils rosidl_generator_c rosidl_default_generators ament_package
    fastcdr
    fastrtps
    fastrtps_cmake_module
    # rosidl_typesupport_fastrtps_c
    # rosidl_typesupport_fastrtps_cpp
    CMAKE_ARGS ${extra_cmake_args})

  dep_build(statistics_msgs CMAKE
    SOURCE_DIR "deps/rcl_interfaces/statistics_msgs"
    DEPENDENCIES rosidl_runtime_cpp rosidl_typesupport_interface ament_cmake_export_include_directories ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces ament_cmake_export_libraries ament_cmake_core rosidl_typesupport_introspection_cpp ament_cmake_version ament_cmake_python ament_cmake_test rcutils rosidl_runtime_c ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h builtin_interfaces ament_cmake ament_cmake_export_definitions rosidl_typesupport_c rosidl_cmake rosidl_typesupport_introspection_c rosidl_generator_cpp ament_cmake_export_targets rosidl_typesupport_cpp rcpputils rosidl_generator_c rosidl_default_generators ament_package
    fastcdr
    fastrtps
    fastrtps_cmake_module
    # rosidl_typesupport_fastrtps_c
    # rosidl_typesupport_fastrtps_cpp
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(std_msgs CMAKE
    SOURCE_DIR "deps/common_interfaces/std_msgs"
    DEPENDENCIES rosidl_runtime_cpp rosidl_typesupport_interface ament_cmake_export_include_directories ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces ament_cmake_export_libraries ament_cmake_core rosidl_typesupport_introspection_cpp ament_cmake_version ament_cmake_python ament_cmake_test rcutils rosidl_runtime_c ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h builtin_interfaces ament_cmake ament_cmake_export_definitions rosidl_typesupport_c rosidl_cmake rosidl_typesupport_introspection_c rosidl_generator_cpp ament_cmake_export_targets rosidl_typesupport_cpp rcpputils rosidl_generator_c rosidl_default_generators ament_package
    fastcdr
    fastrtps
    fastrtps_cmake_module
    # rosidl_typesupport_fastrtps_c
    # rosidl_typesupport_fastrtps_cpp
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(actionlib_msgs CMAKE
    SOURCE_DIR "deps/common_interfaces/actionlib_msgs"
    DEPENDENCIES rosidl_runtime_cpp rosidl_typesupport_interface ament_cmake_export_include_directories ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces std_msgs ament_cmake_export_libraries ament_cmake_core rosidl_typesupport_introspection_cpp ament_cmake_version ament_cmake_python ament_cmake_test rcutils rosidl_runtime_c ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h builtin_interfaces ament_cmake ament_cmake_export_definitions rosidl_typesupport_c rosidl_cmake rosidl_typesupport_introspection_c rosidl_generator_cpp ament_cmake_export_targets rosidl_typesupport_cpp rcpputils rosidl_generator_c rosidl_default_generators ament_package
    fastcdr
    fastrtps
    fastrtps_cmake_module
    # rosidl_typesupport_fastrtps_c
    # rosidl_typesupport_fastrtps_cpp
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(geometry_msgs CMAKE
    SOURCE_DIR "deps/common_interfaces/geometry_msgs"
    DEPENDENCIES rosidl_runtime_cpp rosidl_typesupport_interface ament_cmake_export_include_directories ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces std_msgs ament_cmake_export_libraries ament_cmake_core rosidl_typesupport_introspection_cpp ament_cmake_version ament_cmake_python ament_cmake_test rcutils rosidl_runtime_c ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h builtin_interfaces ament_cmake ament_cmake_export_definitions rosidl_typesupport_c rosidl_cmake rosidl_typesupport_introspection_c rosidl_generator_cpp ament_cmake_export_targets rosidl_typesupport_cpp rcpputils rosidl_generator_c rosidl_default_generators ament_package
    fastcdr
    fastrtps
    fastrtps_cmake_module
    # rosidl_typesupport_fastrtps_c
    # rosidl_typesupport_fastrtps_cpp
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(diagnostic_msgs CMAKE
    SOURCE_DIR "deps/common_interfaces/diagnostic_msgs"
    DEPENDENCIES rosidl_runtime_cpp rosidl_typesupport_interface geometry_msgs ament_cmake_export_include_directories ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces std_msgs ament_cmake_export_libraries ament_cmake_core rosidl_typesupport_introspection_cpp ament_cmake_version ament_cmake_python ament_cmake_test rcutils rosidl_runtime_c ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h builtin_interfaces ament_cmake ament_cmake_export_definitions rosidl_typesupport_c rosidl_cmake rosidl_typesupport_introspection_c rosidl_generator_cpp ament_cmake_export_targets rosidl_typesupport_cpp rcpputils rosidl_generator_c rosidl_default_generators ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(nav_msgs CMAKE
    SOURCE_DIR "deps/common_interfaces/nav_msgs"
    DEPENDENCIES rosidl_runtime_cpp rosidl_typesupport_interface geometry_msgs ament_cmake_export_include_directories ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces std_msgs ament_cmake_export_libraries ament_cmake_core rosidl_typesupport_introspection_cpp ament_cmake_version ament_cmake_python ament_cmake_test rcutils rosidl_runtime_c ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h builtin_interfaces ament_cmake ament_cmake_export_definitions rosidl_typesupport_c rosidl_cmake rosidl_typesupport_introspection_c rosidl_generator_cpp ament_cmake_export_targets rosidl_typesupport_cpp rcpputils rosidl_generator_c rosidl_default_generators ament_package
    fastcdr
    fastrtps
    fastrtps_cmake_module
    # rosidl_typesupport_fastrtps_c
    # rosidl_typesupport_fastrtps_cpp
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(sensor_msgs CMAKE
    SOURCE_DIR "deps/common_interfaces/sensor_msgs"
    DEPENDENCIES rosidl_runtime_cpp rosidl_typesupport_interface geometry_msgs ament_cmake_export_include_directories ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces std_msgs ament_cmake_export_libraries ament_cmake_core rosidl_typesupport_introspection_cpp ament_cmake_version ament_cmake_python ament_cmake_test rcutils rosidl_runtime_c ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h builtin_interfaces ament_cmake ament_cmake_export_definitions rosidl_typesupport_c rosidl_cmake rosidl_typesupport_introspection_c rosidl_generator_cpp ament_cmake_export_targets rosidl_typesupport_cpp rcpputils rosidl_generator_c rosidl_default_generators ament_package
    fastcdr
    fastrtps
    fastrtps_cmake_module
    # rosidl_typesupport_fastrtps_c
    # rosidl_typesupport_fastrtps_cpp
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(shape_msgs CMAKE
    SOURCE_DIR "deps/common_interfaces/shape_msgs"
    DEPENDENCIES rosidl_runtime_cpp rosidl_typesupport_interface geometry_msgs ament_cmake_export_include_directories ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces std_msgs ament_cmake_export_libraries ament_cmake_core rosidl_typesupport_introspection_cpp ament_cmake_version ament_cmake_python ament_cmake_test rcutils rosidl_runtime_c ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h builtin_interfaces ament_cmake ament_cmake_export_definitions rosidl_typesupport_c rosidl_cmake rosidl_typesupport_introspection_c rosidl_generator_cpp ament_cmake_export_targets rosidl_typesupport_cpp rcpputils rosidl_generator_c rosidl_default_generators ament_package
    fastcdr
    fastrtps
    fastrtps_cmake_module
    # rosidl_typesupport_fastrtps_c
    # rosidl_typesupport_fastrtps_cpp
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(std_srvs CMAKE
    SOURCE_DIR "deps/common_interfaces/std_srvs"
    DEPENDENCIES rosidl_runtime_cpp rosidl_typesupport_interface ament_cmake_export_include_directories ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces ament_cmake_export_libraries ament_cmake_core rosidl_typesupport_introspection_cpp ament_cmake_version ament_cmake_test rcutils ament_cmake_python rosidl_runtime_c ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h ament_cmake ament_cmake_export_definitions rosidl_typesupport_c rosidl_cmake rosidl_typesupport_introspection_c rosidl_generator_cpp ament_cmake_export_targets rosidl_typesupport_cpp rcpputils rosidl_generator_c rosidl_default_generators ament_package
    fastcdr
    fastrtps
    fastrtps_cmake_module
    # rosidl_typesupport_fastrtps_c
    # rosidl_typesupport_fastrtps_cpp
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(stereo_msgs CMAKE
    SOURCE_DIR "deps/common_interfaces/stereo_msgs"
    DEPENDENCIES rosidl_runtime_cpp rosidl_typesupport_interface geometry_msgs ament_cmake_export_include_directories ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces std_msgs ament_cmake_export_libraries sensor_msgs ament_cmake_core rosidl_typesupport_introspection_cpp ament_cmake_version ament_cmake_python ament_cmake_test rcutils rosidl_runtime_c ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h builtin_interfaces ament_cmake ament_cmake_export_definitions rosidl_typesupport_c rosidl_cmake rosidl_typesupport_introspection_c rosidl_generator_cpp ament_cmake_export_targets rosidl_typesupport_cpp rcpputils rosidl_generator_c rosidl_default_generators ament_package
    fastcdr
    fastrtps
    fastrtps_cmake_module
    rosidl_typesupport_fastrtps_c
    rosidl_typesupport_fastrtps_cpp
    CMAKE_ARGS ${extra_cmake_args})

  dep_build(tracetools CMAKE
    SOURCE_DIR "deps/ros2_tracing/tracetools"
    DEPENDENCIES ament_cmake_pytest ament_cmake_export_include_directories ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces ament_cmake_export_libraries ament_cmake_core ament_cmake_version ament_cmake_test ament_cmake_python ament_cmake_ros ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gtest ament_cmake_gen_version_h domain_coordinator ament_cmake ament_cmake_gmock ament_cmake_export_definitions ament_cmake_export_targets ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  #  dep_build(rmw_cyclonedds_cpp CMAKE
  #    SOURCE_DIR "deps/rmw_cyclonedds/rmw_cyclonedds_cpp"
  #    DEPENDENCIES rosidl_runtime_cpp rosidl_typesupport_interface ament_cmake_pytest ament_cmake_export_include_directories tracetools ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces ament_cmake_export_libraries ament_cmake_core rosidl_typesupport_introspection_cpp ament_cmake_version ament_cmake_test rcutils ament_cmake_python rosidl_runtime_c ament_cmake_ros ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h domain_coordinator ament_cmake_gtest ament_cmake ament_cmake_gmock ament_cmake_export_definitions rosidl_cmake rosidl_typesupport_introspection_c rmw ament_cmake_export_targets rcpputils rmw_dds_common cyclonedds ament_package
  #    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(rmw_implementation CMAKE
    SOURCE_DIR "deps/rmw_implementation/rmw_implementation"
    DEPENDENCIES rosidl_runtime_cpp rosidl_typesupport_interface ament_cmake_export_include_directories tracetools ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces ament_cmake_export_libraries rmw_implementation_cmake ament_index_cpp ament_cmake_core ament_cmake_version rosidl_typesupport_introspection_cpp ament_cmake_test rcutils ament_cmake_python rosidl_runtime_c ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h ament_cmake ament_cmake_export_definitions rosidl_cmake rosidl_typesupport_introspection_c rmw ament_cmake_export_targets rcpputils rmw_dds_common ament_package
    # rmw_cyclonedds_cpp
    # cyclonedds
    rmw_fastrtps_dynamic_cpp
    CMAKE_ARGS ${extra_cmake_args})

  dep_build(rcl CMAKE
    SOURCE_DIR "deps/rcl/rcl"
    DEPENDENCIES rosidl_typesupport_interface rcl_yaml_param_parser ament_cmake_pytest ament_cmake_export_include_directories tracetools ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces ament_cmake_export_libraries rmw_implementation_cmake rmw_implementation ament_index_cpp ament_cmake_core ament_cmake_version ament_cmake_test rcutils ament_cmake_python rcl_interfaces libyaml_vendor rosidl_runtime_c ament_cmake_ros ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h domain_coordinator builtin_interfaces ament_cmake_gtest ament_cmake ament_cmake_gmock ament_cmake_export_definitions rmw ament_cmake_export_targets rcpputils rcl_logging_interface ament_package
    rcl_logging_android
    CMAKE_ARGS ${extra_cmake_args}
    -DRCL_LOGGING_IMPLEMENTATION=rcl_logging_android
    )

  dep_build(libstatistics_collector CMAKE
    SOURCE_DIR "deps/libstatistics_collector"
    DEPENDENCIES rosidl_typesupport_interface rcl_yaml_param_parser ament_cmake_pytest rosidl_runtime_cpp ament_cmake_export_include_directories tracetools ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces statistics_msgs std_msgs ament_cmake_export_libraries rmw_implementation_cmake rmw_implementation ament_index_cpp ament_cmake_core ament_cmake_version rosidl_typesupport_introspection_cpp ament_cmake_python ament_cmake_test rcutils rcl_interfaces libyaml_vendor rosidl_runtime_c ament_cmake_ros ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h domain_coordinator builtin_interfaces rcl ament_cmake_gtest ament_cmake ament_cmake_gmock ament_cmake_export_definitions rosidl_typesupport_c rosidl_cmake rosidl_typesupport_introspection_c rmw rosidl_generator_cpp ament_cmake_export_targets rosidl_typesupport_cpp rcpputils rosidl_generator_c rosidl_default_generators rcl_logging_interface ament_package
    CMAKE_ARGS ${extra_cmake_args})

  dep_build(rcl_lifecycle CMAKE
    SOURCE_DIR "deps/rcl/rcl_lifecycle"
    DEPENDENCIES rosidl_typesupport_interface rcl_yaml_param_parser ament_cmake_pytest lifecycle_msgs ament_cmake_export_include_directories tracetools ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces ament_cmake_export_libraries rmw_implementation_cmake rmw_implementation ament_index_cpp ament_cmake_core ament_cmake_version ament_cmake_test rcutils ament_cmake_python rcl_interfaces libyaml_vendor rosidl_runtime_c ament_cmake_ros ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h domain_coordinator builtin_interfaces rcl ament_cmake_gtest ament_cmake ament_cmake_gmock ament_cmake_export_definitions rmw ament_cmake_export_targets rcpputils rcl_logging_interface ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(rclcpp CMAKE
    SOURCE_DIR "deps/rclcpp/rclcpp"
    DEPENDENCIES rosidl_typesupport_interface rcl_yaml_param_parser rosidl_runtime_cpp ament_cmake_pytest ament_cmake_export_include_directories tracetools rosgraph_msgs ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces statistics_msgs ament_cmake_export_libraries rmw_implementation_cmake rmw_implementation ament_index_cpp ament_cmake_core ament_cmake_version ament_cmake_test ament_cmake_python rcutils rcl_interfaces libyaml_vendor rosidl_runtime_c ament_cmake_ros ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h domain_coordinator builtin_interfaces rcl ament_cmake_gtest ament_cmake ament_cmake_gmock ament_cmake_export_definitions rosidl_typesupport_c rmw ament_cmake_export_targets rosidl_typesupport_cpp libstatistics_collector rcpputils rcl_logging_interface ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(rclcpp_lifecycle CMAKE
    SOURCE_DIR "deps/rclcpp/rclcpp_lifecycle"
    DEPENDENCIES rclcpp rosidl_typesupport_interface rcl_yaml_param_parser rosidl_runtime_cpp ament_cmake_pytest lifecycle_msgs ament_cmake_export_include_directories tracetools rosgraph_msgs ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces statistics_msgs rcl_lifecycle ament_cmake_export_libraries rmw_implementation_cmake rmw_implementation ament_index_cpp ament_cmake_core ament_cmake_version ament_cmake_test rcutils ament_cmake_python rcl_interfaces libyaml_vendor rosidl_runtime_c ament_cmake_ros ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h domain_coordinator builtin_interfaces rcl ament_cmake_gtest ament_cmake ament_cmake_gmock ament_cmake_export_definitions rosidl_typesupport_c rmw ament_cmake_export_targets rosidl_typesupport_cpp libstatistics_collector rcpputils rcl_logging_interface ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(trajectory_msgs CMAKE
    SOURCE_DIR "deps/common_interfaces/trajectory_msgs"
    DEPENDENCIES rosidl_runtime_cpp rosidl_typesupport_interface geometry_msgs ament_cmake_export_include_directories ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces std_msgs ament_cmake_export_libraries ament_cmake_core rosidl_typesupport_introspection_cpp ament_cmake_version ament_cmake_python ament_cmake_test rcutils rosidl_runtime_c ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h builtin_interfaces ament_cmake ament_cmake_export_definitions rosidl_typesupport_c rosidl_cmake rosidl_typesupport_introspection_c rosidl_generator_cpp ament_cmake_export_targets rosidl_typesupport_cpp rcpputils rosidl_generator_c rosidl_default_generators ament_package
    fastcdr
    fastrtps
    fastrtps_cmake_module
    rosidl_typesupport_fastrtps_c
    rosidl_typesupport_fastrtps_cpp
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(visualization_msgs CMAKE
    SOURCE_DIR "deps/common_interfaces/visualization_msgs"
    DEPENDENCIES rosidl_runtime_cpp rosidl_typesupport_interface geometry_msgs ament_cmake_export_include_directories ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces std_msgs ament_cmake_export_libraries sensor_msgs ament_cmake_core rosidl_typesupport_introspection_cpp ament_cmake_version ament_cmake_python ament_cmake_test rcutils rosidl_runtime_c ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h builtin_interfaces ament_cmake ament_cmake_export_definitions rosidl_typesupport_c rosidl_cmake rosidl_typesupport_introspection_c rosidl_generator_cpp ament_cmake_export_targets rosidl_typesupport_cpp rcpputils rosidl_generator_c rosidl_default_generators ament_package
    fastcdr
    fastrtps
    fastrtps_cmake_module
    rosidl_typesupport_fastrtps_c
    rosidl_typesupport_fastrtps_cpp
    CMAKE_ARGS ${extra_cmake_args})
endmacro()
