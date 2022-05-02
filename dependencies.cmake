include(dep_build.cmake)

macro(build_native_dependencies)
  dep_build(cyclonedds-native CMAKE
    SOURCE_DIR "deps/cyclonedds"
    DEPENDENCIES libssl-dev bison
    CMAKE_ARGS "-DBUILD_DDSCONF=ON"
    # TODO(sloretz) is SSL required for sros2? if so, figure out how to enable
    "-DENABLE_SSL=OFF")
endmacro()

macro(build_crosscompile_dependencies)
  dep_build(ament_index_python PIP
    SOURCE_DIR "deps/ament_index/ament_index_python"
    DEPENDENCIES )
  
  dep_build(ament_package PIP
    SOURCE_DIR "deps/ament_package"
    DEPENDENCIES python3-importlib-metadata python3-setuptools python3-importlib-resources)
  
  dep_build(ament_cmake_core CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_core"
    DEPENDENCIES cmake python3-catkin-pkg-modules ament_package
    CMAKE_ARGS ${android_cmake_args})
  
  dep_build(ament_cmake_export_definitions CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_export_definitions"
    DEPENDENCIES ament_cmake_core
    CMAKE_ARGS ${android_cmake_args})
  
  dep_build(ament_cmake_export_dependencies CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_export_dependencies"
    DEPENDENCIES ament_cmake_core
    CMAKE_ARGS ${android_cmake_args})
  
  dep_build(ament_cmake CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake"
    DEPENDENCIES ament_cmake_export_dependencies cmake ament_cmake_core
    CMAKE_ARGS ${android_cmake_args})
  
  dep_build(ament_cmake_export_include_directories CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_export_include_directories"
    DEPENDENCIES ament_cmake_core
    CMAKE_ARGS ${android_cmake_args})
  
  dep_build(ament_cmake_export_interfaces CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_export_interfaces"
    DEPENDENCIES ament_cmake_core
    CMAKE_ARGS ${android_cmake_args})
  
  dep_build(ament_cmake_export_libraries CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_export_libraries"
    DEPENDENCIES ament_cmake_core
    CMAKE_ARGS ${android_cmake_args})
  
  dep_build(ament_cmake_export_link_flags CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_export_link_flags"
    DEPENDENCIES ament_cmake_core
    CMAKE_ARGS ${android_cmake_args})
  
  dep_build(ament_cmake_export_targets CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_export_targets"
    DEPENDENCIES ament_cmake_core
    CMAKE_ARGS ${android_cmake_args})
  
  dep_build(ament_cmake_gmock CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_gmock"
    DEPENDENCIES ament_cmake_core
    CMAKE_ARGS ${android_cmake_args})
  
  dep_build(ament_cmake_gtest CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_gtest"
    DEPENDENCIES ament_cmake_core
    CMAKE_ARGS ${android_cmake_args})
  
  dep_build(ament_cmake_gen_version_h CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_gen_version_h"
    DEPENDENCIES ament_cmake_core ament_package
    CMAKE_ARGS ${android_cmake_args})
  
  dep_build(ament_cmake_include_directories CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_include_directories"
    DEPENDENCIES ament_cmake_core
    CMAKE_ARGS ${android_cmake_args})
  
  dep_build(ament_cmake_libraries CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_libraries"
    DEPENDENCIES ament_cmake_core
    CMAKE_ARGS ${android_cmake_args})
  
  dep_build(ament_cmake_nose CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_nose"
    DEPENDENCIES ament_cmake_core
    CMAKE_ARGS ${android_cmake_args})
  
  dep_build(ament_cmake_pytest CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_pytest"
    DEPENDENCIES ament_cmake_core
    CMAKE_ARGS ${android_cmake_args})
  
  dep_build(ament_cmake_python CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_python"
    DEPENDENCIES ament_cmake_core
    CMAKE_ARGS ${android_cmake_args})
  
  dep_build(ament_cmake_google_benchmark CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_google_benchmark"
    DEPENDENCIES ament_cmake_python ament_cmake_export_dependencies ament_cmake_core
    CMAKE_ARGS ${android_cmake_args})
  
  dep_build(ament_cmake_target_dependencies CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_target_dependencies"
    DEPENDENCIES ament_cmake_core
    CMAKE_ARGS ${android_cmake_args})
  
  dep_build(ament_cmake_test CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_test"
    DEPENDENCIES ament_cmake_python ament_cmake_core
    CMAKE_ARGS ${android_cmake_args})
  
  dep_build(ament_cmake_version CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_version"
    DEPENDENCIES ament_cmake_core
    CMAKE_ARGS ${android_cmake_args})
  
  dep_build(ament_cmake_auto CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_auto"
    DEPENDENCIES ament_cmake_gtest ament_cmake
    CMAKE_ARGS ${android_cmake_args})
  
  dep_build(ament_index_cpp CMAKE
    SOURCE_DIR "deps/ament_index/ament_index_cpp"
    DEPENDENCIES ament_cmake
    CMAKE_ARGS ${android_cmake_args})
  
  dep_build(common_interfaces CMAKE
    SOURCE_DIR "deps/common_interfaces/common_interfaces"
    DEPENDENCIES ament_cmake
    CMAKE_ARGS ${android_cmake_args})
  
  dep_build(domain_coordinator PIP
    SOURCE_DIR "deps/ament_cmake_ros/domain_coordinator"
    DEPENDENCIES )
  
  dep_build(ament_cmake_ros CMAKE
    SOURCE_DIR "deps/ament_cmake_ros/ament_cmake_ros"
    DEPENDENCIES domain_coordinator ament_cmake
    CMAKE_ARGS ${android_cmake_args})
  
  #dep_build(iceoryx_hoofs CMAKE
  #  SOURCE_DIR "deps/iceoryx/iceoryx_hoofs"
  #  DEPENDENCIES libatomic cmake acl
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(iceoryx_posh CMAKE
  #  SOURCE_DIR "deps/iceoryx/iceoryx_posh"
  #  DEPENDENCIES git cmake iceoryx_hoofs
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(iceoryx_binding_c CMAKE
  #  SOURCE_DIR "deps/iceoryx/iceoryx_binding_c"
  #  DEPENDENCIES cmake iceoryx_hoofs iceoryx_posh
  #  CMAKE_ARGS ${android_cmake_args})
  
  dep_build(cyclonedds CMAKE
    SOURCE_DIR "deps/cyclonedds"
    DEPENDENCIES iceoryx_binding_c bison iceoryx_utils iceoryx_posh cmake libssl-dev
    CMAKE_ARGS
      ${android_cmake_args}
      # allow finding native ddsconf tool
      "-DCMAKE_PREFIX_PATH=${CMAKE_CURRENT_BINARY_DIR}/deps/cyclonedds-native"
      # TODO(sloretz) is SSL required for sros2? if so, figure out how to enable
      "-DENABLE_SSL=OFF")
  
  dep_build(rcutils CMAKE
    SOURCE_DIR "deps/rcutils"
    DEPENDENCIES libatomic python3-empy ament_cmake_ros
    CMAKE_ARGS ${android_cmake_args})
  
  dep_build(rcpputils CMAKE
    SOURCE_DIR "deps/rcpputils"
    DEPENDENCIES ament_cmake_ros ament_cmake rcutils
    CMAKE_ARGS ${android_cmake_args})
  
  #dep_build(libyaml_vendor CMAKE
  #  SOURCE_DIR "deps/libyaml_vendor"
  #  DEPENDENCIES git ament_cmake
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(rcl_logging_interface CMAKE
  #  SOURCE_DIR "deps/rcl_logging/rcl_logging_interface"
  #  DEPENDENCIES ament_cmake_ros rcutils
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(rcl_logging_noop CMAKE
  #  SOURCE_DIR "deps/rcl_logging/rcl_logging_noop"
  #  DEPENDENCIES python3-empy ament_cmake_ros rcutils rcl_logging_interface
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(rmw_implementation_cmake CMAKE
  #  SOURCE_DIR "deps/rmw/rmw_implementation_cmake"
  #  DEPENDENCIES ament_cmake
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(rosidl_adapter CMAKE
  #  SOURCE_DIR "deps/rosidl/rosidl_adapter"
  #  DEPENDENCIES ament_cmake
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(rosidl_cli PIP
  #  SOURCE_DIR "deps/rosidl/rosidl_cli"
  #  DEPENDENCIES )
  #
  #dep_build(rosidl_cmake CMAKE
  #  SOURCE_DIR "deps/rosidl/rosidl_cmake"
  #  DEPENDENCIES ament_cmake_python ament_cmake
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(rosidl_parser CMAKE
  #  SOURCE_DIR "deps/rosidl/rosidl_parser"
  #  DEPENDENCIES ament_cmake
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(rosidl_typesupport_interface CMAKE
  #  SOURCE_DIR "deps/rosidl/rosidl_typesupport_interface"
  #  DEPENDENCIES ament_cmake
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(rosidl_runtime_c CMAKE
  #  SOURCE_DIR "deps/rosidl/rosidl_runtime_c"
  #  DEPENDENCIES rosidl_typesupport_interface ament_cmake_ros rcutils
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(rmw CMAKE
  #  SOURCE_DIR "deps/rmw/rmw"
  #  DEPENDENCIES ament_cmake_ros rosidl_runtime_c ament_cmake_version rcutils
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(rcl_yaml_param_parser CMAKE
  #  SOURCE_DIR "deps/rcl/rcl_yaml_param_parser"
  #  DEPENDENCIES yaml libyaml_vendor ament_cmake_ros rmw rcutils
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(rosidl_generator_c CMAKE
  #  SOURCE_DIR "deps/rosidl/rosidl_generator_c"
  #  DEPENDENCIES ament_cmake_python ament_cmake_ros
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(rosidl_runtime_cpp CMAKE
  #  SOURCE_DIR "deps/rosidl/rosidl_runtime_cpp"
  #  DEPENDENCIES rosidl_runtime_c ament_cmake
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(rosidl_generator_cpp CMAKE
  #  SOURCE_DIR "deps/rosidl/rosidl_generator_cpp"
  #  DEPENDENCIES ament_cmake
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(rosidl_typesupport_introspection_c CMAKE
  #  SOURCE_DIR "deps/rosidl/rosidl_typesupport_introspection_c"
  #  DEPENDENCIES ament_cmake_ros
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(rosidl_typesupport_c CMAKE
  #  SOURCE_DIR "deps/rosidl_typesupport/rosidl_typesupport_c"
  #  DEPENDENCIES rosidl_typesupport_introspection_c rcpputils ament_cmake_ros rosidl_runtime_c rcutils
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(rosidl_typesupport_introspection_cpp CMAKE
  #  SOURCE_DIR "deps/rosidl/rosidl_typesupport_introspection_cpp"
  #  DEPENDENCIES ament_cmake_ros rosidl_typesupport_introspection_c rosidl_runtime_cpp
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(rosidl_default_generators CMAKE
  #  SOURCE_DIR "deps/rosidl_defaults/rosidl_default_generators"
  #  DEPENDENCIES ament_cmake
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(builtin_interfaces CMAKE
  #  SOURCE_DIR "deps/rcl_interfaces/builtin_interfaces"
  #  DEPENDENCIES rosidl_default_generators ament_cmake
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(lifecycle_msgs CMAKE
  #  SOURCE_DIR "deps/rcl_interfaces/lifecycle_msgs"
  #  DEPENDENCIES rosidl_default_generators ament_cmake
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(rmw_dds_common CMAKE
  #  SOURCE_DIR "deps/rmw_dds_common/rmw_dds_common"
  #  DEPENDENCIES rcpputils rosidl_runtime_cpp rosidl_default_generators rmw ament_cmake rcutils
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(rosidl_typesupport_cpp CMAKE
  #  SOURCE_DIR "deps/rosidl_typesupport/rosidl_typesupport_cpp"
  #  DEPENDENCIES rcpputils ament_cmake_ros rosidl_typesupport_introspection_cpp rosidl_typesupport_c rosidl_runtime_c rcutils
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(rosidl_default_runtime CMAKE
  #  SOURCE_DIR "deps/rosidl_defaults/rosidl_default_runtime"
  #  DEPENDENCIES ament_cmake
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(action_msgs CMAKE
  #  SOURCE_DIR "deps/rcl_interfaces/action_msgs"
  #  DEPENDENCIES builtin_interfaces rosidl_default_generators ament_cmake unique_identifier_msgs
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(rcl_interfaces CMAKE
  #  SOURCE_DIR "deps/rcl_interfaces/rcl_interfaces"
  #  DEPENDENCIES builtin_interfaces rosidl_default_generators ament_cmake
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(composition_interfaces CMAKE
  #  SOURCE_DIR "deps/rcl_interfaces/composition_interfaces"
  #  DEPENDENCIES rcl_interfaces rosidl_default_generators ament_cmake
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(rosgraph_msgs CMAKE
  #  SOURCE_DIR "deps/rcl_interfaces/rosgraph_msgs"
  #  DEPENDENCIES builtin_interfaces rosidl_default_generators ament_cmake
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(rosidl_typesupport_introspection_tests CMAKE
  #  SOURCE_DIR "deps/rosidl/rosidl_typesupport_introspection_tests"
  #  DEPENDENCIES ament_cmake
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(sensor_msgs_py PIP
  #  SOURCE_DIR "deps/common_interfaces/sensor_msgs_py"
  #  DEPENDENCIES )
  #
  #dep_build(spdlog_vendor CMAKE
  #  SOURCE_DIR "deps/spdlog_vendor"
  #  DEPENDENCIES spdlog ament_cmake
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(rcl_logging_spdlog CMAKE
  #  SOURCE_DIR "deps/rcl_logging/rcl_logging_spdlog"
  #  DEPENDENCIES spdlog_vendor rcl_logging_interface rcpputils ament_cmake_ros spdlog rcutils
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(statistics_msgs CMAKE
  #  SOURCE_DIR "deps/rcl_interfaces/statistics_msgs"
  #  DEPENDENCIES builtin_interfaces rosidl_default_generators ament_cmake
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(std_msgs CMAKE
  #  SOURCE_DIR "deps/common_interfaces/std_msgs"
  #  DEPENDENCIES builtin_interfaces rosidl_default_generators ament_cmake
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(actionlib_msgs CMAKE
  #  SOURCE_DIR "deps/common_interfaces/actionlib_msgs"
  #  DEPENDENCIES std_msgs builtin_interfaces rosidl_default_generators ament_cmake
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(geometry_msgs CMAKE
  #  SOURCE_DIR "deps/common_interfaces/geometry_msgs"
  #  DEPENDENCIES std_msgs rosidl_default_generators ament_cmake
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(diagnostic_msgs CMAKE
  #  SOURCE_DIR "deps/common_interfaces/diagnostic_msgs"
  #  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs rosidl_default_generators ament_cmake
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(nav_msgs CMAKE
  #  SOURCE_DIR "deps/common_interfaces/nav_msgs"
  #  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs rosidl_default_generators ament_cmake
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(sensor_msgs CMAKE
  #  SOURCE_DIR "deps/common_interfaces/sensor_msgs"
  #  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs rosidl_default_generators ament_cmake
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(shape_msgs CMAKE
  #  SOURCE_DIR "deps/common_interfaces/shape_msgs"
  #  DEPENDENCIES geometry_msgs rosidl_default_generators ament_cmake
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(std_srvs CMAKE
  #  SOURCE_DIR "deps/common_interfaces/std_srvs"
  #  DEPENDENCIES rosidl_default_generators ament_cmake
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(stereo_msgs CMAKE
  #  SOURCE_DIR "deps/common_interfaces/stereo_msgs"
  #  DEPENDENCIES std_msgs rosidl_default_generators sensor_msgs ament_cmake
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(test_msgs CMAKE
  #  SOURCE_DIR "deps/rcl_interfaces/test_msgs"
  #  DEPENDENCIES builtin_interfaces action_msgs rosidl_default_generators ament_cmake test_interface_files
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(tracetools CMAKE
  #  SOURCE_DIR "deps/ros2_tracing/tracetools"
  #  DEPENDENCIES pkg-config ament_cmake_ros
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(rmw_cyclonedds_cpp CMAKE
  #  SOURCE_DIR "deps/rmw_cyclonedds/rmw_cyclonedds_cpp"
  #  DEPENDENCIES rmw_dds_common rosidl_typesupport_introspection_c tracetools rcpputils iceoryx_binding_c ament_cmake_ros cyclonedds rosidl_typesupport_introspection_cpp rosidl_runtime_c rmw rcutils
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(rmw_implementation CMAKE
  #  SOURCE_DIR "deps/rmw_implementation/rmw_implementation"
  #  DEPENDENCIES ament_index_cpp rmw_connextdds rcpputils rmw_fastrtps_dynamic_cpp rmw_cyclonedds_cpp rmw_implementation_cmake rmw_fastrtps_cpp rmw ament_cmake rcutils
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(rcl CMAKE
  #  SOURCE_DIR "deps/rcl/rcl"
  #  DEPENDENCIES rcl_yaml_param_parser rcl_logging_interface rmw_implementation rcl_interfaces ament_cmake_ros tracetools rosidl_runtime_c rcl_logging_spdlog rcutils
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(libstatistics_collector CMAKE
  #  SOURCE_DIR "deps/libstatistics_collector"
  #  DEPENDENCIES rcpputils ament_cmake_ros rcl std_msgs rosidl_default_generators ament_cmake statistics_msgs
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(rcl_action CMAKE
  #  SOURCE_DIR "deps/rcl/rcl_action"
  #  DEPENDENCIES ament_cmake_ros rcl action_msgs rosidl_runtime_c rmw rcutils
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(rcl_lifecycle CMAKE
  #  SOURCE_DIR "deps/rcl/rcl_lifecycle"
  #  DEPENDENCIES lifecycle_msgs ament_cmake_ros rcl tracetools rosidl_runtime_c rmw rcutils
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(rclcpp CMAKE
  #  SOURCE_DIR "deps/rclcpp/rclcpp"
  #  DEPENDENCIES rcl_yaml_param_parser builtin_interfaces ament_index_cpp rcpputils rcl_interfaces ament_cmake_ros rcl python3 rosgraph_msgs rosidl_runtime_cpp statistics_msgs libstatistics_collector tracetools rosidl_typesupport_cpp rosidl_typesupport_c ament_cmake_gen_version_h rmw rcutils
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(rclcpp_action CMAKE
  #  SOURCE_DIR "deps/rclcpp/rclcpp_action"
  #  DEPENDENCIES rclcpp rcpputils ament_cmake_ros action_msgs rcl_action rosidl_runtime_c ament_cmake
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(rclcpp_components CMAKE
  #  SOURCE_DIR "deps/rclcpp/rclcpp_components"
  #  DEPENDENCIES rclcpp ament_index_cpp rcpputils ament_cmake_ros class_loader composition_interfaces
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(rclcpp_lifecycle CMAKE
  #  SOURCE_DIR "deps/rclcpp/rclcpp_lifecycle"
  #  DEPENDENCIES rclcpp lifecycle_msgs rcl_lifecycle ament_cmake_ros rosidl_typesupport_cpp rmw
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(test_rmw_implementation CMAKE
  #  SOURCE_DIR "deps/rmw_implementation/test_rmw_implementation"
  #  DEPENDENCIES ament_cmake
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(tracetools_read PIP
  #  SOURCE_DIR "deps/ros2_tracing/tracetools_read"
  #  DEPENDENCIES )
  #
  #dep_build(tracetools_trace PIP
  #  SOURCE_DIR "deps/ros2_tracing/tracetools_trace"
  #  DEPENDENCIES )
  #
  #dep_build(ros2trace PIP
  #  SOURCE_DIR "deps/ros2_tracing/ros2trace"
  #  DEPENDENCIES ros2cli tracetools_trace)
  #
  #dep_build(tracetools_launch PIP
  #  SOURCE_DIR "deps/ros2_tracing/tracetools_launch"
  #  DEPENDENCIES launch tracetools_trace launch_ros)
  #
  #dep_build(tracetools_test PIP
  #  SOURCE_DIR "deps/ros2_tracing/tracetools_test"
  #  DEPENDENCIES launch tracetools_read tracetools_trace launch_ros tracetools_launch)
  #
  #dep_build(test_tracetools CMAKE
  #  SOURCE_DIR "deps/ros2_tracing/test_tracetools"
  #  DEPENDENCIES lifecycle_msgs rclcpp rclcpp_lifecycle pkg-config std_msgs std_srvs ament_cmake
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(test_tracetools_launch PIP
  #  SOURCE_DIR "deps/ros2_tracing/test_tracetools_launch"
  #  DEPENDENCIES )
  #
  #dep_build(trajectory_msgs CMAKE
  #  SOURCE_DIR "deps/common_interfaces/trajectory_msgs"
  #  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs rosidl_default_generators ament_cmake
  #  CMAKE_ARGS ${android_cmake_args})
  #
  #dep_build(visualization_msgs CMAKE
  #  SOURCE_DIR "deps/common_interfaces/visualization_msgs"
  #  DEPENDENCIES builtin_interfaces geometry_msgs sensor_msgs std_msgs rosidl_default_generators ament_cmake
  #  CMAKE_ARGS ${android_cmake_args})
endmacro()
