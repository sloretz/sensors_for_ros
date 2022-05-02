include(dep_build.cmake)

macro(build_native_dependencies)
  dep_build(cyclonedds-native CMAKE
    SOURCE_DIR "deps/cyclonedds"
    DEPENDENCIES libssl-dev bison
    CMAKE_ARGS "-DBUILD_DDSCONF=ON")
endmacro()

macro(build_crosscompile_dependencies)
  dep_build(ament_index_python PIP
    SOURCE_DIR "deps/ament_index/ament_index_python"
    DEPENDENCIES )
  
  dep_build(ament_package PIP
    SOURCE_DIR "deps/ament_package"
    DEPENDENCIES python3-importlib-metadata python3-importlib-resources python3-setuptools python3-importlib-metadata python3-importlib-resources python3-setuptools)
  
  dep_build(ament_cmake_core CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_core"
    DEPENDENCIES cmake ament_package python3-catkin-pkg-modules
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
    DEPENDENCIES cmake ament_cmake_core ament_cmake_export_dependencies ament_cmake_core ament_cmake_export_dependencies
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
    DEPENDENCIES ament_cmake_core ament_cmake_export_dependencies ament_cmake_python
    CMAKE_ARGS ${android_cmake_args})
  
  dep_build(ament_cmake_target_dependencies CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_target_dependencies"
    DEPENDENCIES ament_cmake_core
    CMAKE_ARGS ${android_cmake_args})
  
  dep_build(ament_cmake_test CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_test"
    DEPENDENCIES ament_cmake_core ament_cmake_python
    CMAKE_ARGS ${android_cmake_args})
  
  dep_build(ament_cmake_version CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_version"
    DEPENDENCIES ament_cmake_core
    CMAKE_ARGS ${android_cmake_args})
  
  dep_build(ament_cmake_auto CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_auto"
    DEPENDENCIES ament_cmake ament_cmake_gtest
    CMAKE_ARGS ${android_cmake_args})
  
  dep_build(ament_index_cpp CMAKE
    SOURCE_DIR "deps/ament_index/ament_index_cpp"
    DEPENDENCIES ament_cmake
    CMAKE_ARGS ${android_cmake_args})
  
  #  dep_build(common_interfaces CMAKE
  #    SOURCE_DIR "deps/common_interfaces/common_interfaces"
  #    DEPENDENCIES ament_cmake
  #    CMAKE_ARGS ${android_cmake_args})
  
  dep_build(domain_coordinator PIP
    SOURCE_DIR "deps/ament_cmake_ros/domain_coordinator"
    DEPENDENCIES )
  
  dep_build(ament_cmake_ros CMAKE
    SOURCE_DIR "deps/ament_cmake_ros/ament_cmake_ros"
    DEPENDENCIES ament_cmake domain_coordinator domain_coordinator
    CMAKE_ARGS ${android_cmake_args})
  
  #  dep_build(iceoryx_hoofs CMAKE
  #    SOURCE_DIR "deps/iceoryx/iceoryx_hoofs"
  #    DEPENDENCIES cmake acl libatomic acl libatomic
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(iceoryx_posh CMAKE
  #    SOURCE_DIR "deps/iceoryx/iceoryx_posh"
  #    DEPENDENCIES cmake git iceoryx_hoofs iceoryx_hoofs
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(iceoryx_binding_c CMAKE
  #    SOURCE_DIR "deps/iceoryx/iceoryx_binding_c"
  #    DEPENDENCIES cmake iceoryx_posh iceoryx_hoofs iceoryx_posh iceoryx_hoofs
  #    CMAKE_ARGS ${android_cmake_args})
  
  dep_build(cyclonedds CMAKE
    SOURCE_DIR "deps/cyclonedds"
    DEPENDENCIES cmake libssl-dev bison iceoryx_binding_c iceoryx_posh iceoryx_utils libssl-dev bison iceoryx_binding_c iceoryx_posh iceoryx_utils cyclonedds-native
    CMAKE_ARGS ${android_cmake_args})
  
  dep_build(rcutils CMAKE
    SOURCE_DIR "deps/rcutils"
    DEPENDENCIES ament_cmake_ros python3-empy libatomic libatomic
    CMAKE_ARGS ${android_cmake_args})
  
  dep_build(rcpputils CMAKE
    SOURCE_DIR "deps/rcpputils"
    DEPENDENCIES ament_cmake ament_cmake_ros rcutils rcutils
    CMAKE_ARGS ${android_cmake_args})
  
  #  dep_build(libyaml_vendor CMAKE
  #    SOURCE_DIR "deps/libyaml_vendor"
  #    DEPENDENCIES ament_cmake git
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(rcl_logging_interface CMAKE
  #    SOURCE_DIR "deps/rcl_logging/rcl_logging_interface"
  #    DEPENDENCIES ament_cmake_ros rcutils rcutils
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(rcl_logging_noop CMAKE
  #    SOURCE_DIR "deps/rcl_logging/rcl_logging_noop"
  #    DEPENDENCIES ament_cmake_ros python3-empy rcl_logging_interface rcutils rcl_logging_interface rcutils
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(rmw_implementation_cmake CMAKE
  #    SOURCE_DIR "deps/rmw/rmw_implementation_cmake"
  #    DEPENDENCIES ament_cmake
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(rosidl_adapter CMAKE
  #    SOURCE_DIR "deps/rosidl/rosidl_adapter"
  #    DEPENDENCIES ament_cmake ament_cmake
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(rosidl_cli PIP
  #    SOURCE_DIR "deps/rosidl/rosidl_cli"
  #    DEPENDENCIES )
  #  
  #  dep_build(rosidl_cmake CMAKE
  #    SOURCE_DIR "deps/rosidl/rosidl_cmake"
  #    DEPENDENCIES ament_cmake ament_cmake_python
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(rosidl_parser CMAKE
  #    SOURCE_DIR "deps/rosidl/rosidl_parser"
  #    DEPENDENCIES ament_cmake
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(rosidl_typesupport_interface CMAKE
  #    SOURCE_DIR "deps/rosidl/rosidl_typesupport_interface"
  #    DEPENDENCIES ament_cmake
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(rosidl_runtime_c CMAKE
  #    SOURCE_DIR "deps/rosidl/rosidl_runtime_c"
  #    DEPENDENCIES ament_cmake_ros rosidl_typesupport_interface rcutils rosidl_typesupport_interface rcutils
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(rmw CMAKE
  #    SOURCE_DIR "deps/rmw/rmw"
  #    DEPENDENCIES ament_cmake_ros ament_cmake_version rcutils rosidl_runtime_c rcutils rosidl_runtime_c
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(rcl_yaml_param_parser CMAKE
  #    SOURCE_DIR "deps/rcl/rcl_yaml_param_parser"
  #    DEPENDENCIES ament_cmake_ros rcutils libyaml_vendor yaml rmw rcutils libyaml_vendor yaml rmw
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(rosidl_generator_c CMAKE
  #    SOURCE_DIR "deps/rosidl/rosidl_generator_c"
  #    DEPENDENCIES ament_cmake_python ament_cmake_ros
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(rosidl_runtime_cpp CMAKE
  #    SOURCE_DIR "deps/rosidl/rosidl_runtime_cpp"
  #    DEPENDENCIES ament_cmake rosidl_runtime_c rosidl_runtime_c
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(rosidl_generator_cpp CMAKE
  #    SOURCE_DIR "deps/rosidl/rosidl_generator_cpp"
  #    DEPENDENCIES ament_cmake
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(rosidl_typesupport_introspection_c CMAKE
  #    SOURCE_DIR "deps/rosidl/rosidl_typesupport_introspection_c"
  #    DEPENDENCIES ament_cmake_ros
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(rosidl_typesupport_c CMAKE
  #    SOURCE_DIR "deps/rosidl_typesupport/rosidl_typesupport_c"
  #    DEPENDENCIES ament_cmake_ros rosidl_runtime_c rosidl_typesupport_introspection_c rcpputils rcutils rosidl_runtime_c rosidl_typesupport_introspection_c rcpputils rcutils
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(rosidl_typesupport_introspection_cpp CMAKE
  #    SOURCE_DIR "deps/rosidl/rosidl_typesupport_introspection_cpp"
  #    DEPENDENCIES ament_cmake_ros rosidl_runtime_cpp rosidl_typesupport_introspection_c rosidl_runtime_cpp rosidl_typesupport_introspection_c
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(rosidl_default_generators CMAKE
  #    SOURCE_DIR "deps/rosidl_defaults/rosidl_default_generators"
  #    DEPENDENCIES ament_cmake
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(builtin_interfaces CMAKE
  #    SOURCE_DIR "deps/rcl_interfaces/builtin_interfaces"
  #    DEPENDENCIES ament_cmake rosidl_default_generators
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(lifecycle_msgs CMAKE
  #    SOURCE_DIR "deps/rcl_interfaces/lifecycle_msgs"
  #    DEPENDENCIES ament_cmake rosidl_default_generators
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(rmw_dds_common CMAKE
  #    SOURCE_DIR "deps/rmw_dds_common/rmw_dds_common"
  #    DEPENDENCIES ament_cmake rosidl_default_generators rcutils rcpputils rmw rosidl_runtime_cpp rcutils rcpputils rmw rosidl_runtime_cpp
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(rosidl_typesupport_cpp CMAKE
  #    SOURCE_DIR "deps/rosidl_typesupport/rosidl_typesupport_cpp"
  #    DEPENDENCIES ament_cmake_ros rosidl_runtime_c rosidl_typesupport_c rosidl_typesupport_introspection_cpp rcutils rcpputils rosidl_runtime_c rosidl_typesupport_c rosidl_typesupport_introspection_cpp rcutils rcpputils
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(rosidl_default_runtime CMAKE
  #    SOURCE_DIR "deps/rosidl_defaults/rosidl_default_runtime"
  #    DEPENDENCIES ament_cmake
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(action_msgs CMAKE
  #    SOURCE_DIR "deps/rcl_interfaces/action_msgs"
  #    DEPENDENCIES ament_cmake rosidl_default_generators builtin_interfaces unique_identifier_msgs builtin_interfaces unique_identifier_msgs
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(rcl_interfaces CMAKE
  #    SOURCE_DIR "deps/rcl_interfaces/rcl_interfaces"
  #    DEPENDENCIES ament_cmake rosidl_default_generators builtin_interfaces builtin_interfaces
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(composition_interfaces CMAKE
  #    SOURCE_DIR "deps/rcl_interfaces/composition_interfaces"
  #    DEPENDENCIES ament_cmake rosidl_default_generators rcl_interfaces rcl_interfaces
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(rosgraph_msgs CMAKE
  #    SOURCE_DIR "deps/rcl_interfaces/rosgraph_msgs"
  #    DEPENDENCIES ament_cmake rosidl_default_generators builtin_interfaces builtin_interfaces
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(rosidl_typesupport_introspection_tests CMAKE
  #    SOURCE_DIR "deps/rosidl/rosidl_typesupport_introspection_tests"
  #    DEPENDENCIES ament_cmake
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(sensor_msgs_py PIP
  #    SOURCE_DIR "deps/common_interfaces/sensor_msgs_py"
  #    DEPENDENCIES )
  #  
  #  dep_build(spdlog_vendor CMAKE
  #    SOURCE_DIR "deps/spdlog_vendor"
  #    DEPENDENCIES ament_cmake spdlog spdlog
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(rcl_logging_spdlog CMAKE
  #    SOURCE_DIR "deps/rcl_logging/rcl_logging_spdlog"
  #    DEPENDENCIES ament_cmake_ros spdlog_vendor spdlog rcl_logging_interface rcpputils rcutils spdlog_vendor spdlog rcl_logging_interface rcpputils rcutils
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(statistics_msgs CMAKE
  #    SOURCE_DIR "deps/rcl_interfaces/statistics_msgs"
  #    DEPENDENCIES ament_cmake rosidl_default_generators builtin_interfaces builtin_interfaces
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(std_msgs CMAKE
  #    SOURCE_DIR "deps/common_interfaces/std_msgs"
  #    DEPENDENCIES ament_cmake rosidl_default_generators builtin_interfaces builtin_interfaces
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(actionlib_msgs CMAKE
  #    SOURCE_DIR "deps/common_interfaces/actionlib_msgs"
  #    DEPENDENCIES ament_cmake rosidl_default_generators builtin_interfaces std_msgs builtin_interfaces std_msgs
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(geometry_msgs CMAKE
  #    SOURCE_DIR "deps/common_interfaces/geometry_msgs"
  #    DEPENDENCIES ament_cmake rosidl_default_generators std_msgs std_msgs
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(diagnostic_msgs CMAKE
  #    SOURCE_DIR "deps/common_interfaces/diagnostic_msgs"
  #    DEPENDENCIES ament_cmake rosidl_default_generators builtin_interfaces geometry_msgs std_msgs builtin_interfaces geometry_msgs std_msgs
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(nav_msgs CMAKE
  #    SOURCE_DIR "deps/common_interfaces/nav_msgs"
  #    DEPENDENCIES ament_cmake rosidl_default_generators builtin_interfaces geometry_msgs std_msgs builtin_interfaces geometry_msgs std_msgs
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(sensor_msgs CMAKE
  #    SOURCE_DIR "deps/common_interfaces/sensor_msgs"
  #    DEPENDENCIES ament_cmake rosidl_default_generators builtin_interfaces geometry_msgs std_msgs builtin_interfaces geometry_msgs std_msgs
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(shape_msgs CMAKE
  #    SOURCE_DIR "deps/common_interfaces/shape_msgs"
  #    DEPENDENCIES ament_cmake rosidl_default_generators geometry_msgs geometry_msgs
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(std_srvs CMAKE
  #    SOURCE_DIR "deps/common_interfaces/std_srvs"
  #    DEPENDENCIES ament_cmake rosidl_default_generators
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(stereo_msgs CMAKE
  #    SOURCE_DIR "deps/common_interfaces/stereo_msgs"
  #    DEPENDENCIES ament_cmake rosidl_default_generators sensor_msgs std_msgs sensor_msgs std_msgs
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(test_msgs CMAKE
  #    SOURCE_DIR "deps/rcl_interfaces/test_msgs"
  #    DEPENDENCIES ament_cmake rosidl_default_generators builtin_interfaces test_interface_files action_msgs builtin_interfaces test_interface_files action_msgs
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(tracetools CMAKE
  #    SOURCE_DIR "deps/ros2_tracing/tracetools"
  #    DEPENDENCIES ament_cmake_ros pkg-config
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(rmw_cyclonedds_cpp CMAKE
  #    SOURCE_DIR "deps/rmw_cyclonedds/rmw_cyclonedds_cpp"
  #    DEPENDENCIES ament_cmake_ros cyclonedds iceoryx_binding_c rcutils rcpputils rmw rmw_dds_common rosidl_runtime_c rosidl_typesupport_introspection_c rosidl_typesupport_introspection_cpp tracetools cyclonedds iceoryx_binding_c rcutils rcpputils rmw rmw_dds_common rosidl_runtime_c rosidl_typesupport_introspection_c rosidl_typesupport_introspection_cpp tracetools
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(rmw_implementation CMAKE
  #    SOURCE_DIR "deps/rmw_implementation/rmw_implementation"
  #    DEPENDENCIES ament_cmake rmw_implementation_cmake rmw rmw_connextdds rmw_cyclonedds_cpp rmw_fastrtps_cpp rmw_fastrtps_dynamic_cpp ament_index_cpp rcpputils rcutils rmw_implementation_cmake rmw rmw_connextdds rmw_cyclonedds_cpp rmw_fastrtps_cpp rmw_fastrtps_dynamic_cpp ament_index_cpp rcpputils rcutils rmw_implementation_cmake
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(rcl CMAKE
  #    SOURCE_DIR "deps/rcl/rcl"
  #    DEPENDENCIES ament_cmake_ros rcl_interfaces rcl_logging_interface rcl_logging_spdlog rcl_yaml_param_parser rcutils rmw_implementation rosidl_runtime_c tracetools rcl_interfaces rcl_logging_interface rcl_logging_spdlog rcl_yaml_param_parser rcutils rmw_implementation rosidl_runtime_c tracetools
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(libstatistics_collector CMAKE
  #    SOURCE_DIR "deps/libstatistics_collector"
  #    DEPENDENCIES ament_cmake ament_cmake_ros rosidl_default_generators std_msgs rcl rcpputils statistics_msgs rosidl_default_generators std_msgs rcl rcpputils statistics_msgs
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(rcl_action CMAKE
  #    SOURCE_DIR "deps/rcl/rcl_action"
  #    DEPENDENCIES ament_cmake_ros action_msgs rcl rcutils rmw rosidl_runtime_c action_msgs rcl rcutils rmw rosidl_runtime_c
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(rcl_lifecycle CMAKE
  #    SOURCE_DIR "deps/rcl/rcl_lifecycle"
  #    DEPENDENCIES ament_cmake_ros lifecycle_msgs rcl rcutils rmw rosidl_runtime_c tracetools lifecycle_msgs rcl rcutils rmw rosidl_runtime_c tracetools
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(rclcpp CMAKE
  #    SOURCE_DIR "deps/rclcpp/rclcpp"
  #    DEPENDENCIES ament_cmake_ros ament_cmake_gen_version_h python3 ament_index_cpp builtin_interfaces rcl_interfaces rosgraph_msgs rosidl_runtime_cpp rosidl_typesupport_c rosidl_typesupport_cpp libstatistics_collector rcl rcl_yaml_param_parser rcpputils rcutils rmw statistics_msgs tracetools ament_index_cpp builtin_interfaces rcl_interfaces rosgraph_msgs rosidl_runtime_cpp rosidl_typesupport_c rosidl_typesupport_cpp libstatistics_collector rcl rcl_yaml_param_parser rcpputils rcutils rmw statistics_msgs tracetools
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(rclcpp_action CMAKE
  #    SOURCE_DIR "deps/rclcpp/rclcpp_action"
  #    DEPENDENCIES ament_cmake_ros rosidl_runtime_c action_msgs rclcpp rcl_action rcpputils ament_cmake rosidl_runtime_c action_msgs rclcpp rcl_action rcpputils ament_cmake
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(rclcpp_components CMAKE
  #    SOURCE_DIR "deps/rclcpp/rclcpp_components"
  #    DEPENDENCIES ament_cmake_ros ament_index_cpp class_loader composition_interfaces rclcpp rcpputils ament_index_cpp class_loader composition_interfaces rclcpp rcpputils
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(rclcpp_lifecycle CMAKE
  #    SOURCE_DIR "deps/rclcpp/rclcpp_lifecycle"
  #    DEPENDENCIES ament_cmake_ros lifecycle_msgs rclcpp rcl_lifecycle rosidl_typesupport_cpp rmw lifecycle_msgs rclcpp rcl_lifecycle rosidl_typesupport_cpp rmw
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(test_rmw_implementation CMAKE
  #    SOURCE_DIR "deps/rmw_implementation/test_rmw_implementation"
  #    DEPENDENCIES ament_cmake
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(tracetools_read PIP
  #    SOURCE_DIR "deps/ros2_tracing/tracetools_read"
  #    DEPENDENCIES )
  #  
  #  dep_build(tracetools_trace PIP
  #    SOURCE_DIR "deps/ros2_tracing/tracetools_trace"
  #    DEPENDENCIES )
  #  
  #  dep_build(ros2trace PIP
  #    SOURCE_DIR "deps/ros2_tracing/ros2trace"
  #    DEPENDENCIES ros2cli tracetools_trace ros2cli tracetools_trace)
  #  
  #  dep_build(tracetools_launch PIP
  #    SOURCE_DIR "deps/ros2_tracing/tracetools_launch"
  #    DEPENDENCIES launch launch_ros tracetools_trace launch launch_ros tracetools_trace)
  #  
  #  dep_build(tracetools_test PIP
  #    SOURCE_DIR "deps/ros2_tracing/tracetools_test"
  #    DEPENDENCIES launch launch_ros tracetools_launch tracetools_read tracetools_trace launch launch_ros tracetools_launch tracetools_read tracetools_trace)
  #  
  #  dep_build(test_tracetools CMAKE
  #    SOURCE_DIR "deps/ros2_tracing/test_tracetools"
  #    DEPENDENCIES ament_cmake pkg-config lifecycle_msgs rclcpp rclcpp_lifecycle std_msgs std_srvs lifecycle_msgs rclcpp rclcpp_lifecycle std_msgs std_srvs
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(test_tracetools_launch PIP
  #    SOURCE_DIR "deps/ros2_tracing/test_tracetools_launch"
  #    DEPENDENCIES )
  #  
  #  dep_build(trajectory_msgs CMAKE
  #    SOURCE_DIR "deps/common_interfaces/trajectory_msgs"
  #    DEPENDENCIES ament_cmake rosidl_default_generators builtin_interfaces geometry_msgs std_msgs builtin_interfaces geometry_msgs std_msgs
  #    CMAKE_ARGS ${android_cmake_args})
  #  
  #  dep_build(visualization_msgs CMAKE
  #    SOURCE_DIR "deps/common_interfaces/visualization_msgs"
  #    DEPENDENCIES ament_cmake rosidl_default_generators builtin_interfaces geometry_msgs sensor_msgs std_msgs builtin_interfaces geometry_msgs sensor_msgs std_msgs
  #    CMAKE_ARGS ${android_cmake_args})
endmacro()
