include(dep_build.cmake)

macro(build_native_dependencies)
  dep_build(cyclonedds CMAKE
    NATIVE
    SOURCE_DIR "deps/cyclonedds"
    DEPENDENCIES libssl-dev bison
    CMAKE_ARGS "-DBUILD_DDSCONF=ON"
    # TODO(sloretz) is SSL required for sros2? if so, figure out how to enable
    "-DENABLE_SSL=OFF")
endmacro()

macro(build_crosscompile_dependencies)
  set(extra_cmake_args ${android_cmake_args})

  dep_build(ament_index_python PIP
    SOURCE_DIR "deps/ament_index/ament_index_python"
    DEPENDENCIES )
  
  dep_build(ament_package PIP
    SOURCE_DIR "deps/ament_package"
    DEPENDENCIES python3-setuptools python3-importlib-metadata python3-importlib-resources)
  
  dep_build(ament_cmake_core CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_core"
    DEPENDENCIES python3-catkin-pkg-modules python3-importlib-resources python3-importlib-metadata cmake python3-setuptools ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(ament_cmake_export_definitions CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_export_definitions"
    DEPENDENCIES python3-catkin-pkg-modules ament_cmake_core python3-importlib-resources python3-importlib-metadata cmake python3-setuptools ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(ament_cmake_export_dependencies CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_export_dependencies"
    DEPENDENCIES python3-catkin-pkg-modules ament_cmake_core python3-importlib-resources python3-importlib-metadata cmake python3-setuptools ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(ament_cmake CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake"
    DEPENDENCIES python3-catkin-pkg-modules ament_cmake_core python3-importlib-resources python3-importlib-metadata ament_cmake_libraries cmake ament_cmake_export_dependencies python3-setuptools ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(ament_cmake_export_include_directories CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_export_include_directories"
    DEPENDENCIES python3-catkin-pkg-modules ament_cmake_core python3-importlib-resources python3-importlib-metadata cmake python3-setuptools ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(ament_cmake_export_interfaces CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_export_interfaces"
    DEPENDENCIES python3-catkin-pkg-modules ament_cmake_core python3-importlib-resources python3-importlib-metadata cmake python3-setuptools ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(ament_cmake_export_libraries CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_export_libraries"
    DEPENDENCIES python3-catkin-pkg-modules ament_cmake_core python3-importlib-resources python3-importlib-metadata cmake python3-setuptools ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(ament_cmake_export_link_flags CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_export_link_flags"
    DEPENDENCIES python3-catkin-pkg-modules ament_cmake_core python3-importlib-resources python3-importlib-metadata cmake python3-setuptools ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(ament_cmake_export_targets CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_export_targets"
    DEPENDENCIES python3-catkin-pkg-modules ament_cmake_core python3-importlib-resources python3-importlib-metadata cmake python3-setuptools ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(ament_cmake_gmock CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_gmock"
    DEPENDENCIES python3-catkin-pkg-modules ament_cmake_core python3-importlib-resources python3-importlib-metadata cmake python3-setuptools ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(ament_cmake_gtest CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_gtest"
    DEPENDENCIES python3-catkin-pkg-modules ament_cmake_core python3-importlib-resources python3-importlib-metadata cmake python3-setuptools ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(ament_cmake_gen_version_h CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_gen_version_h"
    DEPENDENCIES python3-catkin-pkg-modules ament_cmake_core python3-importlib-resources python3-importlib-metadata cmake python3-setuptools ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(ament_cmake_include_directories CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_include_directories"
    DEPENDENCIES python3-catkin-pkg-modules ament_cmake_core python3-importlib-resources python3-importlib-metadata cmake python3-setuptools ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(ament_cmake_libraries CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_libraries"
    DEPENDENCIES python3-catkin-pkg-modules ament_cmake_core python3-importlib-resources python3-importlib-metadata cmake python3-setuptools ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(ament_cmake_nose CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_nose"
    DEPENDENCIES python3-catkin-pkg-modules ament_cmake_core python3-importlib-resources python3-importlib-metadata cmake python3-setuptools ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(ament_cmake_pytest CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_pytest"
    DEPENDENCIES python3-catkin-pkg-modules ament_cmake_core python3-importlib-resources python3-importlib-metadata cmake python3-setuptools ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(ament_cmake_python CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_python"
    DEPENDENCIES python3-catkin-pkg-modules ament_cmake_core python3-importlib-resources python3-importlib-metadata cmake python3-setuptools ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(ament_cmake_google_benchmark CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_google_benchmark"
    DEPENDENCIES python3-catkin-pkg-modules ament_cmake_core python3-importlib-resources python3-importlib-metadata ament_cmake_python ament_cmake_libraries cmake ament_cmake_export_dependencies python3-setuptools ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(ament_cmake_target_dependencies CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_target_dependencies"
    DEPENDENCIES python3-catkin-pkg-modules ament_cmake_core python3-importlib-resources python3-importlib-metadata cmake python3-setuptools ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(ament_cmake_test CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_test"
    DEPENDENCIES python3-catkin-pkg-modules ament_cmake_core python3-importlib-resources python3-importlib-metadata ament_cmake_python cmake python3-setuptools ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(ament_cmake_version CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_version"
    DEPENDENCIES python3-catkin-pkg-modules ament_cmake_core python3-importlib-resources python3-importlib-metadata cmake python3-setuptools ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(ament_cmake_auto CMAKE
    SOURCE_DIR "deps/ament_cmake/ament_cmake_auto"
    DEPENDENCIES python3-importlib-resources ament_cmake_export_include_directories gtest_vendor ament_cmake_libraries ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces gtest ament_cmake_export_libraries python3-catkin-pkg-modules ament_cmake_core ament_cmake_version ament_cmake_test ament_cmake_python python3-importlib-metadata cmake ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gtest ament_cmake_gen_version_h ament_cmake ament_cmake_export_definitions python3-setuptools ament_cmake_export_targets ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(ament_index_cpp CMAKE
    SOURCE_DIR "deps/ament_index/ament_index_cpp"
    DEPENDENCIES python3-importlib-resources ament_cmake_export_include_directories ament_cmake_libraries ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces ament_cmake_export_libraries python3-catkin-pkg-modules ament_cmake_core ament_cmake_version ament_cmake_test ament_cmake_python python3-importlib-metadata cmake ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h ament_cmake ament_cmake_export_definitions python3-setuptools ament_cmake_export_targets ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(common_interfaces CMAKE
    SOURCE_DIR "deps/common_interfaces/common_interfaces"
    DEPENDENCIES python3-importlib-resources ament_cmake_export_include_directories ament_cmake_libraries ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces ament_cmake_export_libraries python3-catkin-pkg-modules ament_cmake_core ament_cmake_version ament_cmake_test ament_cmake_python python3-importlib-metadata cmake ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h ament_cmake ament_cmake_export_definitions python3-setuptools ament_cmake_export_targets ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(domain_coordinator PIP
    SOURCE_DIR "deps/ament_cmake_ros/domain_coordinator"
    DEPENDENCIES )
  
  dep_build(ament_cmake_ros CMAKE
    SOURCE_DIR "deps/ament_cmake_ros/ament_cmake_ros"
    DEPENDENCIES python3-importlib-resources ament_cmake_export_include_directories ament_cmake_libraries ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces ament_cmake_export_libraries python3-catkin-pkg-modules ament_cmake_core ament_cmake_version ament_cmake_test ament_cmake_python python3-importlib-metadata cmake ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h domain_coordinator ament_cmake ament_cmake_export_definitions python3-setuptools ament_cmake_export_targets ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  #dep_build(iceoryx_hoofs CMAKE
  #  SOURCE_DIR "deps/iceoryx/iceoryx_hoofs"
  #  DEPENDENCIES acl cmake libatomic
  #  CMAKE_ARGS ${extra_cmake_args})
  #
  #dep_build(iceoryx_posh CMAKE
  #  SOURCE_DIR "deps/iceoryx/iceoryx_posh"
  #  DEPENDENCIES acl git iceoryx_hoofs cmake libatomic
  #  CMAKE_ARGS ${extra_cmake_args})
  #
  #dep_build(iceoryx_binding_c CMAKE
  #  SOURCE_DIR "deps/iceoryx/iceoryx_binding_c"
  #  DEPENDENCIES acl iceoryx_hoofs iceoryx_posh cmake libatomic
  #  CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(cyclonedds CMAKE
    SOURCE_DIR "deps/cyclonedds"
    DEPENDENCIES acl iceoryx_utils libssl-dev iceoryx_hoofs iceoryx_posh bison cmake libatomic iceoryx_binding_c
    native-cyclonedds
    CMAKE_ARGS
      ${extra_cmake_args}
      # allow finding native ddsconf tool
      "-DCMAKE_PREFIX_PATH=${CMAKE_CURRENT_BINARY_DIR}/native-deps/native-cyclonedds"
      # TODO(sloretz) is SSL required for sros2? if so, figure out how to enable
      "-DENABLE_SSL=OFF")
  
  dep_build(rcutils CMAKE
    SOURCE_DIR "deps/rcutils"
    DEPENDENCIES python3-pytest ament_cmake_pytest python3-importlib-resources ament_cmake_export_include_directories gtest_vendor ament_cmake_libraries ament_cmake_export_link_flags ament_cmake_include_directories libatomic ament_cmake_export_interfaces gtest ament_cmake_export_libraries python3-catkin-pkg-modules ament_cmake_core ament_cmake_version ament_cmake_test ament_cmake_python python3-importlib-metadata cmake ament_cmake_ros ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gtest ament_cmake_gen_version_h domain_coordinator gmock_vendor ament_cmake ament_cmake_gmock python3-empy google-mock ament_cmake_export_definitions python3-setuptools ament_cmake_export_targets ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(rcpputils CMAKE
    SOURCE_DIR "deps/rcpputils"
    DEPENDENCIES python3-pytest ament_cmake_pytest python3-importlib-resources ament_cmake_export_include_directories gtest_vendor ament_cmake_libraries ament_cmake_export_link_flags ament_cmake_include_directories libatomic ament_cmake_export_interfaces gtest ament_cmake_export_libraries python3-catkin-pkg-modules ament_cmake_core ament_cmake_version ament_cmake_test ament_cmake_python python3-importlib-metadata rcutils cmake ament_cmake_ros ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gtest ament_cmake_gen_version_h domain_coordinator gmock_vendor ament_cmake ament_cmake_gmock google-mock ament_cmake_export_definitions python3-setuptools ament_cmake_export_targets ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(libyaml_vendor CMAKE
    SOURCE_DIR "deps/libyaml_vendor"
    DEPENDENCIES git python3-importlib-resources ament_cmake_export_include_directories ament_cmake_libraries ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces ament_cmake_export_libraries python3-catkin-pkg-modules ament_cmake_core ament_cmake_version ament_cmake_test ament_cmake_python python3-importlib-metadata cmake ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h ament_cmake ament_cmake_export_definitions python3-setuptools ament_cmake_export_targets ament_package
    CMAKE_ARGS ${extra_cmake_args})

  # HACK yaml location
  # list(APPEND extra_cmake_args "-Dyaml_DIR=${CMAKE_CURRENT_BINARY_DIR}/deps/libyaml_vendor/cmake")

  dep_build(rcl_logging_interface CMAKE
    SOURCE_DIR "deps/rcl_logging/rcl_logging_interface"
    DEPENDENCIES python3-pytest ament_cmake_pytest python3-importlib-resources ament_cmake_export_include_directories gtest_vendor ament_cmake_libraries ament_cmake_export_link_flags ament_cmake_include_directories libatomic ament_cmake_export_interfaces gtest ament_cmake_export_libraries python3-catkin-pkg-modules ament_cmake_core ament_cmake_version ament_cmake_test ament_cmake_python python3-importlib-metadata rcutils cmake ament_cmake_ros ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gtest ament_cmake_gen_version_h domain_coordinator gmock_vendor ament_cmake ament_cmake_gmock google-mock ament_cmake_export_definitions python3-setuptools ament_cmake_export_targets ament_package
    CMAKE_ARGS ${extra_cmake_args})

  dep_build(rcl_logging_noop CMAKE
    SOURCE_DIR "deps/rcl_logging/rcl_logging_noop"
    DEPENDENCIES python3-pytest ament_cmake_pytest python3-importlib-resources ament_cmake_export_include_directories gtest_vendor ament_cmake_libraries ament_cmake_export_link_flags ament_cmake_include_directories libatomic ament_cmake_export_interfaces gtest ament_cmake_export_libraries python3-catkin-pkg-modules ament_cmake_core ament_cmake_version ament_cmake_test ament_cmake_python python3-importlib-metadata rcutils cmake ament_cmake_ros ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gtest ament_cmake_gen_version_h domain_coordinator gmock_vendor ament_cmake ament_cmake_gmock python3-empy google-mock ament_cmake_export_definitions python3-setuptools ament_cmake_export_targets rcl_logging_interface ament_package
    CMAKE_ARGS ${extra_cmake_args})

  dep_build(rcl_logging_rcutils CMAKE
    SOURCE_DIR "deps/rcl_logging/rcl_logging_rcutils"
    DEPENDENCIES python3-pytest ament_cmake_pytest python3-importlib-resources ament_cmake_export_include_directories gtest_vendor ament_cmake_libraries ament_cmake_export_link_flags ament_cmake_include_directories libatomic ament_cmake_export_interfaces gtest ament_cmake_export_libraries python3-catkin-pkg-modules ament_cmake_core ament_cmake_version ament_cmake_test ament_cmake_python python3-importlib-metadata rcutils cmake ament_cmake_ros ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gtest ament_cmake_gen_version_h domain_coordinator gmock_vendor ament_cmake ament_cmake_gmock python3-empy google-mock ament_cmake_export_definitions python3-setuptools ament_cmake_export_targets rcl_logging_interface ament_package
    CMAKE_ARGS ${extra_cmake_args})

  dep_build(rcl_logging_android CMAKE
    SOURCE_DIR "deps/rcl_logging_android"
    DEPENDENCIES python3-pytest ament_cmake_pytest python3-importlib-resources ament_cmake_export_include_directories gtest_vendor ament_cmake_libraries ament_cmake_export_link_flags ament_cmake_include_directories libatomic ament_cmake_export_interfaces gtest ament_cmake_export_libraries python3-catkin-pkg-modules ament_cmake_core ament_cmake_version ament_cmake_test ament_cmake_python python3-importlib-metadata rcutils cmake ament_cmake_ros ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gtest ament_cmake_gen_version_h domain_coordinator gmock_vendor ament_cmake ament_cmake_gmock python3-empy google-mock ament_cmake_export_definitions python3-setuptools ament_cmake_export_targets rcl_logging_interface ament_package
    CMAKE_ARGS ${extra_cmake_args})

  dep_build(rmw_implementation_cmake CMAKE
    SOURCE_DIR "deps/rmw/rmw_implementation_cmake"
    DEPENDENCIES python3-importlib-resources ament_cmake_export_include_directories ament_cmake_libraries ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces ament_cmake_export_libraries python3-catkin-pkg-modules ament_cmake_core ament_cmake_version ament_cmake_test ament_cmake_python python3-importlib-metadata cmake ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h ament_cmake ament_cmake_export_definitions python3-setuptools ament_cmake_export_targets ament_package
    CMAKE_ARGS ${extra_cmake_args})

  dep_build(rosidl_adapter CMAKE
    SOURCE_DIR "deps/rosidl/rosidl_adapter"
    DEPENDENCIES python3-importlib-resources ament_cmake_export_include_directories ament_cmake_libraries ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces ament_cmake_export_libraries python3-catkin-pkg-modules ament_cmake_core ament_cmake_version ament_cmake_test ament_cmake_python python3-importlib-metadata cmake ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h ament_cmake ament_cmake_export_definitions python3-setuptools ament_cmake_export_targets ament_package
    CMAKE_ARGS ${extra_cmake_args})

  dep_build(rosidl_cli PIP
    SOURCE_DIR "deps/rosidl/rosidl_cli"
    DEPENDENCIES )

  dep_build(rosidl_cmake CMAKE
    SOURCE_DIR "deps/rosidl/rosidl_cmake"
    DEPENDENCIES python3-importlib-resources ament_cmake_export_include_directories ament_cmake_libraries ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces ament_cmake_export_libraries python3-catkin-pkg-modules ament_cmake_core ament_cmake_version ament_cmake_test ament_cmake_python python3-importlib-metadata cmake ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h ament_cmake ament_cmake_export_definitions python3-setuptools ament_cmake_export_targets ament_package
    CMAKE_ARGS ${extra_cmake_args})

  dep_build(rosidl_parser CMAKE
    SOURCE_DIR "deps/rosidl/rosidl_parser"
    DEPENDENCIES python3-importlib-resources ament_cmake_export_include_directories ament_cmake_libraries ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces ament_cmake_export_libraries python3-catkin-pkg-modules ament_cmake_core ament_cmake_version ament_cmake_test ament_cmake_python python3-importlib-metadata cmake ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h ament_cmake ament_cmake_export_definitions python3-setuptools ament_cmake_export_targets ament_package
    CMAKE_ARGS ${extra_cmake_args})

  dep_build(rosidl_typesupport_interface CMAKE
    SOURCE_DIR "deps/rosidl/rosidl_typesupport_interface"
    DEPENDENCIES python3-importlib-resources ament_cmake_export_include_directories ament_cmake_libraries ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces ament_cmake_export_libraries python3-catkin-pkg-modules ament_cmake_core ament_cmake_version ament_cmake_test ament_cmake_python python3-importlib-metadata cmake ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h ament_cmake ament_cmake_export_definitions python3-setuptools ament_cmake_export_targets ament_package
    CMAKE_ARGS ${extra_cmake_args})

  dep_build(rosidl_runtime_c CMAKE
    SOURCE_DIR "deps/rosidl/rosidl_runtime_c"
    DEPENDENCIES python3-pytest rosidl_typesupport_interface ament_cmake_pytest python3-importlib-resources ament_cmake_export_include_directories gtest_vendor ament_cmake_libraries ament_cmake_export_link_flags ament_cmake_include_directories libatomic ament_cmake_export_interfaces gtest ament_cmake_export_libraries python3-catkin-pkg-modules ament_cmake_core ament_cmake_version ament_cmake_test ament_cmake_python python3-importlib-metadata rcutils cmake ament_cmake_ros ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gtest ament_cmake_gen_version_h domain_coordinator gmock_vendor ament_cmake ament_cmake_gmock google-mock ament_cmake_export_definitions python3-setuptools ament_cmake_export_targets ament_package
    CMAKE_ARGS ${extra_cmake_args})

  dep_build(rmw CMAKE
    SOURCE_DIR "deps/rmw/rmw"
    DEPENDENCIES python3-pytest rosidl_typesupport_interface ament_cmake_pytest python3-importlib-resources ament_cmake_export_include_directories gtest_vendor ament_cmake_libraries ament_cmake_export_link_flags ament_cmake_include_directories libatomic ament_cmake_export_interfaces gtest ament_cmake_export_libraries python3-catkin-pkg-modules ament_cmake_core ament_cmake_version ament_cmake_test rcutils python3-importlib-metadata ament_cmake_python rosidl_runtime_c cmake ament_cmake_ros ament_cmake_export_dependencies ament_cmake_gtest ament_cmake_target_dependencies ament_cmake_gen_version_h domain_coordinator gmock_vendor ament_cmake ament_cmake_gmock google-mock ament_cmake_export_definitions python3-setuptools ament_cmake_export_targets ament_package
    CMAKE_ARGS ${extra_cmake_args})

  dep_build(rcl_yaml_param_parser CMAKE
    SOURCE_DIR "deps/rcl/rcl_yaml_param_parser"
    DEPENDENCIES python3-pytest rosidl_typesupport_interface yaml ament_cmake_pytest python3-importlib-resources ament_cmake_export_include_directories gtest_vendor ament_cmake_libraries ament_cmake_export_link_flags ament_cmake_include_directories libatomic ament_cmake_export_interfaces gtest ament_cmake_export_libraries python3-catkin-pkg-modules ament_cmake_core ament_cmake_version ament_cmake_test rcutils python3-importlib-metadata ament_cmake_python libyaml_vendor rosidl_runtime_c cmake ament_cmake_ros ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gtest ament_cmake_gen_version_h domain_coordinator gmock_vendor ament_cmake ament_cmake_gmock google-mock ament_cmake_export_definitions python3-setuptools rmw ament_cmake_export_targets ament_package
    CMAKE_ARGS ${extra_cmake_args})

  dep_build(rosidl_generator_c CMAKE
    SOURCE_DIR "deps/rosidl/rosidl_generator_c"
    DEPENDENCIES python3-pytest ament_cmake_pytest python3-importlib-resources ament_cmake_export_include_directories gtest_vendor ament_cmake_libraries ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces gtest ament_cmake_export_libraries python3-catkin-pkg-modules ament_cmake_core ament_cmake_version ament_cmake_test ament_cmake_python python3-importlib-metadata cmake ament_cmake_ros ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gtest ament_cmake_gen_version_h domain_coordinator gmock_vendor ament_cmake ament_cmake_gmock google-mock ament_cmake_export_definitions python3-setuptools ament_cmake_export_targets ament_package
    CMAKE_ARGS ${extra_cmake_args})

  dep_build(rosidl_runtime_cpp CMAKE
    SOURCE_DIR "deps/rosidl/rosidl_runtime_cpp"
    DEPENDENCIES rosidl_typesupport_interface python3-importlib-resources ament_cmake_export_include_directories ament_cmake_libraries ament_cmake_export_link_flags ament_cmake_include_directories libatomic ament_cmake_export_interfaces ament_cmake_export_libraries python3-catkin-pkg-modules ament_cmake_core ament_cmake_version ament_cmake_test rcutils python3-importlib-metadata ament_cmake_python rosidl_runtime_c cmake ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h ament_cmake ament_cmake_export_definitions python3-setuptools ament_cmake_export_targets ament_package
    CMAKE_ARGS ${extra_cmake_args})

  dep_build(rosidl_generator_cpp CMAKE
    SOURCE_DIR "deps/rosidl/rosidl_generator_cpp"
    DEPENDENCIES python3-importlib-resources ament_cmake_export_include_directories ament_cmake_libraries ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces ament_cmake_export_libraries python3-catkin-pkg-modules ament_cmake_core ament_cmake_version ament_cmake_test ament_cmake_python python3-importlib-metadata cmake ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h ament_cmake ament_cmake_export_definitions python3-setuptools ament_cmake_export_targets ament_package
    CMAKE_ARGS ${extra_cmake_args})

  dep_build(rosidl_typesupport_c CMAKE
    SOURCE_DIR "deps/rosidl_typesupport/rosidl_typesupport_c"
    DEPENDENCIES python3-pytest rosidl_typesupport_interface ament_cmake_pytest python3-importlib-resources ament_cmake_export_include_directories gtest_vendor ament_cmake_libraries ament_cmake_export_link_flags ament_cmake_include_directories libatomic ament_cmake_export_interfaces gtest ament_cmake_export_libraries python3-catkin-pkg-modules ament_cmake_core ament_cmake_version ament_cmake_test rcutils python3-importlib-metadata ament_cmake_python rosidl_runtime_c cmake ament_cmake_ros ament_cmake_export_dependencies ament_cmake_gtest ament_cmake_target_dependencies ament_cmake_gen_version_h domain_coordinator gmock_vendor ament_cmake python3 ament_cmake_gmock python3-empy google-mock ament_cmake_export_definitions rosidl_cmake rosidl_typesupport_introspection_c python3-setuptools ament_cmake_export_targets rcpputils ament_package
    CMAKE_ARGS ${extra_cmake_args})

  dep_build(rosidl_typesupport_introspection_c CMAKE
    SOURCE_DIR "deps/rosidl/rosidl_typesupport_introspection_c"
    DEPENDENCIES python3-pytest ament_cmake_pytest python3-importlib-resources ament_cmake_export_include_directories gtest_vendor ament_cmake_libraries ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces gtest ament_cmake_export_libraries python3-catkin-pkg-modules ament_cmake_core ament_cmake_version ament_cmake_test ament_cmake_python python3-importlib-metadata cmake ament_cmake_ros ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gtest ament_cmake_gen_version_h domain_coordinator gmock_vendor ament_cmake ament_cmake_gmock google-mock ament_cmake_export_definitions python3-setuptools ament_cmake_export_targets ament_package
    CMAKE_ARGS ${extra_cmake_args})

  dep_build(rosidl_typesupport_cpp CMAKE
    SOURCE_DIR "deps/rosidl_typesupport/rosidl_typesupport_cpp"
    DEPENDENCIES python3-pytest rosidl_runtime_cpp rosidl_typesupport_interface ament_cmake_pytest python3-importlib-resources ament_cmake_export_include_directories gtest_vendor ament_cmake_libraries ament_cmake_export_link_flags ament_cmake_include_directories libatomic ament_cmake_export_interfaces gtest ament_cmake_export_libraries python3-catkin-pkg-modules ament_cmake_core rosidl_typesupport_introspection_cpp ament_cmake_version ament_cmake_test python3-importlib-metadata rcutils ament_cmake_python rosidl_runtime_c cmake ament_cmake_ros ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h domain_coordinator ament_cmake_gtest gmock_vendor ament_cmake python3 ament_cmake_gmock python3-empy google-mock ament_cmake_export_definitions rosidl_typesupport_c rosidl_cmake python3-setuptools ament_cmake_export_targets rcpputils ament_package
    # Hack, need a typesupport using rosidl_typesupport_c to exist before cpp typesupport can be built
    # rosidl_typesupport_introspection_c
    CMAKE_ARGS ${extra_cmake_args})

  dep_build(rosidl_typesupport_introspection_cpp CMAKE
    SOURCE_DIR "deps/rosidl/rosidl_typesupport_introspection_cpp"
    DEPENDENCIES python3-pytest rosidl_typesupport_interface rosidl_runtime_cpp ament_cmake_pytest python3-importlib-resources ament_cmake_export_include_directories gtest_vendor ament_cmake_libraries ament_cmake_export_link_flags ament_cmake_include_directories libatomic ament_cmake_export_interfaces gtest ament_cmake_export_libraries python3-catkin-pkg-modules ament_cmake_core ament_cmake_version ament_cmake_test rcutils python3-importlib-metadata ament_cmake_python rosidl_runtime_c cmake ament_cmake_ros ament_cmake_export_dependencies ament_cmake_gtest ament_cmake_target_dependencies ament_cmake_gen_version_h domain_coordinator gmock_vendor ament_cmake python3 ament_cmake_gmock python3-empy google-mock ament_cmake_export_definitions rosidl_cmake rosidl_typesupport_introspection_c python3-setuptools ament_cmake_export_targets ament_package
    CMAKE_ARGS ${extra_cmake_args})

  dep_build(rosidl_default_generators CMAKE
    SOURCE_DIR "deps/rosidl_defaults/rosidl_default_generators"
    DEPENDENCIES python3-importlib-resources ament_cmake_export_include_directories ament_cmake_libraries ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces ament_cmake_export_libraries python3-catkin-pkg-modules ament_cmake_core ament_cmake_version ament_cmake_test ament_cmake_python python3-importlib-metadata cmake ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h ament_cmake ament_cmake_export_definitions python3-setuptools ament_cmake_export_targets ament_package
    CMAKE_ARGS ${extra_cmake_args})

  dep_build(builtin_interfaces CMAKE
    SOURCE_DIR "deps/rcl_interfaces/builtin_interfaces"
    DEPENDENCIES rosidl_runtime_cpp rosidl_typesupport_interface rosidl_generator_py rosidl_typesupport_fastrtps_cpp python3-importlib-resources ament_cmake_export_include_directories ament_cmake_libraries ament_cmake_export_link_flags ament_cmake_include_directories libatomic ament_cmake_export_interfaces ament_cmake_export_libraries python3-catkin-pkg-modules ament_cmake_core rosidl_typesupport_introspection_cpp ament_cmake_version ament_cmake_test python3-importlib-metadata rcutils ament_cmake_python rosidl_runtime_c cmake ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h ament_cmake rosidl_typesupport_fastrtps_c python3 python3-empy ament_cmake_export_definitions rosidl_typesupport_c rosidl_cmake python3-setuptools rosidl_typesupport_introspection_c rosidl_generator_cpp ament_cmake_export_targets rosidl_typesupport_cpp rcpputils rosidl_generator_c rosidl_default_generators ament_package
    CMAKE_ARGS ${extra_cmake_args})

  dep_build(lifecycle_msgs CMAKE
    SOURCE_DIR "deps/rcl_interfaces/lifecycle_msgs"
    DEPENDENCIES rosidl_runtime_cpp rosidl_typesupport_interface rosidl_generator_py rosidl_typesupport_fastrtps_cpp python3-importlib-resources ament_cmake_export_include_directories ament_cmake_libraries ament_cmake_export_link_flags ament_cmake_include_directories libatomic ament_cmake_export_interfaces ament_cmake_export_libraries python3-catkin-pkg-modules ament_cmake_core rosidl_typesupport_introspection_cpp ament_cmake_version ament_cmake_test python3-importlib-metadata rcutils ament_cmake_python rosidl_runtime_c cmake ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h ament_cmake rosidl_typesupport_fastrtps_c python3 python3-empy ament_cmake_export_definitions rosidl_typesupport_c rosidl_cmake python3-setuptools rosidl_typesupport_introspection_c rosidl_generator_cpp ament_cmake_export_targets rosidl_typesupport_cpp rcpputils rosidl_generator_c rosidl_default_generators ament_package
    CMAKE_ARGS ${extra_cmake_args})

  dep_build(rmw_dds_common CMAKE
    SOURCE_DIR "deps/rmw_dds_common/rmw_dds_common"
    DEPENDENCIES rosidl_runtime_cpp rosidl_typesupport_interface rosidl_generator_py rosidl_typesupport_fastrtps_cpp python3-importlib-resources ament_cmake_export_include_directories ament_cmake_libraries ament_cmake_export_link_flags ament_cmake_include_directories libatomic ament_cmake_export_interfaces ament_cmake_export_libraries python3-catkin-pkg-modules ament_cmake_core ament_cmake_version rosidl_typesupport_introspection_cpp ament_cmake_test python3-importlib-metadata rcutils ament_cmake_python rosidl_runtime_c cmake ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h ament_cmake rosidl_typesupport_fastrtps_c python3 python3-empy ament_cmake_export_definitions rosidl_typesupport_c rosidl_cmake python3-setuptools rosidl_typesupport_introspection_c rmw rosidl_generator_cpp ament_cmake_export_targets rosidl_typesupport_cpp rcpputils rosidl_generator_c rosidl_default_generators ament_package
    CMAKE_ARGS ${extra_cmake_args})

  dep_build(rosidl_default_runtime CMAKE
    SOURCE_DIR "deps/rosidl_defaults/rosidl_default_runtime"
    DEPENDENCIES python3-importlib-resources ament_cmake_export_include_directories ament_cmake_libraries ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces ament_cmake_export_libraries python3-catkin-pkg-modules ament_cmake_core ament_cmake_version ament_cmake_test ament_cmake_python python3-importlib-metadata cmake ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h ament_cmake ament_cmake_export_definitions python3-setuptools ament_cmake_export_targets ament_package
    CMAKE_ARGS ${extra_cmake_args})

  # dep_build(action_msgs CMAKE
  #   SOURCE_DIR "deps/rcl_interfaces/action_msgs"
  #   DEPENDENCIES rosidl_runtime_cpp rosidl_typesupport_interface rosidl_generator_py rosidl_typesupport_fastrtps_cpp python3-importlib-resources ament_cmake_export_include_directories ament_cmake_libraries ament_cmake_export_link_flags ament_cmake_include_directories libatomic ament_cmake_export_interfaces ament_cmake_export_libraries python3-catkin-pkg-modules ament_cmake_core rosidl_typesupport_introspection_cpp ament_cmake_version ament_cmake_python python3-importlib-metadata ament_cmake_test rcutils rosidl_runtime_c cmake ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h builtin_interfaces ament_cmake rosidl_typesupport_fastrtps_c python3 python3-empy ament_cmake_export_definitions rosidl_typesupport_c unique_identifier_msgs rosidl_cmake python3-setuptools rosidl_typesupport_introspection_c rosidl_generator_cpp ament_cmake_export_targets rosidl_typesupport_cpp rcpputils rosidl_generator_c rosidl_default_generators ament_package
  #   CMAKE_ARGS ${extra_cmake_args})

  dep_build(rcl_interfaces CMAKE
    SOURCE_DIR "deps/rcl_interfaces/rcl_interfaces"
    DEPENDENCIES rosidl_runtime_cpp rosidl_typesupport_interface rosidl_generator_py rosidl_typesupport_fastrtps_cpp python3-importlib-resources ament_cmake_export_include_directories ament_cmake_libraries ament_cmake_export_link_flags ament_cmake_include_directories libatomic ament_cmake_export_interfaces ament_cmake_export_libraries python3-catkin-pkg-modules ament_cmake_core rosidl_typesupport_introspection_cpp ament_cmake_version ament_cmake_python python3-importlib-metadata ament_cmake_test rcutils rosidl_runtime_c cmake ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h builtin_interfaces ament_cmake rosidl_typesupport_fastrtps_c python3 python3-empy ament_cmake_export_definitions rosidl_typesupport_c rosidl_cmake python3-setuptools rosidl_typesupport_introspection_c rosidl_generator_cpp ament_cmake_export_targets rosidl_typesupport_cpp rcpputils rosidl_generator_c rosidl_default_generators ament_package
    CMAKE_ARGS ${extra_cmake_args})

  dep_build(composition_interfaces CMAKE
    SOURCE_DIR "deps/rcl_interfaces/composition_interfaces"
    DEPENDENCIES rosidl_runtime_cpp rosidl_typesupport_interface rosidl_generator_py rosidl_typesupport_fastrtps_cpp python3-importlib-resources ament_cmake_export_include_directories ament_cmake_libraries ament_cmake_export_link_flags ament_cmake_include_directories libatomic ament_cmake_export_interfaces ament_cmake_export_libraries python3-catkin-pkg-modules ament_cmake_core rosidl_typesupport_introspection_cpp ament_cmake_version ament_cmake_python python3-importlib-metadata ament_cmake_test rcutils rcl_interfaces rosidl_runtime_c cmake ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h builtin_interfaces ament_cmake rosidl_typesupport_fastrtps_c python3 python3-empy ament_cmake_export_definitions rosidl_typesupport_c rosidl_cmake python3-setuptools rosidl_typesupport_introspection_c rosidl_generator_cpp ament_cmake_export_targets rosidl_typesupport_cpp rcpputils rosidl_generator_c rosidl_default_generators ament_package
    CMAKE_ARGS ${extra_cmake_args})

  dep_build(rosgraph_msgs CMAKE
    SOURCE_DIR "deps/rcl_interfaces/rosgraph_msgs"
    DEPENDENCIES rosidl_runtime_cpp rosidl_typesupport_interface rosidl_generator_py rosidl_typesupport_fastrtps_cpp python3-importlib-resources ament_cmake_export_include_directories ament_cmake_libraries ament_cmake_export_link_flags ament_cmake_include_directories libatomic ament_cmake_export_interfaces ament_cmake_export_libraries python3-catkin-pkg-modules ament_cmake_core rosidl_typesupport_introspection_cpp ament_cmake_version ament_cmake_python python3-importlib-metadata ament_cmake_test rcutils rosidl_runtime_c cmake ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h builtin_interfaces ament_cmake rosidl_typesupport_fastrtps_c python3 python3-empy ament_cmake_export_definitions rosidl_typesupport_c rosidl_cmake python3-setuptools rosidl_typesupport_introspection_c rosidl_generator_cpp ament_cmake_export_targets rosidl_typesupport_cpp rcpputils rosidl_generator_c rosidl_default_generators ament_package
    CMAKE_ARGS ${extra_cmake_args})

  #dep_build(sensor_msgs_py PIP
  #  SOURCE_DIR "deps/common_interfaces/sensor_msgs_py"
  #  DEPENDENCIES )

  dep_build(spdlog_vendor CMAKE
    SOURCE_DIR "deps/spdlog_vendor"
    DEPENDENCIES python3-importlib-resources ament_cmake_export_include_directories ament_cmake_libraries ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces ament_cmake_export_libraries python3-catkin-pkg-modules ament_cmake_core ament_cmake_version ament_cmake_test ament_cmake_python python3-importlib-metadata cmake ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h ament_cmake ament_cmake_export_definitions spdlog python3-setuptools ament_cmake_export_targets ament_package
    CMAKE_ARGS ${extra_cmake_args})

  dep_build(rcl_logging_spdlog CMAKE
    SOURCE_DIR "deps/rcl_logging/rcl_logging_spdlog"
    DEPENDENCIES python3-pytest ament_cmake_pytest python3-importlib-resources ament_cmake_export_include_directories gtest_vendor ament_cmake_libraries ament_cmake_export_link_flags ament_cmake_include_directories libatomic ament_cmake_export_interfaces gtest ament_cmake_export_libraries python3-catkin-pkg-modules ament_cmake_core ament_cmake_version ament_cmake_test ament_cmake_python python3-importlib-metadata rcutils cmake ament_cmake_ros ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gtest ament_cmake_gen_version_h domain_coordinator gmock_vendor ament_cmake ament_cmake_gmock google-mock ament_cmake_export_definitions spdlog python3-setuptools ament_cmake_export_targets rcpputils spdlog_vendor rcl_logging_interface ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(statistics_msgs CMAKE
    SOURCE_DIR "deps/rcl_interfaces/statistics_msgs"
    DEPENDENCIES rosidl_runtime_cpp rosidl_typesupport_interface rosidl_generator_py rosidl_typesupport_fastrtps_cpp python3-importlib-resources ament_cmake_export_include_directories ament_cmake_libraries ament_cmake_export_link_flags ament_cmake_include_directories libatomic ament_cmake_export_interfaces ament_cmake_export_libraries python3-catkin-pkg-modules ament_cmake_core rosidl_typesupport_introspection_cpp ament_cmake_version ament_cmake_python python3-importlib-metadata ament_cmake_test rcutils rosidl_runtime_c cmake ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h builtin_interfaces ament_cmake rosidl_typesupport_fastrtps_c python3 python3-empy ament_cmake_export_definitions rosidl_typesupport_c rosidl_cmake python3-setuptools rosidl_typesupport_introspection_c rosidl_generator_cpp ament_cmake_export_targets rosidl_typesupport_cpp rcpputils rosidl_generator_c rosidl_default_generators ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(std_msgs CMAKE
    SOURCE_DIR "deps/common_interfaces/std_msgs"
    DEPENDENCIES rosidl_runtime_cpp rosidl_typesupport_interface rosidl_generator_py rosidl_typesupport_fastrtps_cpp python3-importlib-resources ament_cmake_export_include_directories ament_cmake_libraries ament_cmake_export_link_flags ament_cmake_include_directories libatomic ament_cmake_export_interfaces ament_cmake_export_libraries python3-catkin-pkg-modules ament_cmake_core rosidl_typesupport_introspection_cpp ament_cmake_version ament_cmake_python python3-importlib-metadata ament_cmake_test rcutils rosidl_runtime_c cmake ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h builtin_interfaces ament_cmake rosidl_typesupport_fastrtps_c python3 python3-empy ament_cmake_export_definitions rosidl_typesupport_c rosidl_cmake python3-setuptools rosidl_typesupport_introspection_c rosidl_generator_cpp ament_cmake_export_targets rosidl_typesupport_cpp rcpputils rosidl_generator_c rosidl_default_generators ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(actionlib_msgs CMAKE
    SOURCE_DIR "deps/common_interfaces/actionlib_msgs"
    DEPENDENCIES rosidl_runtime_cpp rosidl_typesupport_interface rosidl_generator_py rosidl_typesupport_fastrtps_cpp python3-importlib-resources ament_cmake_export_include_directories ament_cmake_libraries ament_cmake_export_link_flags ament_cmake_include_directories libatomic ament_cmake_export_interfaces std_msgs ament_cmake_export_libraries python3-catkin-pkg-modules ament_cmake_core rosidl_typesupport_introspection_cpp ament_cmake_version ament_cmake_python python3-importlib-metadata ament_cmake_test rcutils rosidl_runtime_c cmake ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h builtin_interfaces ament_cmake rosidl_typesupport_fastrtps_c python3 python3-empy ament_cmake_export_definitions rosidl_typesupport_c rosidl_cmake python3-setuptools rosidl_typesupport_introspection_c rosidl_generator_cpp ament_cmake_export_targets rosidl_typesupport_cpp rcpputils rosidl_generator_c rosidl_default_generators ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(geometry_msgs CMAKE
    SOURCE_DIR "deps/common_interfaces/geometry_msgs"
    DEPENDENCIES rosidl_runtime_cpp rosidl_typesupport_interface rosidl_generator_py rosidl_typesupport_fastrtps_cpp python3-importlib-resources ament_cmake_export_include_directories ament_cmake_libraries ament_cmake_export_link_flags ament_cmake_include_directories libatomic ament_cmake_export_interfaces std_msgs ament_cmake_export_libraries python3-catkin-pkg-modules ament_cmake_core rosidl_typesupport_introspection_cpp ament_cmake_version ament_cmake_python python3-importlib-metadata ament_cmake_test rcutils rosidl_runtime_c cmake ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h builtin_interfaces ament_cmake rosidl_typesupport_fastrtps_c python3 python3-empy ament_cmake_export_definitions rosidl_typesupport_c rosidl_cmake python3-setuptools rosidl_typesupport_introspection_c rosidl_generator_cpp ament_cmake_export_targets rosidl_typesupport_cpp rcpputils rosidl_generator_c rosidl_default_generators ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(diagnostic_msgs CMAKE
    SOURCE_DIR "deps/common_interfaces/diagnostic_msgs"
    DEPENDENCIES rosidl_runtime_cpp rosidl_typesupport_interface geometry_msgs rosidl_generator_py rosidl_typesupport_fastrtps_cpp python3-importlib-resources ament_cmake_export_include_directories ament_cmake_libraries ament_cmake_export_link_flags ament_cmake_include_directories libatomic ament_cmake_export_interfaces std_msgs ament_cmake_export_libraries python3-catkin-pkg-modules ament_cmake_core rosidl_typesupport_introspection_cpp ament_cmake_version ament_cmake_python python3-importlib-metadata ament_cmake_test rcutils rosidl_runtime_c cmake ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h builtin_interfaces ament_cmake rosidl_typesupport_fastrtps_c python3 python3-empy ament_cmake_export_definitions rosidl_typesupport_c rosidl_cmake python3-setuptools rosidl_typesupport_introspection_c rosidl_generator_cpp ament_cmake_export_targets rosidl_typesupport_cpp rcpputils rosidl_generator_c rosidl_default_generators ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(nav_msgs CMAKE
    SOURCE_DIR "deps/common_interfaces/nav_msgs"
    DEPENDENCIES rosidl_runtime_cpp rosidl_typesupport_interface geometry_msgs rosidl_generator_py rosidl_typesupport_fastrtps_cpp python3-importlib-resources ament_cmake_export_include_directories ament_cmake_libraries ament_cmake_export_link_flags ament_cmake_include_directories libatomic ament_cmake_export_interfaces std_msgs ament_cmake_export_libraries python3-catkin-pkg-modules ament_cmake_core rosidl_typesupport_introspection_cpp ament_cmake_version ament_cmake_python python3-importlib-metadata ament_cmake_test rcutils rosidl_runtime_c cmake ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h builtin_interfaces ament_cmake rosidl_typesupport_fastrtps_c python3 python3-empy ament_cmake_export_definitions rosidl_typesupport_c rosidl_cmake python3-setuptools rosidl_typesupport_introspection_c rosidl_generator_cpp ament_cmake_export_targets rosidl_typesupport_cpp rcpputils rosidl_generator_c rosidl_default_generators ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(sensor_msgs CMAKE
    SOURCE_DIR "deps/common_interfaces/sensor_msgs"
    DEPENDENCIES rosidl_runtime_cpp rosidl_typesupport_interface geometry_msgs rosidl_generator_py rosidl_typesupport_fastrtps_cpp python3-importlib-resources ament_cmake_export_include_directories ament_cmake_libraries ament_cmake_export_link_flags ament_cmake_include_directories libatomic ament_cmake_export_interfaces std_msgs ament_cmake_export_libraries python3-catkin-pkg-modules ament_cmake_core rosidl_typesupport_introspection_cpp ament_cmake_version ament_cmake_python python3-importlib-metadata ament_cmake_test rcutils rosidl_runtime_c cmake ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h builtin_interfaces ament_cmake rosidl_typesupport_fastrtps_c python3 python3-empy ament_cmake_export_definitions rosidl_typesupport_c rosidl_cmake python3-setuptools rosidl_typesupport_introspection_c rosidl_generator_cpp ament_cmake_export_targets rosidl_typesupport_cpp rcpputils rosidl_generator_c rosidl_default_generators ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(shape_msgs CMAKE
    SOURCE_DIR "deps/common_interfaces/shape_msgs"
    DEPENDENCIES rosidl_runtime_cpp rosidl_typesupport_interface geometry_msgs rosidl_generator_py rosidl_typesupport_fastrtps_cpp python3-importlib-resources ament_cmake_export_include_directories ament_cmake_libraries ament_cmake_export_link_flags ament_cmake_include_directories libatomic ament_cmake_export_interfaces std_msgs ament_cmake_export_libraries python3-catkin-pkg-modules ament_cmake_core rosidl_typesupport_introspection_cpp ament_cmake_version ament_cmake_python python3-importlib-metadata ament_cmake_test rcutils rosidl_runtime_c cmake ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h builtin_interfaces ament_cmake rosidl_typesupport_fastrtps_c python3 python3-empy ament_cmake_export_definitions rosidl_typesupport_c rosidl_cmake python3-setuptools rosidl_typesupport_introspection_c rosidl_generator_cpp ament_cmake_export_targets rosidl_typesupport_cpp rcpputils rosidl_generator_c rosidl_default_generators ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(std_srvs CMAKE
    SOURCE_DIR "deps/common_interfaces/std_srvs"
    DEPENDENCIES rosidl_runtime_cpp rosidl_typesupport_interface rosidl_generator_py rosidl_typesupport_fastrtps_cpp python3-importlib-resources ament_cmake_export_include_directories ament_cmake_libraries ament_cmake_export_link_flags ament_cmake_include_directories libatomic ament_cmake_export_interfaces ament_cmake_export_libraries python3-catkin-pkg-modules ament_cmake_core rosidl_typesupport_introspection_cpp ament_cmake_version ament_cmake_test python3-importlib-metadata rcutils ament_cmake_python rosidl_runtime_c cmake ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h ament_cmake rosidl_typesupport_fastrtps_c python3 python3-empy ament_cmake_export_definitions rosidl_typesupport_c rosidl_cmake python3-setuptools rosidl_typesupport_introspection_c rosidl_generator_cpp ament_cmake_export_targets rosidl_typesupport_cpp rcpputils rosidl_generator_c rosidl_default_generators ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(stereo_msgs CMAKE
    SOURCE_DIR "deps/common_interfaces/stereo_msgs"
    DEPENDENCIES rosidl_runtime_cpp rosidl_typesupport_interface geometry_msgs rosidl_generator_py rosidl_typesupport_fastrtps_cpp python3-importlib-resources ament_cmake_export_include_directories ament_cmake_libraries ament_cmake_export_link_flags ament_cmake_include_directories libatomic ament_cmake_export_interfaces std_msgs ament_cmake_export_libraries sensor_msgs python3-catkin-pkg-modules ament_cmake_core rosidl_typesupport_introspection_cpp ament_cmake_version ament_cmake_python python3-importlib-metadata ament_cmake_test rcutils rosidl_runtime_c cmake ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h builtin_interfaces ament_cmake rosidl_typesupport_fastrtps_c python3 python3-empy ament_cmake_export_definitions rosidl_typesupport_c rosidl_cmake python3-setuptools rosidl_typesupport_introspection_c rosidl_generator_cpp ament_cmake_export_targets rosidl_typesupport_cpp rcpputils rosidl_generator_c rosidl_default_generators ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  # dep_build(test_msgs CMAKE
  #   SOURCE_DIR "deps/rcl_interfaces/test_msgs"
  #   DEPENDENCIES rosidl_runtime_cpp rosidl_typesupport_interface rosidl_generator_py rosidl_typesupport_fastrtps_cpp python3-importlib-resources ament_cmake_export_include_directories ament_cmake_libraries ament_cmake_export_link_flags ament_cmake_include_directories libatomic ament_cmake_export_interfaces ament_cmake_export_libraries action_msgs python3-catkin-pkg-modules ament_cmake_core rosidl_typesupport_introspection_cpp ament_cmake_version ament_cmake_python python3-importlib-metadata ament_cmake_test rcutils rosidl_runtime_c cmake ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h builtin_interfaces ament_cmake rosidl_typesupport_fastrtps_c python3 python3-empy ament_cmake_export_definitions rosidl_typesupport_c unique_identifier_msgs rosidl_cmake python3-setuptools rosidl_typesupport_introspection_c rosidl_generator_cpp test_interface_files ament_cmake_export_targets rosidl_typesupport_cpp rcpputils rosidl_generator_c rosidl_default_generators ament_package
  #   CMAKE_ARGS ${extra_cmake_args})

  dep_build(tracetools CMAKE
    SOURCE_DIR "deps/ros2_tracing/tracetools"
    DEPENDENCIES python3-pytest ament_cmake_pytest python3-importlib-resources ament_cmake_export_include_directories gtest_vendor ament_cmake_libraries ament_cmake_export_link_flags ament_cmake_include_directories ament_cmake_export_interfaces gtest ament_cmake_export_libraries python3-catkin-pkg-modules ament_cmake_core ament_cmake_version ament_cmake_test ament_cmake_python python3-importlib-metadata cmake ament_cmake_ros ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gtest ament_cmake_gen_version_h domain_coordinator gmock_vendor ament_cmake ament_cmake_gmock google-mock pkg-config ament_cmake_export_definitions python3-setuptools ament_cmake_export_targets ament_package
    CMAKE_ARGS ${extra_cmake_args})

  dep_build(rmw_cyclonedds_cpp CMAKE
    SOURCE_DIR "deps/rmw_cyclonedds/rmw_cyclonedds_cpp"
    DEPENDENCIES python3-pytest rosidl_runtime_cpp rosidl_typesupport_interface ament_cmake_pytest python3-importlib-resources ament_cmake_export_include_directories gtest_vendor iceoryx_posh tracetools ament_cmake_libraries ament_cmake_export_link_flags ament_cmake_include_directories libatomic ament_cmake_export_interfaces gtest ament_cmake_export_libraries iceoryx_utils python3-catkin-pkg-modules ament_cmake_core rosidl_typesupport_introspection_cpp ament_cmake_version iceoryx_hoofs python3-importlib-metadata ament_cmake_test rcutils ament_cmake_python rosidl_runtime_c cmake ament_cmake_ros iceoryx_binding_c ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h domain_coordinator ament_cmake_gtest acl ament_cmake python3 ament_cmake_gmock python3-empy gmock_vendor google-mock ament_cmake_export_definitions rosidl_cmake python3-setuptools rosidl_typesupport_introspection_c rmw ament_cmake_export_targets rcpputils rmw_dds_common cyclonedds ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(rmw_implementation CMAKE
    SOURCE_DIR "deps/rmw_implementation/rmw_implementation"
    DEPENDENCIES rosidl_runtime_cpp rosidl_typesupport_interface python3-importlib-resources ament_cmake_export_include_directories iceoryx_posh ament_cmake_libraries tracetools ament_cmake_export_link_flags ament_cmake_include_directories libatomic ament_cmake_export_interfaces ament_cmake_export_libraries rmw_connextdds rmw_implementation_cmake iceoryx_utils python3-catkin-pkg-modules ament_index_cpp ament_cmake_core ament_cmake_version rosidl_typesupport_introspection_cpp iceoryx_hoofs python3-importlib-metadata ament_cmake_test rcutils ament_cmake_python cmake rosidl_runtime_c iceoryx_binding_c ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h acl ament_cmake python3 python3-empy ament_cmake_export_definitions rmw_cyclonedds_cpp rosidl_cmake python3-setuptools rosidl_typesupport_introspection_c rmw ament_cmake_export_targets rmw_fastrtps_dynamic_cpp rcpputils rmw_dds_common cyclonedds rmw_fastrtps_cpp ament_package
    CMAKE_ARGS ${extra_cmake_args})

  dep_build(rcl CMAKE
    SOURCE_DIR "deps/rcl/rcl"
    DEPENDENCIES python3-pytest rosidl_typesupport_interface rcl_yaml_param_parser yaml ament_cmake_pytest python3-importlib-resources ament_cmake_export_include_directories rcl_logging_spdlog gtest_vendor tracetools ament_cmake_libraries ament_cmake_export_link_flags ament_cmake_include_directories libatomic ament_cmake_export_interfaces gtest ament_cmake_export_libraries rmw_implementation_cmake rmw_implementation python3-catkin-pkg-modules ament_index_cpp ament_cmake_core ament_cmake_version ament_cmake_test rcutils python3-importlib-metadata ament_cmake_python rcl_interfaces libyaml_vendor rosidl_runtime_c cmake ament_cmake_ros ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h domain_coordinator builtin_interfaces ament_cmake_gtest gmock_vendor ament_cmake ament_cmake_gmock google-mock ament_cmake_export_definitions python3-setuptools rmw ament_cmake_export_targets rcpputils rcl_logging_interface ament_package
    rcl_logging_android
    CMAKE_ARGS ${extra_cmake_args}
    -DRCL_LOGGING_IMPLEMENTATION=rcl_logging_android
    )

  dep_build(libstatistics_collector CMAKE
    SOURCE_DIR "deps/libstatistics_collector"
    DEPENDENCIES python3-pytest rosidl_typesupport_interface rcl_yaml_param_parser yaml rosidl_generator_py ament_cmake_pytest rosidl_typesupport_fastrtps_cpp python3-importlib-resources rosidl_runtime_cpp ament_cmake_export_include_directories rcl_logging_spdlog gtest_vendor tracetools ament_cmake_libraries ament_cmake_export_link_flags ament_cmake_include_directories libatomic ament_cmake_export_interfaces statistics_msgs std_msgs gtest ament_cmake_export_libraries rmw_implementation_cmake rmw_implementation python3-catkin-pkg-modules ament_index_cpp ament_cmake_core ament_cmake_version rosidl_typesupport_introspection_cpp ament_cmake_python python3-importlib-metadata ament_cmake_test rcutils rcl_interfaces libyaml_vendor rosidl_runtime_c cmake ament_cmake_ros ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h domain_coordinator builtin_interfaces rcl ament_cmake_gtest gmock_vendor ament_cmake rosidl_typesupport_fastrtps_c python3 python3-empy ament_cmake_gmock google-mock ament_cmake_export_definitions rosidl_typesupport_c rosidl_cmake python3-setuptools rosidl_typesupport_introspection_c rmw rosidl_generator_cpp ament_cmake_export_targets rosidl_typesupport_cpp rcpputils rosidl_generator_c rosidl_default_generators rcl_logging_interface ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  #dep_build(rcl_action CMAKE
  #  SOURCE_DIR "deps/rcl/rcl_action"
  #  DEPENDENCIES python3-pytest rosidl_typesupport_interface rcl_yaml_param_parser yaml ament_cmake_pytest python3-importlib-resources ament_cmake_export_include_directories rcl_logging_spdlog gtest_vendor ament_cmake_libraries tracetools ament_cmake_export_link_flags ament_cmake_include_directories libatomic ament_cmake_export_interfaces gtest ament_cmake_export_libraries rmw_implementation_cmake rmw_implementation python3-catkin-pkg-modules ament_index_cpp action_msgs ament_cmake_version ament_cmake_test ament_cmake_core python3-importlib-metadata rcutils ament_cmake_python rcl_interfaces libyaml_vendor rosidl_runtime_c cmake ament_cmake_ros ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h domain_coordinator builtin_interfaces rcl ament_cmake_gtest gmock_vendor ament_cmake ament_cmake_gmock google-mock ament_cmake_export_definitions unique_identifier_msgs python3-setuptools rmw ament_cmake_export_targets rcpputils rcl_logging_interface ament_package
  #  CMAKE_ARGS ${extra_cmake_args})
  #
  dep_build(rcl_lifecycle CMAKE
    SOURCE_DIR "deps/rcl/rcl_lifecycle"
    DEPENDENCIES python3-pytest rosidl_typesupport_interface rcl_yaml_param_parser yaml ament_cmake_pytest python3-importlib-resources lifecycle_msgs ament_cmake_export_include_directories rcl_logging_spdlog gtest_vendor tracetools ament_cmake_libraries ament_cmake_export_link_flags ament_cmake_include_directories libatomic ament_cmake_export_interfaces gtest ament_cmake_export_libraries rmw_implementation_cmake rmw_implementation python3-catkin-pkg-modules ament_index_cpp ament_cmake_core ament_cmake_version ament_cmake_test rcutils python3-importlib-metadata ament_cmake_python rcl_interfaces libyaml_vendor rosidl_runtime_c cmake ament_cmake_ros ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h domain_coordinator builtin_interfaces rcl ament_cmake_gtest gmock_vendor ament_cmake ament_cmake_gmock google-mock ament_cmake_export_definitions python3-setuptools rmw ament_cmake_export_targets rcpputils rcl_logging_interface ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(rclcpp CMAKE
    SOURCE_DIR "deps/rclcpp/rclcpp"
    DEPENDENCIES python3-pytest rosidl_typesupport_interface rcl_yaml_param_parser yaml rosidl_runtime_cpp ament_cmake_pytest python3-importlib-resources ament_cmake_export_include_directories rcl_logging_spdlog gtest_vendor tracetools ament_cmake_libraries rosgraph_msgs ament_cmake_export_link_flags ament_cmake_include_directories libatomic ament_cmake_export_interfaces statistics_msgs gtest ament_cmake_export_libraries rmw_implementation_cmake rmw_implementation python3-catkin-pkg-modules ament_index_cpp ament_cmake_core ament_cmake_version ament_cmake_test ament_cmake_python python3-importlib-metadata rcutils rcl_interfaces libyaml_vendor rosidl_runtime_c cmake ament_cmake_ros ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h domain_coordinator builtin_interfaces rcl ament_cmake_gtest gmock_vendor ament_cmake python3 ament_cmake_gmock google-mock ament_cmake_export_definitions rosidl_typesupport_c python3-setuptools rmw ament_cmake_export_targets rosidl_typesupport_cpp libstatistics_collector rcpputils rcl_logging_interface ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  #dep_build(rclcpp_action CMAKE
  #  SOURCE_DIR "deps/rclcpp/rclcpp_action"
  #  DEPENDENCIES python3-pytest rclcpp rosidl_typesupport_interface rcl_yaml_param_parser yaml rosidl_runtime_cpp ament_cmake_pytest python3-importlib-resources ament_cmake_export_include_directories rcl_logging_spdlog gtest_vendor ament_cmake_libraries tracetools rosgraph_msgs ament_cmake_export_link_flags ament_cmake_include_directories libatomic ament_cmake_export_interfaces statistics_msgs gtest ament_cmake_export_libraries rmw_implementation_cmake rmw_implementation python3-catkin-pkg-modules rcl_action ament_index_cpp action_msgs ament_cmake_version ament_cmake_test python3-importlib-metadata ament_cmake_core rcutils ament_cmake_python rcl_interfaces libyaml_vendor cmake rosidl_runtime_c ament_cmake_ros ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h domain_coordinator builtin_interfaces rcl ament_cmake_gtest gmock_vendor ament_cmake python3 ament_cmake_gmock google-mock ament_cmake_export_definitions rosidl_typesupport_c unique_identifier_msgs python3-setuptools rmw ament_cmake_export_targets rosidl_typesupport_cpp libstatistics_collector rcpputils rcl_logging_interface ament_package
  #  CMAKE_ARGS ${extra_cmake_args})
  #
  # dep_build(rclcpp_components CMAKE
  #   SOURCE_DIR "deps/rclcpp/rclcpp_components"
  #   DEPENDENCIES python3-pytest rclcpp rosidl_typesupport_interface rcl_yaml_param_parser yaml rosidl_runtime_cpp ament_cmake_pytest python3-importlib-resources ament_cmake_export_include_directories rcl_logging_spdlog gtest_vendor tracetools ament_cmake_libraries rosgraph_msgs ament_cmake_export_link_flags ament_cmake_include_directories libatomic class_loader ament_cmake_export_interfaces statistics_msgs gtest ament_cmake_export_libraries rmw_implementation_cmake rmw_implementation python3-catkin-pkg-modules ament_index_cpp ament_cmake_core ament_cmake_version ament_cmake_test ament_cmake_python python3-importlib-metadata rcutils rcl_interfaces libyaml_vendor rosidl_runtime_c cmake ament_cmake_ros ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h domain_coordinator builtin_interfaces rcl ament_cmake_gtest gmock_vendor ament_cmake python3 ament_cmake_gmock google-mock ament_cmake_export_definitions rosidl_typesupport_c python3-setuptools rmw ament_cmake_export_targets rosidl_typesupport_cpp libstatistics_collector composition_interfaces rcpputils rcl_logging_interface ament_package
  #   CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(rclcpp_lifecycle CMAKE
    SOURCE_DIR "deps/rclcpp/rclcpp_lifecycle"
    DEPENDENCIES python3-pytest rclcpp rosidl_typesupport_interface rcl_yaml_param_parser yaml rosidl_runtime_cpp ament_cmake_pytest python3-importlib-resources lifecycle_msgs ament_cmake_export_include_directories rcl_logging_spdlog gtest_vendor ament_cmake_libraries tracetools rosgraph_msgs ament_cmake_export_link_flags ament_cmake_include_directories libatomic ament_cmake_export_interfaces statistics_msgs rcl_lifecycle ament_cmake_export_libraries rmw_implementation_cmake rmw_implementation gtest python3-catkin-pkg-modules ament_index_cpp ament_cmake_core ament_cmake_version ament_cmake_test rcutils python3-importlib-metadata ament_cmake_python rcl_interfaces libyaml_vendor rosidl_runtime_c cmake ament_cmake_ros ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h domain_coordinator builtin_interfaces rcl ament_cmake_gtest gmock_vendor ament_cmake python3 ament_cmake_gmock google-mock ament_cmake_export_definitions rosidl_typesupport_c python3-setuptools rmw ament_cmake_export_targets rosidl_typesupport_cpp libstatistics_collector rcpputils rcl_logging_interface ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
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
  #  DEPENDENCIES launch launch_ros tracetools_trace)
  #
  #dep_build(tracetools_test PIP
  #  SOURCE_DIR "deps/ros2_tracing/tracetools_test"
  #  DEPENDENCIES launch_ros tracetools_read launch tracetools_trace tracetools_launch)
  #
  #dep_build(test_tracetools_launch PIP
  #  SOURCE_DIR "deps/ros2_tracing/test_tracetools_launch"
  #  DEPENDENCIES )

  dep_build(trajectory_msgs CMAKE
    SOURCE_DIR "deps/common_interfaces/trajectory_msgs"
    DEPENDENCIES rosidl_runtime_cpp rosidl_typesupport_interface geometry_msgs rosidl_generator_py rosidl_typesupport_fastrtps_cpp python3-importlib-resources ament_cmake_export_include_directories ament_cmake_libraries ament_cmake_export_link_flags ament_cmake_include_directories libatomic ament_cmake_export_interfaces std_msgs ament_cmake_export_libraries python3-catkin-pkg-modules ament_cmake_core rosidl_typesupport_introspection_cpp ament_cmake_version ament_cmake_python python3-importlib-metadata ament_cmake_test rcutils rosidl_runtime_c cmake ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h builtin_interfaces ament_cmake rosidl_typesupport_fastrtps_c python3 python3-empy ament_cmake_export_definitions rosidl_typesupport_c rosidl_cmake python3-setuptools rosidl_typesupport_introspection_c rosidl_generator_cpp ament_cmake_export_targets rosidl_typesupport_cpp rcpputils rosidl_generator_c rosidl_default_generators ament_package
    CMAKE_ARGS ${extra_cmake_args})
  
  dep_build(visualization_msgs CMAKE
    SOURCE_DIR "deps/common_interfaces/visualization_msgs"
    DEPENDENCIES rosidl_runtime_cpp rosidl_typesupport_interface geometry_msgs rosidl_generator_py rosidl_typesupport_fastrtps_cpp python3-importlib-resources ament_cmake_export_include_directories ament_cmake_libraries ament_cmake_export_link_flags ament_cmake_include_directories libatomic ament_cmake_export_interfaces std_msgs ament_cmake_export_libraries sensor_msgs python3-catkin-pkg-modules ament_cmake_core rosidl_typesupport_introspection_cpp ament_cmake_version ament_cmake_python python3-importlib-metadata ament_cmake_test rcutils rosidl_runtime_c cmake ament_cmake_export_dependencies ament_cmake_target_dependencies ament_cmake_gen_version_h builtin_interfaces ament_cmake rosidl_typesupport_fastrtps_c python3 python3-empy ament_cmake_export_definitions rosidl_typesupport_c rosidl_cmake python3-setuptools rosidl_typesupport_introspection_c rosidl_generator_cpp ament_cmake_export_targets rosidl_typesupport_cpp rcpputils rosidl_generator_c rosidl_default_generators ament_package
    CMAKE_ARGS ${extra_cmake_args})
endmacro()
