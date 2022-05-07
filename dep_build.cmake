# Inputs:
#   Name to give to the rule
#   path to the source code
#   packages it depends on
#   Extra CMake arguments
# Append 

function(dep_build name)
  cmake_parse_arguments(ARG "CMAKE;PIP;NATIVE" "SOURCE_DIR" "CMAKE_ARGS;DEPENDENCIES" ${ARGN})
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "dep_build() given unknown arguments: "
      "${ARG_UNPARSED_ARGUMENTS}")
  endif()
  if(NOT ARG_CMAKE AND NOT ARG_PIP)
    message(FATAL_ERROR "dep_build() must be given PIP or CMAKE")
  endif()
  if(ARG_CMAKE AND ARG_PIP)
    message(FATAL_ERROR "dep_build() must be given only one of PIP or CMAKE")
  endif()

  if (NOT ARG_SOURCE_DIR)
    message(FATAL_ERROR "SOURCE_DIR for dependency must be given")
  endif()

  # Assume path is relative to current source dir
  set(ARG_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/${ARG_SOURCE_DIR}")

  if (NOT EXISTS "${ARG_SOURCE_DIR}")
    message(FATAL_ERROR "SOURCE_DIR must exist: ${ARG_SOURCE_DIR}")
  endif()

  if(ARG_NATIVE)
    # Prefix with `native-` so find_package() can't normally find CycloneDDS here
    set(name "native-${name}")
  endif()

  set(dep_name "deps-${name}")
  # Place where CMake packges get installed
  set(cmake_install_dir "${CMAKE_CURRENT_BINARY_DIR}/deps")
  # Place where Python packages get installed
  set(pip_install_dir "${CMAKE_CURRENT_BINARY_DIR}/deps/_python_")
  if(ARG_NATIVE)
    set(cmake_install_dir "${CMAKE_CURRENT_BINARY_DIR}/native-deps")
    set(pip_install_dir "${CMAKE_CURRENT_BINARY_DIR}/native-deps/_python_")
  endif()

  set(ament_prefix_path)
  list(APPEND dependency_targets)
  foreach(_dependency ${ARG_DEPENDENCIES})
    # Assume each dependency was also called with `dep_build`
    set(_dep_target "deps-${_dependency}")
    if(NOT TARGET ${_dep_target})
      message(WARNING "Dependency target ${_dep_target} does not exist")
    else()
      list(APPEND dependency_targets ${_dep_target})
    endif()
  endforeach()

  if(ARG_CMAKE)
    set(cmake_with_env "${CMAKE_COMMAND}" -E
      env
      "PYTHONPATH=${pip_install_dir}"
      "AMENT_PREFIX_PATH=${cmake_install_dir}"
      "${CMAKE_COMMAND}")

    ExternalProject_Add(${dep_name}
      # BUILD_ALWAYS ON
      DOWNLOAD_COMMAND ""
      # Assume every CMake package might need access to installed python packages when building
      CMAKE_COMMAND ${cmake_with_env}
      BUILD_COMMAND ${cmake_with_env} --build .
      DEPENDS
      ${dependency_targets}
      SOURCE_DIR "${ARG_SOURCE_DIR}"
      CMAKE_ARGS
      "-DCMAKE_FIND_ROOT_PATH=${cmake_install_dir}"
      "-DCMAKE_INSTALL_PREFIX=${cmake_install_dir}"
      -DBUILD_TESTING=OFF
      "-DPYTHON_INSTALL_DIR=_python_"
      ${ARG_CMAKE_ARGS})
  elseif(ARG_PIP)
    ExternalProject_Add(${dep_name}
      # BUILD_ALWAYS ON
      DOWNLOAD_COMMAND ""
      CONFIGURE_COMMAND ""
      BUILD_COMMAND ""
      TEST_COMMAND ""
      DEPENDS
      ${dependency_targets}
      SOURCE_DIR "${ARG_SOURCE_DIR}"
      INSTALL_COMMAND
      pip install
      -t "${pip_install_dir}"
      --no-deps
      "${ARG_SOURCE_DIR}")
  endif()
endfunction()
