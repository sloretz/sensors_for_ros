


# Inputs:
#   Name to give to the rule
#   path to the source code
#   packages it depends on
#   Extra CMake arguments
# Append 

function(dep_build name)
  cmake_parse_arguments(ARG "CMAKE;PIP" "SOURCE_DIR" "CMAKE_ARGS;DEPENDENCIES" ${ARGN})
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

  set(dep_name "deps-${name}")
  # Place where CMake packges get installed
  set(cmake_install_dir "${CMAKE_CURRENT_BINARY_DIR}/deps/${name}")
  # Place where Python packages get installed
  set(pip_install_dir "${CMAKE_CURRENT_BINARY_DIR}/deps/_python_")

  # Assume each dependency was also called with `dep_build` and has an install target
  list(APPEND dependency_targets)
  foreach(_dependency ${DEPENDENCIES})
    set(_dep_target "deps-${_dependency}-install")
    list(APPEND dependency_targets ${_dep_target})
    if(NOT TARGET ${_dep_target})
      message(FATAL_ERROR "Dependency target ${_dep_target} does not exist")
    endif()
  endforeach()

  if(ARG_CMAKE)
    ExternalProject_Add(${dep_name}
      DOWNLOAD_COMMAND ""
      # Assume every CMake package might need access to installed python packages when building
      CMAKE_COMMAND "${CMAKE_COMMAND}" -E
        env
          "PYTHONPATH=${pip_install_dir}"
        "${CMAKE_COMMAND}"
      DEPENDS
        ${dependency_targets}
        SOURCE_DIR "${ARG_SOURCE_DIR}"
      CMAKE_ARGS
        "-DCMAKE_FIND_ROOT_PATH=${CMAKE_CURRENT_BINARY_DIR}/deps"
        "-DCMAKE_INSTALL_PREFIX=${cmake_install_dir}"
        -DBUILD_TESTING=OFF
        ${ARG_CMAKE_ARGS}
    )
  endif()

  # Add an install target so other external projects can depend on it
  ExternalProject_Add_StepTargets("${dep_name}" "install")
endfunction()


