#!/usr/bin/env python3

"""
Generate a CMake superbuild from a directory of ROS packages.
"""

CMAKE_TEMPLATE = """
dep_build({package_name} CMAKE
  SOURCE_DIR "{path_to_source}"
  DEPENDENCIES {dependencies}
  CMAKE_ARGS ${{android_cmake_args}})
"""

PIP_TEMPLATE = """
dep_build({package_name} PIP
  SOURCE_DIR "{path_to_source}"
  DEPENDENCIES {dependencies})
"""

import pathlib
import sys

import argparse

from catkin_pkg.topological_order import topological_order


def _build_depends(pkg):
    deps = []
    deps.extend(pkg.buildtool_depends)
    deps.extend(pkg.build_depends)
    deps.extend(pkg.build_depends)
    # TODO(sloretz) interface packages need group depends?

    just_names = set([dep.name for dep in deps])
    rendered = ' '.join(just_names)
    return rendered


def _source_dir(pkg):
    package_xml_path = pathlib.Path(pkg.filename)
    return str(package_xml_path.parent)


def _generate_cmake(pkg, output_to):
    rendered = CMAKE_TEMPLATE.format(
        package_name=pkg.name,
        path_to_source=_source_dir(pkg),
        dependencies=_build_depends(pkg))
    output_to.write(rendered)


def _generate_pip(pkg, output_to):
    rendered = PIP_TEMPLATE.format(
        package_name=pkg.name,
        path_to_source=_source_dir(pkg),
        dependencies=_build_depends(pkg))
    output_to.write(rendered)


def generate_ros_superbuild(source_dir, output_to=sys.stdout):
    for _, pkg in topological_order(source_dir):
        build_type = pkg.get_build_type().lower()
        if build_type in ('cmake', 'ament_cmake'):
            _generate_cmake(pkg, output_to)
        elif build_type in ('python', 'ament_python'):
            _generate_pip(pkg, output_to)
        else:
            raise ValueError(f'Unknown build type: {build_type}')


def main():
    parser = argparse.ArgumentParser(
        description='Generate a superbuild from a directory of ROS packages.')
    parser.add_argument(
        'path', metavar='PATH', type=str,
        help='path to directory containing ROS packages')

    args = parser.parse_args()

    generate_ros_superbuild(args.path)


if __name__ == '__main__':
    main()
