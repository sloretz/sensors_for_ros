#!/usr/bin/env python3

""" Generate a CMake superbuild from a directory of ROS packages. """

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


def _export_depends(pkg):
    deps = []
    deps.extend(pkg.buildtool_export_depends)
    deps.extend(pkg.build_export_depends)

    return deps


def _build_depends(pkg):
    deps = []
    deps.extend(pkg.buildtool_depends)
    deps.extend(pkg.build_depends)
    return deps


def _dependencies_to_names(packages):
    just_names = set([pkg.name for pkg in packages])
    rendered = ' '.join(just_names)
    return rendered


def _source_dir(pkg):
    package_xml_path = pathlib.Path(pkg.filename)
    return str(package_xml_path.parent)


def _generate_cmake(pkg, dependencies, output_to):
    rendered = CMAKE_TEMPLATE.format(
        package_name=pkg.name,
        path_to_source=_source_dir(pkg),
        dependencies=_dependencies_to_names(dependencies))
    output_to.write(rendered)


def _generate_pip(pkg, dependencies, output_to):
    rendered = PIP_TEMPLATE.format(
        package_name=pkg.name,
        path_to_source=_source_dir(pkg),
        dependencies=_dependencies_to_names(dependencies))
    output_to.write(rendered)


def generate_ros_superbuild(source_dir, output_to=sys.stdout):

    export_lookup = {}
    packages = dict(topological_order(source_dir))

    for pkg in packages.values():
        export_lookup[pkg.name] = _export_depends(pkg)

    for pkg_path, pkg in packages.items():
        # Add exported dependencies transitively
        dependencies = set()
        to_crawl_deps = _build_depends(pkg)
        while to_crawl_deps:
            dep = to_crawl_deps.pop()
            dependencies.add(dep)
            if dep.name in export_lookup:
                for transitive_dep in export_lookup[dep.name]:
                    if transitive_dep not in dependencies:
                        to_crawl_deps.append(transitive_dep)
            # else:
            #     output_to.write(f'# system dep? {dep.name}')
        
        build_type = pkg.get_build_type().lower()

        if build_type in ('cmake', 'ament_cmake'):
            _generate_cmake(pkg, dependencies, output_to)
        elif build_type in ('python', 'ament_python'):
            _generate_pip(pkg, dependencies, output_to)
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
