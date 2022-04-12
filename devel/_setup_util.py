#!/usr/bin/python3
# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""This file generates shell code for the setup.SHELL scripts to set environment variables."""

from __future__ import print_function

import argparse
import copy
import errno
import os
import platform
import sys

CATKIN_MARKER_FILE = '.catkin'

system = platform.system()
IS_DARWIN = (system == 'Darwin')
IS_WINDOWS = (system == 'Windows')

PATH_TO_ADD_SUFFIX = ['bin']
if IS_WINDOWS:
    # while catkin recommends putting dll's into bin, 3rd party packages often put dll's into lib
    # since Windows finds dll's via the PATH variable, prepend it with path to lib
    PATH_TO_ADD_SUFFIX.extend([['lib', os.path.join('lib', 'aarch64-linux-gnu')]])

# subfolder of workspace prepended to CMAKE_PREFIX_PATH
ENV_VAR_SUBFOLDERS = {
    'CMAKE_PREFIX_PATH': '',
    'LD_LIBRARY_PATH' if not IS_DARWIN else 'DYLD_LIBRARY_PATH': ['lib', os.path.join('lib', 'aarch64-linux-gnu')],
    'PATH': PATH_TO_ADD_SUFFIX,
    'PKG_CONFIG_PATH': [os.path.join('lib', 'pkgconfig'), os.path.join('lib', 'aarch64-linux-gnu', 'pkgconfig')],
    'PYTHONPATH': 'lib/python3/dist-packages',
}


def rollback_env_variables(environ, env_var_subfolders):
    """
    Generate shell code to reset environment variables.

    by unrolling modifications based on all workspaces in CMAKE_PREFIX_PATH.
    This does not cover modifications performed by environment hooks.
    """
    lines = []
    unmodified_environ = copy.copy(environ)
    for key in sorted(env_var_subfolders.keys()):
        subfolders = env_var_subfolders[key]
        if not isinstance(subfolders, list):
            subfolders = [subfolders]
        value = _rollback_env_variable(unmodified_environ, key, subfolders)
        if value is not None:
            environ[key] = value
            lines.append(assignment(key, value))
    if lines:
        lines.insert(0, comment('reset environment variables by unrolling modifications based on all workspaces in CMAKE_PREFIX_PATH'))
    return lines


def _rollback_env_variable(environ, name, subfolders):
    """
    For each catkin workspace in CMAKE_PREFIX_PATH remove the first entry from env[NAME] matching workspace + subfolder.

    :param subfolders: list of str '' or subfoldername that may start with '/'
    :returns: the updated value of the environment variable.
    """
    value = environ[name] if name in environ else ''
    env_paths = [path for path in value.split(os.pathsep) if path]
    value_modified = False
    for subfolder in subfolders:
        if subfolder:
            if subfolder.startswith(os.path.sep) or (os.path.altsep and subfolder.startswith(os.path.altsep)):
                subfolder = subfolder[1:]
            if subfolder.endswith(os.path.sep) or (os.path.altsep and subfolder.endswith(os.path.altsep)):
                subfolder = subfolder[:-1]
        for ws_path in _get_workspaces(environ, include_fuerte=True, include_non_existing=True):
            path_to_find = os.path.join(ws_path, subfolder) if subfolder else ws_path
            path_to_remove = None
            for env_path in env_paths:
                env_path_clean = env_path[:-1] if env_path and env_path[-1] in [os.path.sep, os.path.altsep] else env_path
                if env_path_clean == path_to_find:
                    path_to_remove = env_path
                    break
            if path_to_remove:
                env_paths.remove(path_to_remove)
                value_modified = True
    new_value = os.pathsep.join(env_paths)
    return new_value if value_modified else None


def _get_workspaces(environ, include_fuerte=False, include_non_existing=False):
    """
    Based on CMAKE_PREFIX_PATH return all catkin workspaces.

    :param include_fuerte: The flag if paths starting with '/opt/ros/fuerte' should be considered workspaces, ``bool``
    """
    # get all cmake prefix paths
    env_name = 'CMAKE_PREFIX_PATH'
    value = environ[env_name] if env_name in environ else ''
    paths = [path for path in value.split(os.pathsep) if path]
    # remove non-workspace paths
    workspaces = [path for path in paths if os.path.isfile(os.path.join(path, CATKIN_MARKER_FILE)) or (include_fuerte and path.startswith('/opt/ros/fuerte')) or (include_non_existing and not os.path.exists(path))]
    return workspaces


def prepend_env_variables(environ, env_var_subfolders, workspaces):
    """Generate shell code to prepend environment variables for the all workspaces."""
    lines = []
    lines.append(comment('prepend folders of workspaces to environment variables'))

    paths = [path for path in workspaces.split(os.pathsep) if path]

    prefix = _prefix_env_variable(environ, 'CMAKE_PREFIX_PATH', paths, '')
    lines.append(prepend(environ, 'CMAKE_PREFIX_PATH', prefix))

    for key in sorted(key for key in env_var_subfolders.keys() if key != 'CMAKE_PREFIX_PATH'):
        subfolder = env_var_subfolders[key]
        prefix = _prefix_env_variable(environ, key, paths, subfolder)
        lines.append(prepend(environ, key, prefix))
    return lines


def _prefix_env_variable(environ, name, paths, subfolders):
    """
    Return the prefix to prepend to the environment variable NAME.

    Adding any path in NEW_PATHS_STR without creating duplicate or empty items.
    """
    value = environ[name] if name in environ else ''
    environ_paths = [path for path in value.split(os.pathsep) if path]
    checked_paths = []
    for path in paths:
        if not isinstance(subfolders, list):
            subfolders = [subfolders]
        for subfolder in subfolders:
            path_tmp = path
            if subfolder:
                path_tmp = os.path.join(path_tmp, subfolder)
            # skip nonexistent paths
            if not os.path.exists(path_tmp):
                continue
            # exclude any path already in env and any path we already added
            if path_tmp not in environ_paths and path_tmp not in checked_paths:
                checked_paths.append(path_tmp)
    prefix_str = os.pathsep.join(checked_paths)
    if prefix_str != '' and environ_paths:
        prefix_str += os.pathsep
    return prefix_str


def assignment(key, value):
    if not IS_WINDOWS:
        return 'export %s="%s"' % (key, value)
    else:
        return 'set %s=%s' % (key, value)


def comment(msg):
    if not IS_WINDOWS:
        return '# %s' % msg
    else:
        return 'REM %s' % msg


def prepend(environ, key, prefix):
    if key not in environ or not environ[key]:
        return assignment(key, prefix)
    if not IS_WINDOWS:
        return 'export %s="%s$%s"' % (key, prefix, key)
    else:
        return 'set %s=%s%%%s%%' % (key, prefix, key)


def find_env_hooks(environ, cmake_prefix_path):
    """Generate shell code with found environment hooks for the all workspaces."""
    lines = []
    lines.append(comment('found environment hooks in workspaces'))

    generic_env_hooks = []
    generic_env_hooks_workspace = []
    specific_env_hooks = []
    specific_env_hooks_workspace = []
    generic_env_hooks_by_filename = {}
    specific_env_hooks_by_filename = {}
    generic_env_hook_ext = 'bat' if IS_WINDOWS else 'sh'
    specific_env_hook_ext = environ['CATKIN_SHELL'] if not IS_WINDOWS and 'CATKIN_SHELL' in environ and environ['CATKIN_SHELL'] else None
    # remove non-workspace paths
    workspaces = [path for path in cmake_prefix_path.split(os.pathsep) if path and os.path.isfile(os.path.join(path, CATKIN_MARKER_FILE))]
    for workspace in reversed(workspaces):
        env_hook_dir = os.path.join(workspace, 'etc', 'catkin', 'profile.d')
        if os.path.isdir(env_hook_dir):
            for filename in sorted(os.listdir(env_hook_dir)):
                if filename.endswith('.%s' % generic_env_hook_ext):
                    # remove previous env hook with same name if present
                    if filename in generic_env_hooks_by_filename:
                        i = generic_env_hooks.index(generic_env_hooks_by_filename[filename])
                        generic_env_hooks.pop(i)
                        generic_env_hooks_workspace.pop(i)
                    # append env hook
                    generic_env_hooks.append(os.path.join(env_hook_dir, filename))
                    generic_env_hooks_workspace.append(workspace)
                    generic_env_hooks_by_filename[filename] = generic_env_hooks[-1]
                elif specific_env_hook_ext is not None and filename.endswith('.%s' % specific_env_hook_ext):
                    # remove previous env hook with same name if present
                    if filename in specific_env_hooks_by_filename:
                        i = specific_env_hooks.index(specific_env_hooks_by_filename[filename])
                        specific_env_hooks.pop(i)
                        specific_env_hooks_workspace.pop(i)
                    # append env hook
                    specific_env_hooks.append(os.path.join(env_hook_dir, filename))
                    specific_env_hooks_workspace.append(workspace)
                    specific_env_hooks_by_filename[filename] = specific_env_hooks[-1]
    env_hooks = generic_env_hooks + specific_env_hooks
    env_hooks_workspace = generic_env_hooks_workspace + specific_env_hooks_workspace
    count = len(env_hooks)
    lines.append(assignment('_CATKIN_ENVIRONMENT_HOOKS_COUNT', count))
    for i in range(count):
        lines.append(assignment('_CATKIN_ENVIRONMENT_HOOKS_%d' % i, env_hooks[i]))
        lines.append(assignment('_CATKIN_ENVIRONMENT_HOOKS_%d_WORKSPACE' % i, env_hooks_workspace[i]))
    return lines


def _parse_arguments(args=None):
    parser = argparse.ArgumentParser(description='Generates code blocks for the setup.SHELL script.')
    parser.add_argument('--extend', action='store_true', help='Skip unsetting previous environment variables to extend context')
    parser.add_argument('--local', action='store_true', help='Only consider this prefix path and ignore other prefix path in the environment')
    return parser.parse_known_args(args=args)[0]


if __name__ == '__main__':
    try:
        try:
            args = _parse_arguments()
        except Exception as e:
            print(e, file=sys.stderr)
            sys.exit(1)

        if not args.local:
            # environment at generation time
            CMAKE_PREFIX_PATH = r'/home/parallels/csda10f_motoman/devel;/home/parallels/demo02ws/devel;/home/parallels/motoman_CSDA10F/devel;/opt/ros/noetic;/home/parallels/ros2_foxy/install/rosbag2;/home/parallels/ros2_foxy/install/rosbag2_transport;/home/parallels/ros2_foxy/install/rosbag2_compression;/home/parallels/ros2_foxy/install/zstd_vendor;/home/parallels/ros2_foxy/install/rviz_visual_testing_framework;/home/parallels/ros2_foxy/install/rviz2;/home/parallels/ros2_foxy/install/rviz_default_plugins;/home/parallels/ros2_foxy/install/rviz_common;/home/parallels/ros2_foxy/install/rosbag2_storage_default_plugins;/home/parallels/ros2_foxy/install/rosbag2_converter_default_plugins;/home/parallels/ros2_foxy/install/bag_recorder_nodes;/home/parallels/ros2_foxy/install/rosbag2_cpp;/home/parallels/ros2_foxy/install/rosbag2_storage;/home/parallels/ros2_foxy/install/yaml_cpp_vendor;/home/parallels/ros2_foxy/install/ros1_bridge;/home/parallels/ros2_foxy/install/interactive_markers;/home/parallels/ros2_foxy/install/common_interfaces;/home/parallels/ros2_foxy/install/visualization_msgs;/home/parallels/ros2_foxy/install/dummy_robot_bringup;/home/parallels/ros2_foxy/install/robot_state_publisher;/home/parallels/ros2_foxy/install/kdl_parser;/home/parallels/ros2_foxy/install/urdf;/home/parallels/ros2_foxy/install/urdfdom;/home/parallels/ros2_foxy/install/urdfdom_headers;/home/parallels/ros2_foxy/install/turtlesim;/home/parallels/ros2_foxy/install/geometry2;/home/parallels/ros2_foxy/install/tf2_tools;/home/parallels/ros2_foxy/install/tf2_sensor_msgs;/home/parallels/ros2_foxy/install/test_tf2;/home/parallels/ros2_foxy/install/tf2_kdl;/home/parallels/ros2_foxy/install/tf2_geometry_msgs;/home/parallels/ros2_foxy/install/tf2_eigen;/home/parallels/ros2_foxy/install/tf2_bullet;/home/parallels/ros2_foxy/install/tf2_ros;/home/parallels/ros2_foxy/install/tf2_py;/home/parallels/ros2_foxy/install/tf2_msgs;/home/parallels/ros2_foxy/install/test_msgs;/home/parallels/ros2_foxy/install/sros2_cmake;/home/parallels/ros2_foxy/install/ros2cli_common_extensions;/home/parallels/ros2_foxy/install/rqt_console;/home/parallels/ros2_foxy/install/rqt_py_common;/home/parallels/ros2_foxy/install/ros_testing;/home/parallels/ros2_foxy/install/quality_of_service_demo_cpp;/home/parallels/ros2_foxy/install/demo_nodes_cpp;/home/parallels/ros2_foxy/install/composition;/home/parallels/ros2_foxy/install/rclpy;/home/parallels/ros2_foxy/install/examples_rclcpp_minimal_action_server;/home/parallels/ros2_foxy/install/examples_rclcpp_minimal_action_client;/home/parallels/ros2_foxy/install/action_tutorials_cpp;/home/parallels/ros2_foxy/install/rclcpp_action;/home/parallels/ros2_foxy/install/rcl_action;/home/parallels/ros2_foxy/install/move_base_msgs;/home/parallels/ros2_foxy/install/examples_rclcpp_minimal_service;/home/parallels/ros2_foxy/install/examples_rclcpp_minimal_client;/home/parallels/ros2_foxy/install/example_interfaces;/home/parallels/ros2_foxy/install/action_tutorials_interfaces;/home/parallels/ros2_foxy/install/action_msgs;/home/parallels/ros2_foxy/install/unique_identifier_msgs;/home/parallels/ros2_foxy/install/ament_lint_common;/home/parallels/ros2_foxy/install/ament_cmake_uncrustify;/home/parallels/ros2_foxy/install/uncrustify_vendor;/home/parallels/ros2_foxy/install/trajectory_msgs;/home/parallels/ros2_foxy/install/tracetools_test;/home/parallels/ros2_foxy/install/pendulum_control;/home/parallels/ros2_foxy/install/tlsf_cpp;/home/parallels/ros2_foxy/install/rqt_gui_cpp;/home/parallels/ros2_foxy/install/rosbag2_test_common;/home/parallels/ros2_foxy/install/ros2lifecycle_test_fixtures;/home/parallels/ros2_foxy/install/lifecycle;/home/parallels/ros2_foxy/install/rclcpp_lifecycle;/home/parallels/ros2_foxy/install/logging_demo;/home/parallels/ros2_foxy/install/image_tools;/home/parallels/ros2_foxy/install/examples_rclcpp_minimal_composition;/home/parallels/ros2_foxy/install/demo_nodes_cpp_native;/home/parallels/ros2_foxy/install/rclcpp_components;/home/parallels/ros2_foxy/install/laser_geometry;/home/parallels/ros2_foxy/install/intra_process_demo;/home/parallels/ros2_foxy/install/examples_rclcpp_multithreaded_executor;/home/parallels/ros2_foxy/install/examples_rclcpp_minimal_timer;/home/parallels/ros2_foxy/install/examples_rclcpp_minimal_subscriber;/home/parallels/ros2_foxy/install/examples_rclcpp_minimal_publisher;/home/parallels/ros2_foxy/install/dummy_sensors;/home/parallels/ros2_foxy/install/dummy_map_server;/home/parallels/ros2_foxy/install/rclcpp;/home/parallels/ros2_foxy/install/rcl_lifecycle;/home/parallels/ros2_foxy/install/libstatistics_collector;/home/parallels/ros2_foxy/install/rcl;/home/parallels/ros2_foxy/install/tracetools;/home/parallels/ros2_foxy/install/tlsf;/home/parallels/ros2_foxy/install/tinyxml_vendor;/home/parallels/ros2_foxy/install/qt_gui_core;/home/parallels/ros2_foxy/install/qt_gui_cpp;/home/parallels/ros2_foxy/install/pluginlib;/home/parallels/ros2_foxy/install/tinyxml2_vendor;/home/parallels/ros2_foxy/install/tf2_eigen_kdl;/home/parallels/ros2_foxy/install/tf2;/home/parallels/ros2_foxy/install/test_security;/home/parallels/ros2_foxy/install/test_rmw_implementation;/home/parallels/ros2_foxy/install/test_rclcpp;/home/parallels/ros2_foxy/install/test_quality_of_service;/home/parallels/ros2_foxy/install/test_launch_testing;/home/parallels/ros2_foxy/install/test_interface_files;/home/parallels/ros2_foxy/install/test_communication;/home/parallels/ros2_foxy/install/test_cli_remapping;/home/parallels/ros2_foxy/install/test_cli;/home/parallels/ros2_foxy/install/qt_gui_app;/home/parallels/ros2_foxy/install/qt_gui;/home/parallels/ros2_foxy/install/tango_icons_vendor;/home/parallels/ros2_foxy/install/stereo_msgs;/home/parallels/ros2_foxy/install/std_srvs;/home/parallels/ros2_foxy/install/shape_msgs;/home/parallels/ros2_foxy/install/map_msgs;/home/parallels/ros2_foxy/install/sensor_msgs;/home/parallels/ros2_foxy/install/nav_msgs;/home/parallels/ros2_foxy/install/diagnostic_msgs;/home/parallels/ros2_foxy/install/geometry_msgs;/home/parallels/ros2_foxy/install/actionlib_msgs;/home/parallels/ros2_foxy/install/std_msgs;/home/parallels/ros2_foxy/install/statistics_msgs;/home/parallels/ros2_foxy/install/sqlite3_vendor;/home/parallels/ros2_foxy/install/rcl_logging_spdlog;/home/parallels/ros2_foxy/install/spdlog_vendor;/home/parallels/ros2_foxy/install/shared_queues_vendor;/home/parallels/ros2_foxy/install/rviz_rendering_tests;/home/parallels/ros2_foxy/install/rviz_rendering;/home/parallels/ros2_foxy/install/rviz_ogre_vendor;/home/parallels/ros2_foxy/install/rviz_assimp_vendor;/home/parallels/ros2_foxy/install/rttest;/home/parallels/ros2_foxy/install/rosgraph_msgs;/home/parallels/ros2_foxy/install/rmw_implementation;/home/parallels/ros2_foxy/install/rmw_fastrtps_dynamic_cpp;/home/parallels/ros2_foxy/install/rmw_fastrtps_cpp;/home/parallels/ros2_foxy/install/rmw_fastrtps_shared_cpp;/home/parallels/ros2_foxy/install/rmw_cyclonedds_cpp;/home/parallels/ros2_foxy/install/rmw_dds_common;/home/parallels/ros2_foxy/install/composition_interfaces;/home/parallels/ros2_foxy/install/rcl_interfaces;/home/parallels/ros2_foxy/install/pendulum_msgs;/home/parallels/ros2_foxy/install/lifecycle_msgs;/home/parallels/ros2_foxy/install/builtin_interfaces;/home/parallels/ros2_foxy/install/rosidl_default_runtime;/home/parallels/ros2_foxy/install/rosidl_default_generators;/home/parallels/ros2_foxy/install/rosidl_generator_py;/home/parallels/ros2_foxy/install/rosidl_typesupport_cpp;/home/parallels/ros2_foxy/install/rosidl_typesupport_introspection_cpp;/home/parallels/ros2_foxy/install/rosidl_typesupport_c;/home/parallels/ros2_foxy/install/rosidl_typesupport_introspection_c;/home/parallels/ros2_foxy/install/rosidl_typesupport_fastrtps_c;/home/parallels/ros2_foxy/install/rosidl_typesupport_fastrtps_cpp;/home/parallels/ros2_foxy/install/rmw_connext_cpp;/home/parallels/ros2_foxy/install/rosidl_typesupport_connext_c;/home/parallels/ros2_foxy/install/rosidl_typesupport_connext_cpp;/home/parallels/ros2_foxy/install/rmw;/home/parallels/ros2_foxy/install/rosidl_runtime_c;/home/parallels/ros2_foxy/install/rosidl_generator_cpp;/home/parallels/ros2_foxy/install/rosidl_generator_c;/home/parallels/ros2_foxy/install/rosidl_typesupport_interface;/home/parallels/ros2_foxy/install/rosidl_runtime_cpp;/home/parallels/ros2_foxy/install/rosidl_generator_dds_idl;/home/parallels/ros2_foxy/install/rosidl_cmake;/home/parallels/ros2_foxy/install/rosidl_parser;/home/parallels/ros2_foxy/install/rosidl_adapter;/home/parallels/ros2_foxy/install/rosbag2_tests;/home/parallels/ros2_foxy/install/ros_environment;/home/parallels/ros2_foxy/install/rmw_implementation_cmake;/home/parallels/ros2_foxy/install/rmw_connext_shared_cpp;/home/parallels/ros2_foxy/install/resource_retriever;/home/parallels/ros2_foxy/install/class_loader;/home/parallels/ros2_foxy/install/rcpputils;/home/parallels/ros2_foxy/install/rcl_logging_noop;/home/parallels/ros2_foxy/install/rcl_logging_log4cxx;/home/parallels/ros2_foxy/install/rcutils;/home/parallels/ros2_foxy/install/rcl_yaml_param_parser;/home/parallels/ros2_foxy/install/qt_gui_py_common;/home/parallels/ros2_foxy/install/qt_dotgraph;/home/parallels/ros2_foxy/install/python_qt_binding;/home/parallels/ros2_foxy/install/launch_testing_ament_cmake;/home/parallels/ros2_foxy/install/python_cmake_module;/home/parallels/ros2_foxy/install/performance_test_fixture;/home/parallels/ros2_foxy/install/osrf_testing_tools_cpp;/home/parallels/ros2_foxy/install/orocos_kdl;/home/parallels/ros2_foxy/install/mimick_vendor;/home/parallels/ros2_foxy/install/message_filters;/home/parallels/ros2_foxy/install/libyaml_vendor;/home/parallels/ros2_foxy/install/libcurl_vendor;/home/parallels/ros2_foxy/install/ament_cmake_ros;/home/parallels/ros2_foxy/install/ament_cmake_gmock;/home/parallels/ros2_foxy/install/gmock_vendor;/home/parallels/ros2_foxy/install/ament_cmake_gtest;/home/parallels/ros2_foxy/install/gtest_vendor;/home/parallels/ros2_foxy/install/ament_cmake_google_benchmark;/home/parallels/ros2_foxy/install/google_benchmark_vendor;/home/parallels/ros2_foxy/install/fastrtps;/home/parallels/ros2_foxy/install/foonathan_memory_vendor;/home/parallels/ros2_foxy/install/fastrtps_cmake_module;/home/parallels/ros2_foxy/install/fastcdr;/home/parallels/ros2_foxy/install/eigen3_cmake_module;/home/parallels/ros2_foxy/install/cyclonedds;/home/parallels/ros2_foxy/install/console_bridge_vendor;/home/parallels/ros2_foxy/install/connext_cmake_module;/home/parallels/ros2_foxy/install/ament_cmake_xmllint;/home/parallels/ros2_foxy/install/ament_cmake_pyflakes;/home/parallels/ros2_foxy/install/ament_cmake_pycodestyle;/home/parallels/ros2_foxy/install/ament_cmake_pep257;/home/parallels/ros2_foxy/install/ament_cmake_pclint;/home/parallels/ros2_foxy/install/ament_lint_auto;/home/parallels/ros2_foxy/install/ament_cmake_auto;/home/parallels/ros2_foxy/install/ament_cmake;/home/parallels/ros2_foxy/install/ament_cmake_version;/home/parallels/ros2_foxy/install/ament_cmake_pytest;/home/parallels/ros2_foxy/install/ament_cmake_nose;/home/parallels/ros2_foxy/install/ament_cmake_mypy;/home/parallels/ros2_foxy/install/ament_cmake_lint_cmake;/home/parallels/ros2_foxy/install/ament_cmake_flake8;/home/parallels/ros2_foxy/install/ament_cmake_cpplint;/home/parallels/ros2_foxy/install/ament_cmake_cppcheck;/home/parallels/ros2_foxy/install/ament_cmake_copyright;/home/parallels/ros2_foxy/install/ament_cmake_clang_tidy;/home/parallels/ros2_foxy/install/ament_cmake_clang_format;/home/parallels/ros2_foxy/install/ament_cmake_test;/home/parallels/ros2_foxy/install/ament_cmake_target_dependencies;/home/parallels/ros2_foxy/install/ament_cmake_python;/home/parallels/ros2_foxy/install/ament_cmake_export_dependencies;/home/parallels/ros2_foxy/install/ament_cmake_libraries;/home/parallels/ros2_foxy/install/ament_cmake_include_directories;/home/parallels/ros2_foxy/install/ament_cmake_export_targets;/home/parallels/ros2_foxy/install/ament_cmake_export_link_flags;/home/parallels/ros2_foxy/install/ament_cmake_export_interfaces;/home/parallels/ros2_foxy/install/ament_cmake_export_libraries;/home/parallels/ros2_foxy/install/ament_cmake_export_include_directories;/home/parallels/ros2_foxy/install/ament_cmake_export_definitions;/home/parallels/ros2_foxy/install/ament_cmake_core;/home/parallels/ros2_foxy/install/ament_index_cpp'.split(';')
        else:
            # don't consider any other prefix path than this one
            CMAKE_PREFIX_PATH = []
        # prepend current workspace if not already part of CPP
        base_path = os.path.dirname(__file__)
        # CMAKE_PREFIX_PATH uses forward slash on all platforms, but __file__ is platform dependent
        # base_path on Windows contains backward slashes, need to be converted to forward slashes before comparison
        if os.path.sep != '/':
            base_path = base_path.replace(os.path.sep, '/')

        if base_path not in CMAKE_PREFIX_PATH:
            CMAKE_PREFIX_PATH.insert(0, base_path)
        CMAKE_PREFIX_PATH = os.pathsep.join(CMAKE_PREFIX_PATH)

        environ = dict(os.environ)
        lines = []
        if not args.extend:
            lines += rollback_env_variables(environ, ENV_VAR_SUBFOLDERS)
        lines += prepend_env_variables(environ, ENV_VAR_SUBFOLDERS, CMAKE_PREFIX_PATH)
        lines += find_env_hooks(environ, CMAKE_PREFIX_PATH)
        print('\n'.join(lines))

        # need to explicitly flush the output
        sys.stdout.flush()
    except IOError as e:
        # and catch potential "broken pipe" if stdout is not writable
        # which can happen when piping the output to a file but the disk is full
        if e.errno == errno.EPIPE:
            print(e, file=sys.stderr)
            sys.exit(2)
        raise

    sys.exit(0)
