# Copyright 2025 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.8)
project(openarm_description)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(joint_axis_reflection
                        test/test_joint_axis_reflection.py)
  ament_add_pytest_test(component_interfaces test/test_component_interfaces.py)
  ament_add_pytest_test(v2_camera_components test/test_v2_camera_components.py)
  ament_add_pytest_test(v2_assembly test/test_v2_assembly.py TIMEOUT 180)
  ament_add_pytest_test(body_root_mapping test/test_body_root_mapping.py)
  ament_add_pytest_test(v1_mirroring test/test_v1_mirroring.py)
  ament_add_pytest_test(v1_assembly test/test_v1_assembly.py)
  ament_add_pytest_test(v1_mount_overrides test/test_v1_mount_overrides.py)
endif()

foreach(install_dir assets launch rviz scripts)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${install_dir}")
    install(DIRECTORY ${install_dir} DESTINATION share/${PROJECT_NAME})
  endif()
endforeach()

ament_package()
