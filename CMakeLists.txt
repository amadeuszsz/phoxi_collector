# Copyright 2022 Amadeusz Szymko
# Perception for Physical Interaction Laboratory at Poznan University of Technology
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
project(phoxi_collector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
add_compile_options(-std=c++17)

find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED COMPONENTS io)

ament_auto_find_build_dependencies(REQUIRED
    ${${PROJECT_NAME}_BUILD_DEPENDS}
    ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

set(PHOXI_COLLECTOR_NODE_SRC
    src/phoxi_collector_node.cpp
)

set(PHOXI_COLLECTOR_NODE_HEADERS
    include/phoxi_collector/phoxi_collector_node.hpp
    include/phoxi_collector/visibility_control.hpp
)

# generate component node library
ament_auto_add_library(phoxi_collector_node SHARED
    ${PHOXI_COLLECTOR_NODE_SRC}
    ${PHOXI_COLLECTOR_NODE_HEADERS}
)

rclcpp_components_register_node(phoxi_collector_node
    PLUGIN "phoxi_collector::PhoxiCollectorNode"
    EXECUTABLE phoxi_collector_node_exe
)

target_link_libraries(phoxi_collector_node
    ${OpenCV_LIBS} 
    ${PCL_LIBRARIES}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  param
)