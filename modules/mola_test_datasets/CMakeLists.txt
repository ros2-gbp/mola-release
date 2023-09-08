# ------------------------------------------------------------------------------
#        A Modular Optimization framework for Localization and mApping
#                               (MOLA)
#
# Copyright (C) 2018-2023, Jose Luis Blanco-Claraco, contributors (AUTHORS.md)
# All rights reserved.
# Released under BSD 3-clause license. See LICENSE file
# ------------------------------------------------------------------------------

# Minimum CMake vesion: limited by CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS
cmake_minimum_required(VERSION 3.4)

if("$ENV{ROS_VERSION}" STREQUAL "2")
	set(DETECTED_ROS2 TRUE)
endif()

project(mola_test_datasets)

# Instrumental lib:
add_library(${PROJECT_NAME} INTERFACE)


# Install files:
install(DIRECTORY
  datasets
  DESTINATION 
    share/${PROJECT_NAME}
)

# -----------------------------------------------------------------------------
#  ROS2
# -----------------------------------------------------------------------------
if (DETECTED_ROS2)
	# find dependencies
	find_package(ament_cmake REQUIRED)

	#ament_target_dependencies(MY_TARGET
	#  rclcpp...
	#)

	ament_package()
endif()
