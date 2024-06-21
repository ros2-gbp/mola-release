^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mola_input_rosbag2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


1.0.5 (2024-05-28)
------------------

1.0.4 (2024-05-14)
------------------
* bump cmake_minimum_required to 3.5
* Contributors: Jose Luis Blanco-Claraco

1.0.3 (2024-04-22)
------------------
* FIXBUG: inverse sensor poses in rosbag2 reader.
  Also: unify notation in C++ calls to lookupTransform()
* Fix package.xml website URL
* Contributors: Jose Luis Blanco-Claraco

1.0.2 (2024-04-04)
------------------

1.0.1 (2024-03-28)
------------------
* BUGFIX: Inverted value of "use_fixed_sensor_pose" was used
* Contributors: Jose Luis Blanco-Claraco

1.0.0 (2024-03-19)
------------------
* tolerate missing tf data temporarily
* Merge ROS2 input and output in one module
* Add use_fixed_sensor_pose flag
* Fix ROS2 sensorPose from /tf data
* Fix jump forward in time in rosbags
* Implement UI Dataset in the rest of dataset sources
* New option to shutdown automatically mola-cli after dataset ends
* Automatic determination of storage id from file extension
* Show dataset replay progress percent
* Correct usage of mola:: namespace in cmake targets
* copyright update
* Hide ros2 dependencies from public .h
* Contributors: Jose Luis Blanco-Claraco

0.2.2 (2023-09-08)
------------------