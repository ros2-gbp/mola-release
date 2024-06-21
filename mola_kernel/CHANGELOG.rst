^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mola_kernel
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.5 (2024-05-28)
------------------
* viz: fix mismatched free/delete inside nanogui layout
* Contributors: Jose Luis Blanco-Claraco

1.0.4 (2024-05-14)
------------------
* bump cmake_minimum_required to 3.5
* Avoid global static objects
* remove useless #include's
* Define Dataset_UI dtor/ctor in a separate translation unit
* Contributors: Jose Luis Blanco-Claraco

1.0.3 (2024-04-22)
------------------
* Fix package.xml website URL
* Contributors: Jose Luis Blanco-Claraco

1.0.2 (2024-04-04)
------------------

1.0.1 (2024-03-28)
------------------
* Remove now-useless build dependencies and includes for mola-kernel
* Contributors: Jose Luis Blanco-Claraco

1.0.0 (2024-03-19)
------------------
* add methods to query for subscribers
* New interfaces
* Refactor initialize()
* mola_kernel: new UI interface for datasets
* New option to shutdown automatically mola-cli after dataset ends
* viz API: add enqueue_custom_nanogui_code()
* mola_viz: show console messages
* Correct usage of mola:: namespace in cmake targets
* copyright update
* mola_viz: support visualizing velodyne observations
* Add look_at() viz interface
* Fewer mutex locking()
* dont force by default load() lazy-load observations
* FrontEndBase: attach to VizInterface too
* Fix loss of yaml key/values when using import-from-file feature
* kitti eval cli moves to its own package
* port to mrpt::lockHelper()
* reorganize as monorepo
* Contributors: Jose Luis Blanco-Claraco

0.2.2 (2023-09-08)
------------------
* Correct references to the license.
* viz interface: new service update_3d_object()
* Fix const-correctness of observations
* FIX missing dependency on mrpt::gui for public header
* Contributors: Jose Luis Blanco-Claraco

0.2.1 (2023-09-02)
------------------

* Add virtual interface for dataset groundtruth
* Update copyright date
* Update to new colcon ROS2 build system
* Contributors: Jose Luis Blanco-Claraco
