^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mola_input_kitti_dataset
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.3 (2024-08-28)
------------------
* Depend on new mrpt_lib packages (deprecate mrpt2)
* Contributors: Jose Luis Blanco-Claraco

1.1.2 (2024-08-26)
------------------

1.1.1 (2024-08-23)
------------------

1.1.0 (2024-08-18)
------------------
* Merge pull request `#62 <https://github.com/MOLAorg/mola/issues/62>`_ from MOLAorg/docs-fixes
  Docs fixes
* Fix ament_xmllint warnings in package.xml
* Contributors: Jose Luis Blanco-Claraco

1.0.8 (2024-07-29)
------------------
* ament_lint_cmake: clean warnings
* Contributors: Jose Luis Blanco-Claraco

1.0.7 (2024-07-24)
------------------

1.0.6 (2024-06-21)
------------------

1.0.5 (2024-05-28)
------------------

1.0.4 (2024-05-14)
------------------
* bump cmake_minimum_required to 3.5
* Contributors: Jose Luis Blanco-Claraco

1.0.3 (2024-04-22)
------------------
* kitti dataset module: show correct module name in log messages
* Fix package.xml website URL
* Contributors: Jose Luis Blanco-Claraco

1.0.2 (2024-04-04)
------------------
* Add docs on expected KITTI dataset layout
* Contributors: Jose Luis Blanco-Claraco

1.0.1 (2024-03-28)
------------------

1.0.0 (2024-03-19)
------------------
* Refactor initialize()
* implement UI dataset
* New option to shutdown automatically mola-cli after dataset ends
* save useless memory in offline dataset access mode
* Correct usage of mola:: namespace in cmake targets
* copyright update
* kitti eval cli moves to its own package
* mola-kitti-eval-error: allow comparing against other custom GT files
* reorganize for monorepo
* Contributors: Jose Luis Blanco-Claraco

0.2.2 (2023-09-08)
------------------
* Correct references to license
* Fix published ground truth axis of reference
* Fix wrong ground truth matrix indexing.
* Contributors: Jose Luis Blanco-Claraco

0.2.1 (2023-09-02)
------------------

* Implement ground-truth interface for KITTI
* Update copyright date
* Update to new colcon ROS2 build system
* Contributors: Jose Luis Blanco-Claraco
