^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package flexbe_behavior_engine
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.0.1 (2024-09-26)
------------------

4.0.0 (2024-08-24)
------------------
* Version 4.0.0 release using state_id for communication
* this breaks API with flexbe_app and requires version 4.1.0+ of the FlexBE WebUI API

3.0.7 (2024-08-24)
------------------

3.0.6 (2024-08-05)
------------------

3.0.5 (2024-07-02)
------------------

3.0.4 (2024-07-02)
------------------

3.0.3 (2024-06-06)
------------------
* Update preparing for Jazzy release

3.0.2 (2024-06-04)
------------------

3.0.1 (2024-05-31)
------------------

3.0.0 (2024-05-01)
------------------
* update with state map and changes to concurrent handling
* allow removing action clients and service callers
* flake 8 cleanup
* support state id and concurrency handling
* specify version 4.0+ of the FlexBE UI
* add link for new flexbe_webui

2.3.4 (2024-05-01)
------------------
* update CMakeLists cmake_minimum_version
* Updates to dependencies for ROS build farm

2.3.3 (2023-08-09)
------------------

2.3.2 (2023-08-01)
------------------

2.3.1 (2023-07-31)
------------------
* Update flexbe_ci.yml for actions/checkout@v3

2.3.0 (2023-07-20)
------------------
* update CMakeLists cmake_minimum_version
* Updates to dependencies for ROS build farm

2.2.0 (2023-06-29)
------------------
* [flexbe_core, flexbe_onboard, flexbe_mirror, flexbe_widget]
  - Use behavior_key and behavior_id for consistency
  - rework mirror and be_launcher to prevent deadlocks
  - add heartbeat messages to mirror and launcher
  - pep257 and flake8 cleanup
* [flexbe_msgs] Add comments and modify BehaviorSelection message to use
  behavior_key and behavior_id for consistency with other messages
* update README links; tweak LICENSE and package info
* Modify shutdown handling for clean stop
* Modify wait() handling to avoid creating a rate object

2.1.0 (2022-08-02)
------------------
* ROS 2 Humble release
* Includes changes sync check handling
* Add formatted traceback to local log for behavior errors
* Tested under Ubuntu 22.04 and ROS Humble

2.0.0 (2022-02-22)
------------------
* Initial ROS 2 "foxy" release based on ROS 1 commit a343c657
* Includes changes to concurrent state and sleep handling

1.3.1 (2020-12-11)
------------------

1.3.0 (2020-11-19)
------------------

1.2.5 (2020-06-14)
------------------
* Merge branch 'develop' into feature/state_logger_rework
* Contributors: Philipp Schillinger

1.2.4 (2020-03-25)
------------------

1.2.3 (2020-01-10)
------------------
* Merge remote-tracking branch 'origin/develop' into feature/test_behaviors
  # Conflicts:
  #	flexbe_testing/bin/testing_node
  #	flexbe_testing/src/flexbe_testing/state_tester.py
* Contributors: Philipp Schillinger

1.2.2 (2019-09-16)
------------------

1.2.1 (2019-06-02)
------------------
* Merge remote-tracking branch 'origin/feature/sub_parameters' into develop
* Bump required flexbe_app version
* Merge remote-tracking branch 'origin/master' into develop
* Contributors: Philipp Schillinger

1.1.2 (2019-04-09)
------------------
* Merge remote-tracking branch 'origin/master' into develop
* Contributors: Philipp Schillinger

1.1.1 (2018-12-18)
------------------
* Merge remote-tracking branch 'origin/master' into develop
* Contributors: Philipp Schillinger

1.1.0 (2018-12-01)
------------------
* Merge branch 'develop'
* Merge branch 'feature/flexbe_app' into develop
* Update maintainer information
* Merge pull request `#55 <https://github.com/team-vigir/flexbe_behavior_engine/issues/55>`_ from alireza-hosseini/add-metapackage
  feat: Add flexbe_behavior_engine metapackage
* feat: Add flexbe_behavior_engine metapackage
* Contributors: Philipp Schillinger, alireza
