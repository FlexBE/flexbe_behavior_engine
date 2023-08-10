^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package flexbe_behavior_engine
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
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
