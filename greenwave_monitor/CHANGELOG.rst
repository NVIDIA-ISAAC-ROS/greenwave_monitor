^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package greenwave_monitor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2026-02-27)
------------------
* Fix flakey minimal publisher shutdown (`#36 <https://github.com/NVIDIA-ISAAC-ROS/greenwave_monitor/issues/36>`_)
* Fix errors overlaying on topics in ncurses frontend (`#35 <https://github.com/NVIDIA-ISAAC-ROS/greenwave_monitor/issues/35>`_)
  Logs are now directed to a logfile instead of the screen by default.
  ---------
* Fix issue `#23 <https://github.com/NVIDIA-ISAAC-ROS/greenwave_monitor/issues/23>`_: prevent duplicate diagnostics when external node is already publishing (`#33 <https://github.com/NVIDIA-ISAAC-ROS/greenwave_monitor/issues/33>`_)
  Fixes `#23 <https://github.com/NVIDIA-ISAAC-ROS/greenwave_monitor/issues/23>`_.
  Subscribe to /diagnostics in GreenwaveMonitor to track topic names already published externally. add_topic() now returns an error if a topic's diagnostics are already being published by an external node, preventing duplicate and potentially conflicting diagnostic entries.
  ----
* Unify timestamp checks and fix headerless FPS checks (`#31 <https://github.com/NVIDIA-ISAAC-ROS/greenwave_monitor/issues/31>`_)
  Fixes `#18 <https://github.com/NVIDIA-ISAAC-ROS/greenwave_monitor/issues/18>`_ . Also migrates the time check logic to one shared class configured with flags. This unifies logic between node and message time allowing for them to both verify similar failure cases. The current checks that can be enabled are:
  checkFpsJitter: the normal check previously implemented where we produce diagnostic errors if the time between frames is ever outside of the FPS tolerance
  checkFpsWindow: a new check that uses the average frame rate from the FPS window to evaluate if it is out of range. This is useful for timestamps that have high variance and may trigger the FpsJitter check frequently.
  checkIncreasing: same check from before but also being applied to node time now too. Makes sure time is always increasing.
  Alongside the new checks there is a ROS parameter gw_time_check_preset. This parameter accepts presets HeaderWithFallback, HeaderOnly, NodeOnly, and None. The HeaderWithFallback configures the tests to use message time when a message timestamp exists and node time when it does not. None does not change the configuration and allows the user to fully specify the checks they want to be enabled. The *Only presets enable flags specifically for Header or Node time.
* Remove ament_target_dependencies to match ROS 2 Rolling (`#32 <https://github.com/NVIDIA-ISAAC-ROS/greenwave_monitor/issues/32>`_)
  Apparently ROS 2 on rolling is fully deprecating ament_target_dependencies 😮 `#572 <https://github.com/NVIDIA-ISAAC-ROS/greenwave_monitor/issues/572>`_
  Update the Cmakelist to do the same. Recently in Jan 2026 it was fully removed. The post I linked is the reasons for doing it.
  ---
* Add more docs and a clean integration example publisher. (`#29 <https://github.com/NVIDIA-ISAAC-ROS/greenwave_monitor/issues/29>`_)
  Adds docs for how to inline greenwave diagnostics into a node. Include clean example node, add an agents.md file.
* Add pre-commit for better linting (`#25 <https://github.com/NVIDIA-ISAAC-ROS/greenwave_monitor/issues/25>`_)
  Adds a pre-commit config that uses uncrustify and flake8 just like ament. ament isn't used directly since I wanted formatting to occur outside of a ROS environment when making small fixes. Also has other tools like whitespace, newline, copyright, yaml checks, code spelling, shell script formatting, and commit sign off checking.
  Useful for fixing most issues with linting before it is pushed. Automatically reformats your commits, fixes copyright year issues, etc.
* Use parameter yaml on startup for configuring greenwave monitor + rename to greenwave diagnostics (`#22 <https://github.com/NVIDIA-ISAAC-ROS/greenwave_monitor/issues/22>`_)
  Add support for ROS parameter YAML on startup and rename message diagnostics to greenwave diagnostics to match parameter yaml group name. Look at the example.yaml if you want to see how it is integrated.
  Fixes a few bugs/makes the system robust also. Specifically `#16 <https://github.com/NVIDIA-ISAAC-ROS/greenwave_monitor/issues/16>`_ .
* Add --hide-unmonitored flag to ncurses frontend (`#20 <https://github.com/NVIDIA-ISAAC-ROS/greenwave_monitor/issues/20>`_)
  Add a --hide-unmonitored flag to the ncurses frontend. This makes it possible for the user to start greenwave monitor with only the topics that are being monitored visible.
  Tests were also added to verify the argument parsing is functional with the frontend.
* Relax test tolerances to avoid flaky tests in buildfarm.  (`#17 <https://github.com/NVIDIA-ISAAC-ROS/greenwave_monitor/issues/17>`_)
* Make r2s_gw an optional dependency  (`#13 <https://github.com/NVIDIA-ISAAC-ROS/greenwave_monitor/issues/13>`_)
  r2s_gw requires a version of textual that the ubuntu noble Debians don't have, which makes getting into ros2 build farm annoying, so just make it it's own repo that is optional, and use curses UI as the primary.
  Move r2s_gw into seperate repo, r2s_gw depends on greenwave_monitor, but no dependencies in the other direction.
* Fix mixing ament_cmake_auto rosdep, update CI to use base images rather than ros-core images
* replace large latency values with N/A, add latency note to README (`#11 <https://github.com/NVIDIA-ISAAC-ROS/greenwave_monitor/issues/11>`_)
  * add N/A when latency is nonsense
  * minor fixup to make types in messaage_diagnostics consistent
  * update readme and update tests
* Contributors: Sean Gillen, bmchalenv

0.1.0 (2025-10-09)
------------------
* Update package.xml to add authors tag (`#9 <https://github.com/NVIDIA-ISAAC-ROS/greenwave_monitor/issues/9>`_)
* Change logic for UI adaptor to still work with NITROS (`#7 <https://github.com/NVIDIA-ISAAC-ROS/greenwave_monitor/issues/7>`_)
* Fixup headers, maintainers, README, lint, before public release (`#5 <https://github.com/NVIDIA-ISAAC-ROS/greenwave_monitor/issues/5>`_)
* Add nccurses based frontend (`#3 <https://github.com/NVIDIA-ISAAC-ROS/greenwave_monitor/issues/3>`_)
  Adds a simple curses based frontend, less featureful, but more performant with large number of topics than r2s_gw
* Initial Commit
* Contributors: Sean Gillen
