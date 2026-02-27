^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package greenwave_monitor_interfaces
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Add pre-commit for better linting (`#25 <https://github.com/NVIDIA-ISAAC-ROS/greenwave_monitor/issues/25>`_)
  Adds a pre-commit config that uses uncrustify and flake8 just like ament. ament isn't used directly since I wanted formatting to occur outside of a ROS environment when making small fixes. Also has other tools like whitespace, newline, copyright, yaml checks, code spelling, shell script formatting, and commit sign off checking.
  Useful for fixing most issues with linting before it is pushed. Automatically reformats your commits, fixes copyright year issues, etc.
* Contributors: bmchalenv

0.1.0 (2025-10-09)
------------------
* Update package.xml to add authors tag (`#9 <https://github.com/NVIDIA-ISAAC-ROS/greenwave_monitor/issues/9>`_)
* Fixup headers, maintainers, README, lint, before public release (`#5 <https://github.com/NVIDIA-ISAAC-ROS/greenwave_monitor/issues/5>`_)
* Initial Commit
* Contributors: Sean Gillen
