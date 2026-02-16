# Repository Guidelines

## Project Structure & Module Organization
This repository contains two ROS 2 packages:
- `greenwave_monitor/`: main package with C++ nodes (`src/`, `include/`), Python dashboard modules (`greenwave_monitor/greenwave_monitor/`), launch files (`launch/`), config (`config/`), examples (`examples/`), and tests (`test/`).
- `greenwave_monitor_interfaces/`: ROS 2 service definitions in `srv/` and interface build metadata.

Supporting files live at the root: `README.md`, `Contributing.md`, `.pre-commit-config.yaml`, and helper scripts in `scripts/`.

## Build, Test, and Development Commands
Use a ROS 2 workspace root (one level above this repo) for build/test commands.
- `colcon build --packages-up-to greenwave_monitor`: build monitor and required interfaces.
- `colcon test --packages-select greenwave_monitor && colcon test-result --verbose`: run and inspect tests.
- `ros2 run greenwave_monitor ncurses_dashboard --demo`: run the dashboard with demo publishers.
- `ros2 launch greenwave_monitor hz.launch.py gw_monitored_topics:='["/topic1", "/topic2"]'`: launch topic monitoring.
- `pre-commit run --all-files`: run formatting, lint, copyright, spelling, and sign-off checks locally.

## Coding Style & Naming Conventions
- C++ targets use C++17 with warnings enabled (`-Wall -Wextra -Wpedantic`).
- Format C/C++ with `uncrustify` using `ament_code_style.cfg` (wired into pre-commit).
- Python style uses `autopep8` and `flake8` (max line length 99).
- Prefer snake_case for Python modules/functions and descriptive ROS node/topic names.
- Keep headers in `include/`, implementations in `src/`, and place tests under `test/test_*.py` or `test/test_*.cpp`.

## Testing Guidelines
The package uses:
- `ament_cmake_pytest` for Python and integration tests.
- `ament_cmake_gtest` for C++ unit tests.
- `ament_lint_auto` for linting/copyright checks.

Name tests with `test_` prefix and keep scope narrow (single behavior per test file where practical). Run all package tests before opening a PR.

## Commit & Pull Request Guidelines
- Follow existing history style: imperative subject line, concise scope (example: `Fix README smoke tests in CI (#26)`).
- Sign off every commit (`git commit -s ...`); unsigned commits are not accepted (DCO requirement).
- PRs should include: purpose, key changes, test evidence (`colcon test` or equivalent), and linked issue(s) when applicable.
- Include screenshots or terminal output snippets for dashboard/UI behavior changes.
