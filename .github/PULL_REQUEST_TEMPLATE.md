# Pull Request: LiDAR Mapping Bringup System

## Description

<!--
Provide a clear description of the changes in this pull request.
Explain the problem that this change solves and why it is needed.
If there are related issues, link them using the #123 format.
-->

### Motivation
<!-- Explain the motivation and context for this change -->

### Changes Made
<!-- List the specific changes made in this PR -->

---

## Type of Change

<!--
Check the boxes that apply to this PR.
This follows Conventional Commits format (feat, fix, docs, etc.).
-->

- [ ] **feat** - New feature
- [ ] **fix** - Bug fix
- [ ] **docs** - Documentation changes (no code changes)
- [ ] **style** - Code style changes (no functional changes: formatting, semicolons, etc.)
- [ ] **refactor** - Code refactoring (no functional changes)
- [ ] **perf** - Performance improvement
- [ ] **test** - Test additions or modifications
- [ ] **build** - Build system or external dependency changes
- [ ] **ci** - CI/CD configuration changes
- [ ] **chore** - Other maintenance tasks

---

## ROS 2 Specific Changes

<!-- Document ROS 2-specific changes in detail -->

### Package Dependencies
- [ ] New package dependencies added
  - Package: `<package_name>`
  - Reason: `<reason>`
  - Affects: `livox_ros_driver2`, `fast_lio`, `octomap_server`

### Topic/Service Changes
- [ ] New topics published/subscribed
  - Topic: `<topic_name>`, Type: `<message_type>`
- [ ] Topic structure modified
  - Topic: `<topic_name>`, Change: `<change_description>`

### Launch File Changes
- [ ] New launch parameters added
  - Parameter: `<param_name>`, Default: `<default_value>`
- [ ] Launch parameters removed or modified
  - Parameter: `<param_name>`, Change: `<change_description>`

### Configuration Changes
- [ ] Livox driver configuration modified (`livox_ros_driver2/config/MID360_config.json`)
- [ ] FAST-LIO SLAM configuration modified (`fast_lio/config/mid360.yaml`)
- [ ] OctoMap server configuration modified (launch parameters)

### Frame ID / Coordinate System Changes
- [ ] Fixed frame changed: `<old_frame>` → `<new_frame>`
- [ ] Other frame ID modifications

---

## Testing Checklist

<!--
Verify the items below before requesting review.
Refer to CLAUDE.md for testing command details.
-->

### Build & Compilation
- [ ] Package builds successfully: `colcon build --packages-select lidar_mapping_bringup`
- [ ] No build warnings (verify with `--verbose` flag)
- [ ] All dependencies installed: `rosdep install --from-paths . --ignore-src -r -y`

### Code Quality
- [ ] Linting checks pass: `colcon test --packages-select lidar_mapping_bringup`
- [ ] Pre-commit hooks pass: `pre-commit run --all-files`
- [ ] Python type hints validated

### Runtime Testing
- [ ] Launch file syntax verified: `ros2 launch lidar_mapping_bringup system_bringup.launch.py --show-args`
- [ ] System launches successfully: `ros2 launch lidar_mapping_bringup system_bringup.launch.py`
- [ ] Topics connect correctly (`ros2 topic list`, `ros2 node list`)
- [ ] RViz visualization works: `use_rviz:=true` parameter tested

### Component-Specific Tests
- [ ] Livox driver: Point cloud published to `/livox/lidar`
- [ ] FAST-LIO: Odometry and filtered cloud published (`/Odometry`, `/cloud_registered`)
- [ ] OctoMap: Occupancy maps published (`/octomap_binary`, `/projected_map`)

### Documentation
- [ ] README.md updated (if applicable)
- [ ] Launch parameter documentation updated (if changed)
- [ ] New configuration files documented (if applicable)

---

## Related Issues

<!-- Link any related issues or discussions -->

Fixes #<issue_number> (if applicable)
Related to #<issue_number> (if applicable)

---

## Additional Notes

<!-- Any additional information reviewers should know -->

### Breaking Changes
- [ ] This change breaks existing launch file compatibility
- [ ] This change modifies topic/service interfaces
- [ ] This change requires updates to dependent packages

### Performance Impact
- [ ] Memory usage changed
- [ ] CPU utilization changed
- [ ] Latency characteristics changed

### Screenshots / Logs
<!-- Attach RViz screenshots, error logs, performance metrics, etc. -->

---

## Reviewer Checklist

- [ ] Code follows the project's SOLID principles
- [ ] Changes follow Kent Beck's TDD and Tidy First methodology
- [ ] Commit messages are clear and follow Conventional Commits
- [ ] All tests pass
- [ ] Documentation is complete and accurate
- [ ] No unrelated changes are included

---

**Note**: Not all sections of this template are required.
Complete only the sections relevant to your changes.
