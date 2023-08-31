# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/).

## Unreleased
## 0.3.0 - 2023-09-01
### Added
- Macro `ROTATE_LOCAL_MAP` that rotates the local map in order to keep x-axis in the direction of the car.
- Subscriber on `/path` that enables checking collisions on the Path. Obstacles are published to `/path/obstacles`.
- Parameter `~filter_map` that enables filtering of map from the LaserScan data.

### Changed
- ROS1 version is using `tf2` package.
- `tf2` dependency is shared for both ROS versions.
- Every cell on the static map with probability > 50 is treated as obstacle.
- Node starts after first valid transformation is found.

### Removed
- `tf` dependency for ROS1.

## 0.2.0 - 2023-06-13
### Added
- Publish local map to `/map/local`.
- ROS2 support.
- Parameters `~frame_id` and `~child_frame_id` for the transform listener.

## 0.1.1 - 2022-08-18
### Fixed
- Added missing `roscpp` dependency to the package manifest.

## 0.1.0 - 2022-08-18
### Added
- License file.

## 0.0.9 - 2022-08-18
### Added
- Readme file.
- Link to the new repository.

### Changed
- Updated package manifest format to version 3.
- Maintainer is now set to the community email.

### Deprecated
- Starting from v0.1.0 this package will be available only on GitHub. This is the last version here.

## 0.0.3 - 2021-03-24
### Added
- Remapping topic to the launch file.

### Fixed
- Frame `frame_id` received from Lidar is used as reference instead of static `laser`.
- When using simulated time, wait until it is truly published.

## 0.0.2 - 2021-03-16
### Fixed
- Fixed dependencies and dependency issues with `obstacle_msgs`.

## 0.0.1 - 2021-03-16
### Added
- First version of obstacle server package in C++.
