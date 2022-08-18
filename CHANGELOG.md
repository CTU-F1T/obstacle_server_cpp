# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/).

## Unreleased
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
