## [v0.2.0] - 2024-09-29

### ❗ Breaking Changes

- All user-facing interfaces now default to use radians instead of degrees.
- URDF now loads from [Wiki-GRx-Models](https://github.com/FFTAI/Wiki-GRx-Models) via [fourier-robot-descriptions](https://pypi.org/project/fourier-robot-descriptions/) package.
- Nameing convention for all links and joints are now consistent with the URDF.
- Config file structure changed

### 🚀 Features

- Robot namespace support in both CLI and API
- `control_joints` API for velocity mode and current mode
- forward kinematcs and inverse kinematics API
- Add gravity compensation option to `move_joints`
- `movej` API


### 💪 Enhancements

- Add `py.typed` file to package
- Migrated all examples from [iki-grx-deploy](https://gitee.com/FourierIntelligence/wiki-grx-deploy)
- Jupyter notebook tutorial

### 🐛 Bug Fixes

- Disabled zenoh broadcast by default to avoid clashes with other zenoh clients

## [v0.1.5] - 2024-09-11

### 💪 Enhancements

- Fortify service call handling in `ZenohSession`

### 🐛 Bug Fixes

- Rename `zenoh.py` to `zenoh_utils.py` to avoid name clashing


## [v0.1.4] - 2024-08-30

### 💪 Enhancements

- Add `degrees` option to `move_joints`
- Better documentation for `move_joints`

## [v0.1.3] - 2024-08-30

### 💪 Enhancements

- Explicitly define exports in `__init__.py`

## [v0.1.2] - 2024-08-30

### 🚀 Features

- Support `fourier-grx` version `v0.1.1-rc.6`

## [v0.1.1] - 2024-08-30

### 🐛 Bug Fixes

- Fix typo in `RobotClient`
- Allow multicast for compatibility

## [v0.1.0] - 2024-08-30

### 🚀 Features

- Support python >= 3.8
- Support `fourier-grx` version `v0.1.1-rc.3`
- Allow addressing control groups by name in `move_joints`
