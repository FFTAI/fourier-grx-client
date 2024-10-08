# fourier-grx-client

This is the client library for the Fourier GRX robot.
The correponding server library can be found [here](https://pypi.org/project/fourier-grx/1.0.0a13/).

## ❗ Breaking Changes

For users who have been using Wiki-GRx-Deploy and those who are new to this library, Please note the following breaking changes:

- Wiki-GRx-Deploy has been deprecated and is no longer supported.
- All user-facing interfaces now default to use **radians** instead of degrees.
- URDF now loads from [Wiki-GRx-Models](https://github.com/FFTAI/Wiki-GRx-Models) via [fourier-robot-descriptions](https://pypi.org/project/fourier-robot-descriptions/) package.
- Nameing convention for all links and joints are now consistent with the URDF.
- Config file structure has been updated to match the latest version of the server.
Default config file can be found under [config](config/) directory.

## Installation


> **This software is highly unstable and subject to change at any time. Version 0.2.0a5 is only compatible with `fourier-grx` version `v1.0.0a13` and up.**

### On the robot computer:

On the robot, install `fourier-grx==1.0.0a13` following the [Installation Instructions](docs/Installation.md).

### On other machines:

**Grx client will be automatically installed when installing `fourier-grx` on the robot.**

If you want to control the robot from another machine, you can install the client library using the following methods:

#### Install from pypi

Run:

```bash
pip install fourier-grx-client==0.2.0a5

```

#### Install from source

```bash
git clone https://github.com/FFTAI/fourier-grx-client.git
cd fourier-grx-client
pip install -e .
```

## Usage

Please read the [Tutorial](Tutorial.ipynb) for a step-by-step guide on how to get started and use the interfaces.

Demo scripts can be found in the [examples](examples/) directory.


> For more information on the API, see the [API Reference](https://fftai.github.io/fourier-grx-client/latest/reference/api/).


## Development

1. Install the `pdm` package manager:

```bash
pip install pdm
```

2. use `pdm` to install the package and all development dependencies:

```bash
pdm install -v -d
```

3. Install pre-commit hooks:

```bash
pre-commit install
```
