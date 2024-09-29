# fourier-grx-client

This is the client library for the Fourier GRX robot.
The correponding server library can be found [here](https://pypi.org/project/fourier-grx/0.1.1rc6/).

## Installation

> [!IMPORTANT]
> This software is highly unstable and subject to change at any time. Version 0.2.0 is only compatible with `fourier-grx` version `v1.0.0` and up.

### On the robot

On the robot, install `fourier-grx==1.0.0a8` following the instructions [here](https://github.com/FFTAI/Wiki-GRx-Deploy/tree/0.1.1rc6).

### On your local machine:

#### Install from pypi

Run:

```bash
pip install fourier-grx-client==0.2.0
```

#### Install from source

```bash
git clone https://github.com/FFTAI/fourier-grx-client.git
cd fourier-grx-client
pip install -e .
```

## Usage

Please read the [Tutorial](tutorial.ipynb) for a step-by-step guide on how to get started and use the interfaces.

Demo scripts can be found in the [example](example/) directory.

> [!TIP]
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
