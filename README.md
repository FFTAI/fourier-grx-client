# fourier-grx-client

This is the client library for the Fourier GRX robot. 
The correponding server library can be found [here](https://pypi.org/project/fourier-grx/0.1.1rc6/).

## Installation

> [!IMPORTANT] 
    > This software is highly unstable and subject to change at any time. Version 0.1.4 is only compatible with `fourier-grx` version `v0.1.1-rc.6`.


### On the robot

On the robot, install `fourier-grx==0.1.0rc6` following the instructions [here](https://github.com/FFTAI/Wiki-GRx-Deploy/tree/0.1.0rc6).

### On your local machine:

Run:

```bash
pip install fourier-grx-client==0.1.4
```

## Usage

First make sure you know the IP address of the robot server and the namespace it is running on. Then you can create a client object like this:

```python
from fourier_grx_client import *
r = RobotClient(namespace="gr/my_awesome_robot", server_ip="192.168.6.6")
```

!!! tip
    For more information on the API, see the [API Reference](reference/api.md).
