# `grx` CLI Reference

!!! warning "Attention" 
    The `grx` CLI is installed with the `fourier-grx` package,  which is only comaptible with `python==3.11`. It is intended to be running onboard the robot. Make sure to install the package before using the CLI.


# `grx`

**Usage**:

```console
$ grx [OPTIONS] COMMAND [ARGS]...
```

**Options**:

* `--install-completion`: Install completion for the current shell.
* `--show-completion`: Show completion for the current shell, to copy it or customize the installation.
* `--help`: Show this message and exit.

**Commands**:

* `run`: Run the robot server.
* `states`: Print the current robot states.
* `calibrate`: Calibrate the robot sensors and save the...
* `disable`: Disable all the motors.
* `enable`: Enable all the motors.
* `generate-sensor-offset`: Generate sensor offset file from the...
* `run`: Run the robot server.
* `states`: Print the current robot states.


## `grx run`

Run the robot server.

**Usage**:

```console
$ grx run [OPTIONS] CONFIG
```

**Arguments**:

* `CONFIG`: Path to the config file  [required]

**Options**:

* `--namespace TEXT`: Namespace for the robot
* `--namespace TEXT`: Namespace for the robot  [default: gr]
* `--urdf-path TEXT`: Path to the urdf file  [default: ./urdf]
* `--freq INTEGER`: Main loop frequency in hz. defaults to 400hz.  [default: 400]
* `--verbose / --no-verbose`: Print internal debug info  [default: verbose]
* `--help`: Show this message and exit.

## `grx states`

Print the current robot states.

**Usage**:

```console
$ grx states [OPTIONS]
```

**Options**:

* `--help`: Show this message and exit.

## `grx calibrate`

Calibrate the robot sensors and save the offsets to a file

**Usage**:

```console
$ grx calibrate [OPTIONS] [OUTPUT_PATH]
```

**Arguments**:

* `[OUTPUT_PATH]`: Path to the output file  [default: sensor_offsets.json]

**Options**:

* `--help`: Show this message and exit.

## `grx disable`

Disable all the motors.

**Usage**:

```console
$ grx disable [OPTIONS]
```

**Options**:

* `--help`: Show this message and exit.

## `grx enable`

Enable all the motors.

**Usage**:

```console
$ grx enable [OPTIONS]
```

**Options**:

* `--help`: Show this message and exit.

## `grx generate-sensor-offset`

Generate sensor offset file from the control SDK installed on the host machine, usually located at ~/RoCS/bin/pythonscripts/absAngle.json

**Usage**:

```console
$ grx generate-sensor-offset [OPTIONS]
```

**Options**:

* `--help`: Show this message and exit.

