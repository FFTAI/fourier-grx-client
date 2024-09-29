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

* `calibrate`: Calibrate the robot sensors and save the...
* `disable`: Disable all the motors.
* `enable`: Enable all the motors.
* `run`: Run the robot server.
* `states`: Print the current robot states.

## `grx calibrate`

Calibrate the robot sensors and save the offsets to a file

**Usage**:

```console
$ grx calibrate [OPTIONS]
```

**Options**:

* `--namespace TEXT`: Namespace for the robot
* `--ip TEXT`: IP address of the robot server.  [default: localhost]
* `--help`: Show this message and exit.

## `grx disable`

Disable all the motors.

**Usage**:

```console
$ grx disable [OPTIONS]
```

**Options**:

* `--namespace TEXT`: Namespace for the robot
* `--ip TEXT`: IP address of the robot server.  [default: localhost]
* `--help`: Show this message and exit.

## `grx enable`

Enable all the motors.

**Usage**:

```console
$ grx enable [OPTIONS]
```

**Options**:

* `--namespace TEXT`: Namespace for the robot
* `--ip TEXT`: IP address of the robot server.  [default: localhost]
* `--help`: Show this message and exit.

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
* `--freq INTEGER`: Main loop frequency in hz. defaults to 400hz.  [default: 400]
* `--verbose / --no-verbose`: Print internal debug info  [default: no-verbose]
* `--visualize / --no-visualize`: Visualize the robot in meshcat  [default: no-visualize]
* `--help`: Show this message and exit.

## `grx states`

Print the current robot states.

**Usage**:

```console
$ grx states [OPTIONS]
```

**Options**:

* `--namespace TEXT`: Namespace for the robot
* `--ip TEXT`: IP address of the robot server.  [default: localhost]
* `--help`: Show this message and exit.
