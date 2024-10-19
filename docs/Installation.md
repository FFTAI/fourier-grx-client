# Fourier-GRX

Welcome to the Fourier-GRX SDK, your gateway to controlling [Fourier](https://fourierintelligence.com/)'s humanoid robots! This guide will help you set up your environment. Let’s dive into the future of robotics! 🤖🚀

## 💻 System Requirement

    • Operating System: Ubuntu 20.04 and up
    • Python Version: Python 3.11

**If you are first time using the robot, please set up the permission for IMU(HIPNUC IMU) and joysticks with [permission_description](./permissions.md)**.

### System CPU Setup

In order to have good performance of the Neural Network inference speed, it is recommended to disable the effeciency cores of the CPU.
The following steps are for Intel CPUs:

1. Enter the BIOS setup by pressing the F2 key during boot.
2. Navigate to the Advanced tab.
3. Select Processor Configuration.
4. Disable the efficiency cores by setting the number of active cores to 0.
5. Press F10 to save and exit.
6. Reboot the system.

## 🚀 Environment Setup

### Step 1: Install Conda

1. **Download and install Miniconda:**
   Follow the instructions on the [Miniconda installation guide](https://docs.conda.io/en/latest/miniconda.html).

### Step 2: Create and Activate Conda Environment

1. **Create and activate the environment:**

   > [!NOTE]
   > For now we only support Python 3.11.

    ```bash
    conda create -n grx-env python=3.11
    conda activate grx-env
    ```

   For more details, see the [Conda user guide](https://docs.conda.io/projects/conda/en/latest/user-guide/getting-started.html).

### Step 3: Set Up Firewall

If you are using `ufw`, follow these steps to enable firewall access for the `grx` server to be able to automatically detect clients:

1. **Enable firewall access:**
    ```bash
    sudo ufw allow 7446/udp
    ```

2. **Check the firewall status:**
    ```bash
    sudo ufw status
    ```

### Step 4: Install `fourier-grx` Library

1. **Install the library:**
    ```bash
    python -m pip install fourier-grx==1.0.0a18
    ```

2. **Verify installation:**
    ```bash
    grx --help
    ```
