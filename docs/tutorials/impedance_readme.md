Updating the Fourier Robot Control Environment and Model Files

Steps:

1. Update the grx-env environment
```bash
pip install fourier-grx==1.0.0a16
pip install fourier-core==0.2.4a4
```

2. Update model files
   - Delete the cache file in the home directory:./cache/fourier_robot_descriptions

3. Activate the robot emergency stop switch and start the server, the server will automatically update model files.
   `grx run ./config_GR1_T2_test.yaml`

4. Open a new terminal and recalibrate (this is crucial for joints with absolute encoders), this step will generate a file called "sensor_offset.json" in current directory. After this, it is necessary to actively terminate the server process and restart it.
   ```bash
   conda activate grx-env
   grx calibrate
   grx run ./config_GR1_T2_test.yaml
   ```

5. Then running the test file.
   ```bash
   python ./test_impedance_control.py
   ```

> [!WARNING]
>
> 1. During the testing of the robot, if the emergency stop switch is activated and the joints are not at the zero position during subsequent server restarts, running the test file may cause the joints to rapidly reset and potentially diverge during movement.
> 2. When the host CPU usage is too high or there is interference from other port communications, the server-side execution frequency may fail to reach 120Hz. In such cases, the bash window will display the message: `[ImpedanceController] The recommended control frequency is above 120Hz.’`At this point, torque control may become unstable. Please close the relevant programs and try again.

> [!NOTE]
>
> 1. If there is a need for **remote connection**, the following actions can be taken:
>
>    - add namespace parameter while restart server.
>
>      ```bash
>      grx run ./config_GR1_T2_test.yaml --namespace robot/xxx
>      ```
>
>    - modify the file(script.py):
>
>      ```python
>      client = RobotClient(namespace = "robot/xxx", server_ip = "localhost")
>      ```
>
>    - The namespace should match the input of the `grx run` command, and the server IP refers to the IP address of the computer that is running the service.It is important to note that both the server and client need to be deployed on the computer mounted on the robot, and then another computer can connect to it  via SSH.
>
> 2. When you need to **modify the configuration**, you can make the necessary changes in the config file(config\_*.\_yaml).
>
>    - When you need to use the T1 or T2 robots, you can adjust "robot.mechanism" and "robot_config.robot_name".
>      - robot.mechanism:"T1" or "T2",
>      - robot_config.robot_name: "GR1T2" or "GR1T2_jaw" or "GR1T2_inspire_hand" or "GR1T2" or "GR1T2_jaw" or "GR1T2_inspire_hand".
>    - To disable a specific joint, you can modify the corresponding value in the `actuator.comm_enalbe` array to `false`.
>    - You can modify the variables upper_impedance_controller.k and upper_impedance_controller.b to alter the motion states of the motors related to upper limb current control. Additionally, by adjusting the parameters upper_impedance_controller.\*\_kp and upper_impedance_controller.\*\_kd, you can change the motion states of the motors associated with upper limb position control.



## Test

|                            | RMSE                 | Max Error            | Mean Error              | Standard Deviation   |
| -------------------------- | -------------------- | -------------------- | ----------------------- | -------------------- |
| left_shoulder_pitch_joint  | 0.04008544070660181  | 0.06234344735152031  | -0.011687628077918923   | 0.03834373360217718  |
| left_shoulder_roll_joint   | 0.04235997844834761  | 0.07318156555641808  | 0.0196626723093489      | 0.03751995591414838  |
| left_shoulder_yaw_joint    | 0.02507504074171475  | 0.04605894773775532  | 0.0011418101024684546   | 0.025049030677624144 |
| left_elbow_pitch_joint     | 0.025307513930046044 | 0.047534448024741494 | 0.0042375332882000445   | 0.024950221901836284 |
| left_wrist_yaw_joint       | 0.054103787637786824 | 0.08319393881881898  | -0.005080400854025337   | 0.053864732097330184 |
| right_shoulder_pitch_joint | 0.045023584847789636 | 0.0910698050826666   | -0.014690908047414585   | 0.042559375151528266 |
| right_shoulder_roll_joint  | 0.03575382041185528  | 0.060584443437756486 | -0.01331000735075535    | 0.033184022938246016 |
| right_shoulder_yaw_joint   | 0.02790457745214032  | 0.048764440814140775 | -0.0031824174691271285  | 0.02772251182405181  |
| right_elbow_pitch_joint    | 0.024735911356084384 | 0.044650959819256875 | -0.00353523548144576    | 0.02448198155188406  |
| right_wrist_yaw_joint      | 0.06782975820004591  | 0.10850632415460848  | -0.00035285815679188883 | 0.06782884038960037  |
