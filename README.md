# Differential-Drive Robot EKF Localization

ROS 2 workspace for a differential-drive robot using Gazebo Sim, RTAB-Map visual odometry, IMU orientation, commanded-velocity prediction, and a planar Extended Kalman Filter (EKF).

The fused pose is published on `/ekf/odom` with an `odom` to `base_link` transform.

## Concept and data flow

The system uses `/cmd_vel` for prediction, `/vo/odom` for visual position and velocity, and `/zed/zed_node/imu/data_raw` for IMU orientation. `measurement_node` combines visual position and IMU orientation into `/measurement/odom`. `prediction_node` integrates the command through a unicycle model. `ekf_node` predicts from the model and corrects it with the measurement.

## EKF mathematics

The planar state is `x = [x, y, psi]^T`, where `x` and `y` are metres and `psi` is yaw in radians. `Sigma` is the 3x3 state covariance.

### Prediction

For timestep `dt`, the unicycle model is:

```text
x_next   = x   + v cos(psi) dt
y_next   = y   + v sin(psi) dt
psi_next = psi + omega dt
```

The nonlinear model is `x_next = f(x,u) + w`. Process noise `w` represents slip, command uncertainty, timing error, and model error. The Jacobian used by the EKF is:

```text
G = [ 1  0  -v sin(psi) dt ]
    [ 0  1   v cos(psi) dt ]
    [ 0  0        1         ]
```

Covariance prediction: `Sigma_pred = G Sigma G^T + R`. `R` is process-noise covariance; increasing it makes the filter trust prediction less.

### Measurement update

The measurement is `z = [x_visual, y_visual, psi_imu]^T`. Since it directly observes the state, `h(x) = x` and `H = I`.

Innovation: `r = z - x_pred`.

Innovation covariance: `S = H Sigma_pred H^T + Q`.

Kalman gain: `K = Sigma_pred H^T S^-1`.

Correction: `x_est = x_pred + K r`.

`Q` is measurement-noise covariance. Small `Q` means the visual/IMU measurement is trusted more; large `Q` means prediction is trusted more. Yaw innovations are wrapped to `[-pi, pi]`, preventing a `+179 degree` to `-179 degree` transition from appearing as a `358 degree` error.

The code uses the Joseph covariance update: `Sigma_est = (I-KH) Sigma_pred (I-KH)^T + K Q K^T`. This is numerically safer and better preserves covariance symmetry and positive semidefiniteness.

## Installation and build

```bash
source /opt/ros/<ros_distro>/setup.bash
mkdir -p ~/EKF_ws
cd ~/EKF_ws
git clone https://github.com/Amir-sut82/EKF_ws.git .
sudo apt update
sudo apt install -y python3-rosdep python3-numpy python3-tf-transformations
sudo rosdep init   # once per machine, if needed
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro <ros_distro> -r -y
colcon build --symlink-install
source install/setup.bash
```

Replace `<ros_distro>` with your installed distribution, such as `humble` or `jazzy`.

## Run the complete system

```bash
source /opt/ros/<ros_distro>/setup.bash
source ~/EKF_ws/install/setup.bash
ros2 launch robot_description gazebo.launch.py
```

Drive the robot from another sourced terminal:

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}"
```

Inspect `/prediction/odom`, `/measurement/odom`, and `/ekf/odom` with `ros2 topic echo`.

## Keyboard driving and path plotting

Install the standard keyboard teleoperation package once:

```bash
sudo apt install ros-<ros_distro>-teleop-twist-keyboard
```

Use three terminals. Source ROS and the workspace in each terminal.

Terminal 1 — start Gazebo, the bridge, visual odometry, prediction, measurement, and EKF:

```bash
ros2 launch robot_local_localization gazebo.launch.py
```

Terminal 2 — start the rectangle test and path recorder:

```bash
ros2 run robot_local_localization test_node
```

`test_node` publishes `/ekf_path`, `/measurement_path`, `/prediction_path`, and `/gt_path`. It also contains an optional automatic rectangle command sequence. When it is stopped with `Ctrl-C`, it plots the collected EKF, measurement, prediction, and ground-truth paths with Matplotlib.

Terminal 3 — drive manually with the keyboard:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

The keyboard node publishes `/cmd_vel`. Press `u`, `i`, `o`, `j`, `k`, `l`, `m`, `,`, and `.` for motion, and `k` or the teleop stop key for zero velocity. If using the automatic rectangle test, do not run a second publisher to `/cmd_vel` at the same time.

The current launch file already starts `ros_gz_bridge` using `config/gz_bridge.yaml`; a separate bridge command is normally unnecessary. To inspect the ground-truth bridge directly:

```bash
ros2 topic echo /world/depot/dynamic_pose/info
```

For a manual bridge in an older setup, the equivalent direction is Gazebo to ROS:

```bash
ros2 run ros_gz_bridge parameter_bridge \
  '/world/depot/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
```

The configuration-file approach is preferred because it starts all required bridge mappings together.

## Important topics

| Topic | Purpose |
|---|---|
| `/cmd_vel` | commanded linear and angular velocity |
| `/vo/odom` | RTAB-Map visual odometry |
| `/zed/zed_node/imu/data_raw` | simulated IMU |
| `/prediction/odom` | uncorrected motion-model prediction |
| `/measurement/odom` | visual position plus IMU orientation |
| `/ekf/odom` | fused pose and covariance |

## Parameters

`src/robotic_course/robot_autonomy/robot_local_localization/config/params.yaml` contains robot geometry, timestep, frame names, process-noise entries (`R`), and measurement-noise entries (`Q`). Noise entries are variances, not standard deviations; variance `0.0025` corresponds to standard deviation `0.05`.

## Package layout

```text
EKF_ws/
├── README.md
└── src/robotic_course/
    ├── robot_description/                         # robot model and Gazebo Sim
    └── robot_autonomy/robot_local_localization/   # prediction, measurement, and EKF nodes
```

## Limitations

The filter estimates only planar `x`, `y`, and yaw. IMU and visual messages are not time-synchronized, measurement covariance is fixed rather than propagated from sensors, and prediction uses commanded velocity instead of measured wheel ticks. For production navigation, add timestamp synchronization, wheel encoders, covariance propagation, outlier rejection, and a full 3-D or established localization stack.

## License

Apache License 2.0. See [`src/robotic_course/LICENSE`](src/robotic_course/LICENSE).

