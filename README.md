# ComboFox Parking Controller

This ROS 2 package (`combofox_parking`) provides a node to control the ComboFox mobile manipulator and move it to a predefined "parking pose," which is ideal for shipping or safe storage.

The node executes an automated sequence that includes:
1.  Rearming the arm.
2.  Moving to the defined parking pose.
3.  Disarming the arm upon completion.

---

## Dependencies

This package requires the following ROS 2 packages to be present and correctly configured in the workspace:

* **`ibt_ros2_driver`**: The low-level driver for robot communication.
* **`ibt_ros2_description`**: Contains the robot description files (URDF).
* **`ibt_ros2_interfaces`**: Custom message, service, and action definitions.
* **`combo_box_interfaces`**: Additional custom interface definitions.

---

## Installation

1.  **Clone the Repository**: Ensure this package and all its dependencies are in the `src` folder of your ROS 2 workspace.

    ```bash
    cd ~/ros2_ws/src
    git clone [https://github.com/fabrisdnl/combofox_parking.git](https://github.com/fabrisdnl/combofox_parking.git)
    # Clone other dependency repositories here as well...
    ```

2.  **Install ROS Dependencies**:
    ```bash
    cd ~/ros2_ws
    rosdep install --from-paths src -y --ignore-src
    ```

3.  **Build the Workspace**:
    ```bash
    cd ~/ros2_ws
    colcon build --symlink-install
    ```

---

## Usage

To run the parking sequence, use the provided launch file. This will start all the necessary nodes, including the drivers and the controller node.

1.  **Source the Workspace**:
    ```bash
    source ~/ros2_ws/install/setup.bash
    ```

2.  **Run the Launch File**:
    ```bash
    ros2 launch combofox_parking parking.launch.py
    ```

The node will activate after a 3-second delay, execute the parking sequence, and then terminate cleanly.

### Launch Arguments

You can customize the launch using command-line arguments:

* **`use_rviz`**: Whether to launch RViz for visualization. (default: `false`)
* **`arm_type`**: Specifies the arm type to use. (default: `robofox_61814v3`)
* **`arm_prefix`**: Prefix for the arm's topics and frames. (default: `robofox`)

Example for launching with RViz:
```bash
ros2 launch combofox_parking parking.launch.py use_rviz:=true