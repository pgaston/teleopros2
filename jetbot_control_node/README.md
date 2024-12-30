# JetBot Control Node

This project implements a ROS2 control node for the JetBot, enabling it to receive commands and control its movements.

## Project Structure

```
jetbot_control_node
├── src
│   ├── jetbot_control_node
│   │   ├── __init__.py
│   │   ├── control_node.py
│   │   └── utils.py
├── launch
│   └── control_node_launch.py
├── package.xml
├── setup.py
└── README.md
```

## Installation

1. Ensure you have ROS2 installed on your system.
2. Clone this repository to your local machine.
3. Navigate to the project directory:
   ```
   cd jetbot_control_node
   ```
4. Install the package using:
   ```
   colcon build
   ```

## Usage

To launch the JetBot control node, use the following command:
```
ros2 launch jetbot_control_node control_node_launch.py
```

## Dependencies

This package requires the following ROS2 packages:
- rclpy
- geometry_msgs
- std_msgs

## Contributing

Contributions are welcome! Please open an issue or submit a pull request for any improvements or bug fixes.

## License

This project is licensed under the MIT License. See the LICENSE file for details.