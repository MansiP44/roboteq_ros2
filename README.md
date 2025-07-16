This ROS2 roboteq controller was developed for the VineBot at THWS.

1. Install Serial library by following steps on https://github.com/wjwwood/serial
2. A plugin for roboteq hardware interface is created.
3. The config parameters are read in a xacro file and then sent to the hardware_interface via the hardware_interface::HardwareInfo object that is obtained from the urdf by the controller manager.
