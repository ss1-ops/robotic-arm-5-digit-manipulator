## Manus ROS2 Package

This package is provided and created by [**Manus**](https://www.manus-meta.com/).  
We provide this package here for convenience.  
Please refer to the [original documentation](https://docs.manus-meta.com/3.1.0/Plugins/SDK/ROS2/getting%20started/) and contact Manus support for any issues or questions regarding this package.

We include instructions below to install and use the Manus gloves for convenience.  
These are a simplified subset of the official Manus documentation.

---

## 🧰 Installation

1. **Build the package** in your ROS 2 workspace:
   ```bash
   colcon build
   ```

2. **Grant USB permissions** for the Manus gloves by installing the provided `udev` rule:
   ```bash
   sudo cp <path/to/manus_glove_pkg>/70-manus-hid.rules /etc/udev/rules.d/
   ```

3. **Reload the udev rules**:
   ```bash
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ```

---

## 🚀 Usage

1. **Source your ROS 2 workspace**:
   ```bash
   source install/setup.bash
   ```

2. **Run the Manus glove node**:
   ```bash
   ros2 run manus_glove_pkg manus_data_publisher
   ```

---

### ⚙️ Notes

We make minor adjustments to the original Manus ROS 2 package as documented below:

1. The data frame convention is changed from **Left-Handed** to **Right-Handed** to match ROS 2 standards.  
2. The SDK is configured to use **Integrated Mode only**, as we do not use Remote Mode.
