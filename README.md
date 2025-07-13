# 🟠 Orange Cone Detection using ROS 2 & OpenCV

This project implements **orange cone detection** using a front-facing camera on a **4-wheeled differential drive robot** in a **ROS 2** simulation (Gazebo). It uses **OpenCV** for detecting orange color regions and overlays a **bounding box** when the cone is detected.

## 📦 Features

- Real-time orange color detection via camera
- HSV masking to isolate orange objects
- Area thresholding to determine proximity
- Draws **bounding box** and label ("Orange Cone") on the detected cone
- Uses **contours** to extract region of interest
- Prevents repeated logging using a **flag variable**

## 🧪 Requirements

Ensure you have the following ROS 2 packages and Python libraries installed:

- ROS 2 Humble (or compatible version)
- `cv_bridge`
- `OpenCV`
- `NumPy` (version **<2.0** recommended to avoid compatibility issues with openCV)

## ⚙️ Setup Instructions

## Building the Workspace and Sourcing 

```bash
cd ~/four_wheel_ws
colcon build
source install/setup.bash
```

---

### Step 1: Launch the simulation

> Open a new terminal

```bash
cd ~/four_wheel_ws
source install/setup.bash
ros2 launch four_wheel_bot view_robot.launch.py
```

---

### Step 2: Launch teleop

> Open a new terminal

```bash
cd ~/four_wheel_ws
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
```

---

## Difficulties Faced

### 1.Numpy and OpenCV compatibility issue:
- The orange_detector node initially failed due to incompatibility between OpenCV and NumPy. I downgradred by numpy to work with openCV using the command:

```bash
pip install numpy<2.0
```
### 2. Repeated log spam in the terminal:
- After the detection of orange cone, the warning message kept spanning. I fixed this using the variable alert_triggered to only display the message once in the terminal.
