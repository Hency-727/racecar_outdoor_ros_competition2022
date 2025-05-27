# racecar_outdoor_ros_competition2022
Codebase for 17th outdoor racing car competition based on ros.
---
## 📁 Configuration
```bash
${racing_car_ws}/
├── src/
│ ├── racing_driver/ # Chassis driver using PID controller.
│ ├── robot_localization/ # Localization.
│ ├── sztu_mapping/ # Mapping.
│ ├── sztu_racecar/ # Main code: PP controller and key points recording.
│ └── sztu_vision/ # Vision module, get the steering angle.
└── README.md
```
---

## 🔧 Environment

- Ubuntu-16.04 + ROS Kinetic
- `Opencv2`
- `ros_control`


### ✅ Installation:
```bash
git clone https://github.com/Hency-727/racecar_outdoor_ros_competition2022.git
catkin_make
echo "source $(pwd)/devel/setup.bash" >> ~/.bashrc && source ~/.bashrc
```

### 🚀 Start:
```bash
roslaunch sztu_racecar sztu_main.launch
```
```bash
Rviz
```