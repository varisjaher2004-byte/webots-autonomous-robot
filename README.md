# Autonomous Smart Home Indoor Robot Simulation 🤖🏠

A 3D simulation of an autonomous mobile robot operating within a smart home indoor environment using **Webots** and **Python**. The robot dynamically senses its surroundings to navigate rooms, avoid dynamic/static obstacles, and maintain continuous motion without human intervention.

---

## 🔑 Key Features & Technical Highlights

* **Proximity Sensing & Obstacle Detection:** Utilizes differential distance sensor arrays (`ps0` to `ps7`) positioned around the robot chassis to continuously sample proximity data.
* **Camera Sensor Integration:** Features an onboard camera feed integrated for real-time visual environment monitoring.
* **Dynamic Navigation Algorithm:** Implemented threshold-based reactive movement logic (Braitenberg obstacle avoidance strategy) to calculate turn angles and adjust wheel speeds dynamically.
* **Custom Indoor World:** Designed and configured a multi-room smart home environment with residential layout constraints, furniture barriers, and defined floor boundaries.
* **Differential Drive Control:** Smooth velocity actuation using independent left and right wheel motor controllers (`left wheel motor`, `right wheel motor`).

---

## 🛠️ Tech Stack & Tools

* **Simulator:** Webots Open-Source Robot Simulator
* **Programming Language:** Python 3.x
* **Libraries/APIs:** `controller` (Webots Python API)
* **Control Mechanism:** Differential Drive Kinematics & Reactive Sensor-Motor Control Loop

---

## 📐 How the Autonomous Controller Works

The core control script `my_controller.py` operates on a constant sampling loop (`TIME_STEP = 64` ms):

1. **Sensor Initialization:** Enables 8 distance sensors and the camera module.
2. **Threshold Monitoring:** Reads scalar distance values from front, left, and right proximity sensors.
3. **Collision Risk Assessment:** Evaluates if obstacles cross the defined proximity threshold (`100.0` units).
4. **Behavioral Switching:**
   * **Obstacle Detected Right:** Pivots left by reducing left motor speed.
   * **Obstacle Detected Left:** Pivots right by reducing right motor speed.
   * **Clear Path:** Maintains forward cruise velocity (`MAX_SPEED = 6.28` rad/s).

---

## 📂 Repository Structure

```text
webots-autonomous-robot/
├── indoor/
│   ├── controllers/
│   │   └── my_controller/
│   │       └── my_controller.py      # Main Python control script (Logic & Sensor Loop)
│   └── worlds/
│       ├── NEW.wbt                   # Webots 3D indoor world configuration
│       ├── .NEW.wbproj               # Webots project configuration
│       └── .NEW.jpg                  # Environment preview thumbnail
└── README.md                         # Project documentation
