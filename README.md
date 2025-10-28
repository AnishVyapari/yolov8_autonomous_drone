# 🚁 YOLOv8 Autonomous Drone

![License](https://img.shields.io/badge/license-MIT-blue.svg)
![YOLOv8](https://img.shields.io/badge/YOLOv8-Ultralytics-orange.svg)
![Python](https://img.shields.io/badge/Python-3.9%2B-brightgreen.svg)
![Build](https://img.shields.io/badge/build-passing-brightgreen.svg)
![CI](https://img.shields.io/badge/CI-GitHub%20Actions-blue.svg)

> Autonomous drone with YOLOv8 object detection for real-time aerial surveillance and navigation

---

## 💡 Project Idea

An edge-AI drone platform leveraging YOLOv8 for onboard object detection, target tracking, and autonomous navigation. The system fuses vision with IMU/GPS data to perform waypoint following, geofencing, and safe RTL (return-to-launch).

**Highlights:**
- Real-time YOLOv8 inference (N/S/M sizes) with TensorRT acceleration (optional)
- Multi-target tracking (DeepSORT/ByteTrack)
- MAVLink/Dronekit/ROS2 control pipeline
- Geo-fence enforcement and collision avoidance
- Onboard recording and mission telemetry

---

## 🔧 Hardware Components

- Flight Controller: Pixhawk 4 / Cube Orange (ArduPilot or PX4)
- Companion Computer: Jetson Nano/Xavier NX or Raspberry Pi 4 + Coral TPU
- Camera: CSI/USB Camera (IMX219/IMX477 or Logitech C920)
- GPS + Compass: u-blox M8N or M9N
- Frame: 250–450mm quadcopter frame with appropriate ESCs and motors
- Power: 4S/6S LiPo, BEC for companion power

### Hardware Setup Notes
1. Mount camera with vibration damping; align optical axis with body frame
2. Ensure serial/MAVLink wiring between FCU and companion (UART/USB)
3. Isolate companion power from noisy ESC lines; use filtered BEC
4. Calibrate compass and IMU before flight; verify GPS lock
5. Configure video device (e.g., /dev/video0) and camera parameters

---

## 🧱 Software Architecture

```
┌───────────────────────────────────────────────┐
│                Mission Controller             │
├───────────────────────────────────────────────┤
│  YOLOv8 Inference  │  Tracker  │  Planner     │
├───────────────────────────────────────────────┤
│  Video Capture  │  Sensor Fusion  │  MAVLink   │
└───────────────────────────────────────────────┘
```

- YOLOv8: Ultralytics models for detection
- Tracker: DeepSORT/ByteTrack for ID persistence
- Planner: Waypoint navigation and safety logic
- MAVLink: Dronekit or pymavlink for command/control

---

## 💻 Code Implementation

### Dependencies & Installation

```bash
# Python env
python3 -m venv .venv && source .venv/bin/activate
pip install --upgrade pip

# Core packages
pip install ultralytics opencv-python numpy scipy onnxruntime-gpu==1.17.3 filterpy
pip install lap cython_bbox # for trackers
pip install dronekit pymavlink pyserial

# Optional: TensorRT acceleration on Jetson
# Refer to NVIDIA docs; then export engine for YOLOv8
```

### Clone & Run
```bash
git clone https://github.com/AnishVyapari/yolov8_autonomous_drone.git
cd yolov8_autonomous_drone
python scripts/infer.py --source 0 --model yolov8n.pt --conf 0.4
```

### Typical Commands
```bash
# Arm and takeoff to 10m
dronekitscripts/takeoff.py --alt 10

# Start detection + tracking
python scripts/track.py --model yolov8s.pt --tracker bytetrack.yaml --source 0

# Autonomous mission from waypoints
python scripts/mission.py --waypoints data/waypoints.csv
```

---

## 📁 File Structure
```
yolov8_autonomous_drone/
├── scripts/
│   ├── infer.py              # YOLOv8 live inference
│   ├── track.py              # Detection + tracking
│   ├── mission.py            # Autonomous mission logic
│   ├── mavlink_bridge.py     # MAVLink/Dronekit interface
│   └── utils/                # Helpers
├── config/
│   ├── detector.yaml         # YOLO/Tracker config
│   └── mission.yaml          # Mission parameters
├── data/
│   ├── waypoints.csv
│   └── samples/
├── tests/
├── docs/
├── .github/workflows/ci.yml  # CI pipeline
├── requirements.txt
├── LICENSE
└── README.md
```

---

## 🧪 Testing/Simulation Setup

- SITL (Software-In-The-Loop) with ArduPilot or PX4
```bash
# ArduPilot SITL example
sim_vehicle.py -v ArduCopter -f quad --console --map
# Connect with Dronekit at 127.0.0.1:14550
```

- Video simulation feed with sample MP4 or webcam
```bash
python scripts/infer.py --source path/to/video.mp4
```

- Unit tests
```bash
pytest -q
```

---

## 📚 Documentation Tips

- Add dataset notes and labeling guidelines
- Provide model export steps (ONNX/TensorRT)
- Include safety checklist and preflight procedures
- Record flight logs and create post-flight analysis notebook

---

## ✅ CI Status

![CI](https://img.shields.io/github/actions/workflow/status/AnishVyapari/yolov8_autonomous_drone/ci.yml?branch=main)

---

## 📄 License

MIT License. See the LICENSE file for details.

---

## 🙌 Acknowledgments

- Ultralytics YOLOv8
- ArduPilot/PX4 communities
- Open-source drone ecosystem
