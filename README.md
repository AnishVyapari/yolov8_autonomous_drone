# 🚁 YOLOv8 Autonomous Drone

<div align="center">

[![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![YOLOv8](https://img.shields.io/badge/YOLOv8-Ultralytics-00FFFF.svg)](https://github.com/ultralytics/ultralytics)
[![DroneKit](https://img.shields.io/badge/DroneKit-2.9.2-green.svg)](https://github.com/dronekit/dronekit-python)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

Real-time object detection and autonomous navigation system for drones using YOLOv8 and MAVLink

</div>

## 📋 Overview

This project implements an advanced autonomous drone system that combines YOLOv8 object detection with MAVLink communication for real-time aerial surveillance and intelligent navigation. The system enables drones to detect, track, and respond to objects in their environment autonomously.

### ✨ Key Features

- **🎯 Real-time Object Detection**: Powered by YOLOv8 for high-speed, accurate detection
- **🛸 Autonomous Flight**: DroneKit integration for intelligent drone control
- **📡 MAVLink Communication**: Seamless communication with flight controllers (Pixhawk/APM)
- **📊 Object Tracking**: Multi-object tracking with DeepSORT/ByteTrack
- **🎥 Camera Support**: Works with USB, CSI cameras, and Raspberry Pi Camera Module
- **⚙️ Configurable**: YAML-based configuration for easy customization
- **📈 Telemetry Overlay**: Real-time display of drone status and GPS data
- **💾 Data Logging**: Save detections, videos, and mission data

## 🏗️ Architecture

```
┌─────────────────┐
│  Drone Camera   │
└────────┬────────┘
         │
         ▼
┌─────────────────────────────┐
│    YOLOv8 Inference Engine  │
│  - Object Detection         │
│  - Object Tracking          │
│  - Real-time Processing     │
└────────┬────────────────────┘
         │
         ▼
┌─────────────────────────────┐
│   Autonomous Controller     │
│  - Decision Making          │
│  - Path Planning            │
│  - Safety Checks            │
└────────┬────────────────────┘
         │
         ▼
┌─────────────────────────────┐
│      MAVLink Bridge         │
│  - Flight Controller Comm   │
│  - Telemetry Data           │
└────────┬────────────────────┘
         │
         ▼
┌─────────────────┐
│  Pixhawk/APM    │
│  Flight Control │
└─────────────────┘
```

## 🚀 Quick Start

### Prerequisites

- Python 3.8 or higher
- CUDA-capable GPU (recommended for real-time performance)
- Companion computer (Raspberry Pi 4, Jetson Nano, or similar)
- Flight controller (Pixhawk, APM)
- Camera module

### Installation

1. **Clone the repository**
   ```bash
   git clone https://github.com/AnishVyapari/yolov8_autonomous_drone.git
   cd yolov8_autonomous_drone
   ```

2. **Install dependencies**
   ```bash
   pip install -r requirements.txt
   ```

3. **Download YOLOv8 weights**
   ```bash
   # Nano model (fastest)
   wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt
   
   # Or use other variants: yolov8s.pt, yolov8m.pt, yolov8l.pt, yolov8x.pt
   ```

4. **Configure the system**
   ```bash
   # Edit config/detector.yaml to match your setup
   nano config/detector.yaml
   ```

### Basic Usage

#### 1. Object Detection Only (No Drone)

```bash
python scripts/infer.py --config config/detector.yaml --no-display
```

#### 2. With Drone Connection

```bash
# Connect via USB/Serial
python scripts/infer.py --config config/detector.yaml --connection /dev/ttyACM0

# Connect via UDP (for simulation)
python scripts/infer.py --config config/detector.yaml --connection udp:127.0.0.1:14550
```

#### 3. Object Tracking Mode

```bash
python scripts/scripts/track.py --config config/detector.yaml --camera 0
```

#### 4. Autonomous Mission

```bash
# Create waypoints.csv with your mission waypoints
python scripts/mission.py --waypoints data/waypoints.csv --connection udp:127.0.0.1:14550
```

## 📁 Project Structure

```
yolov8_autonomous_drone/
├── config/
│   └── detector.yaml          # Main configuration file
├── scripts/
│   ├── infer.py               # YOLOv8 inference with DroneKit
│   ├── mission.py             # Autonomous mission control
│   ├── mavlink_bridge.py      # MAVLink communication bridge
│   ├── scripts/
│   │   └── track.py           # Object tracking implementation
│   └── utils/
│       ├── camera.py          # Camera utilities
│       ├── gps.py             # GPS utilities
│       └── io.py              # I/O utilities
├── data/                      # Data directory
├── output/                    # Output directory for logs/videos
├── requirements.txt           # Python dependencies
├── LICENSE                    # MIT License
└── README.md                  # This file
```

## ⚙️ Configuration

Edit `config/detector.yaml` to customize:

- **Model Settings**: Model weights, confidence threshold, device
- **Camera Settings**: Resolution, FPS, device ID
- **Drone Settings**: Connection string, baud rate
- **Detection Settings**: Tracking, class filters
- **Autonomous Behavior**: Target classes, safety parameters
- **Logging**: Video saving, output directories

## 🎮 Hardware Setup

### Recommended Hardware

1. **Companion Computer**:
   - Raspberry Pi 4 (4GB+ RAM)
   - NVIDIA Jetson Nano/Xavier
   - Intel NUC

2. **Flight Controller**:
   - Pixhawk 4/5/6
   - APM 2.8

3. **Camera**:
   - Raspberry Pi Camera Module V2
   - Logitech C920/C922
   - Any USB webcam

### Wiring

```
Companion Computer  ←→  Flight Controller
   (USB/UART)              (TELEM2)
        ↓
    Camera Module
```

## 🔧 Advanced Usage

### Custom Object Detection

Train YOLOv8 on your custom dataset:

```python
from ultralytics import YOLO

# Load a model
model = YOLO('yolov8n.yaml')

# Train the model
results = model.train(data='your_dataset.yaml', epochs=100, imgsz=640)
```

### MAVLink Bridge

Forward MAVLink messages between flight controller and ground station:

```bash
python scripts/mavlink_bridge.py --fc /dev/ttyACM0 --udp-out 127.0.0.1:14600
```

## 📊 Performance

| Model | mAP | FPS (Jetson Nano) | FPS (RPi 4) | Size |
|-------|-----|-------------------|-------------|------|
| YOLOv8n | 37.3 | ~25 | ~8 | 6.3 MB |
| YOLOv8s | 44.9 | ~18 | ~5 | 22 MB |
| YOLOv8m | 50.2 | ~12 | ~3 | 50 MB |

## 🛡️ Safety Features

- **Geofence**: Configurable safety radius
- **Battery Monitoring**: Auto RTL on low battery
- **GPS Validation**: Ensures valid GPS before autonomous flight
- **Failsafe Modes**: Automatic fallback behaviors

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## 📝 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- [Ultralytics YOLOv8](https://github.com/ultralytics/ultralytics) - Object detection framework
- [DroneKit-Python](https://github.com/dronekit/dronekit-python) - Drone control library
- [MAVLink](https://mavlink.io/) - Micro Air Vehicle communication protocol

## 📧 Contact

Anish Vyapari - [@AnishVyapari](https://github.com/AnishVyapari)

Project Link: [https://github.com/AnishVyapari/yolov8_autonomous_drone](https://github.com/AnishVyapari/yolov8_autonomous_drone)

---

<div align="center">

**⭐ Star this repository if you find it helpful! ⭐**

</div>
