<!-- Drone & Air Glassmorphism Banner with Detection Theme -->
<div align="center">
  <svg width="800" height="200" xmlns="http://www.w3.org/2000/svg">
    <defs>
      <linearGradient id="skyGrad" x1="0%" y1="0%" x2="100%" y2="100%">
        <stop offset="0%" style="stop-color:rgba(135,206,250,0.35);stop-opacity:1" />
        <stop offset="100%" style="stop-color:rgba(0,119,182,0.35);stop-opacity:1" />
      </linearGradient>
      <filter id="softBlur"><feGaussianBlur stdDeviation="2"/></filter>
    </defs>

    <!-- Glass Sky Panel -->
    <rect width="800" height="200" rx="18" fill="url(#skyGrad)" opacity="0.65" filter="url(#softBlur)"/>
    <rect width="800" height="200" rx="18" fill="none" stroke="rgba(255,255,255,0.5)" stroke-width="2"/>

    <!-- Drone Icon -->
    <g transform="translate(140,85)">
      <circle cx="0" cy="0" r="18" fill="rgba(255,255,255,0.25)" stroke="#fff"/>
      <g>
        <circle cx="-40" cy="-20" r="10" fill="rgba(255,255,255,0.25)" stroke="#fff">
          <animateTransform attributeName="transform" type="rotate" from="0 -40 -20" to="360 -40 -20" dur="2s" repeatCount="indefinite"/>
        </circle>
        <circle cx="40" cy="-20" r="10" fill="rgba(255,255,255,0.25)" stroke="#fff">
          <animateTransform attributeName="transform" type="rotate" from="0 40 -20" to="-360 40 -20" dur="2s" repeatCount="indefinite"/>
        </circle>
        <circle cx="-40" cy="20" r="10" fill="rgba(255,255,255,0.25)" stroke="#fff">
          <animateTransform attributeName="transform" type="rotate" from="0 -40 20" to="-360 -40 20" dur="2s" repeatCount="indefinite"/>
        </circle>
        <circle cx="40" cy="20" r="10" fill="rgba(255,255,255,0.25)" stroke="#fff">
          <animateTransform attributeName="transform" type="rotate" from="0 40 20" to="360 40 20" dur="2s" repeatCount="indefinite"/>
        </circle>
      </g>
    </g>

    <!-- Detection Boxes -->
    <g fill="none" stroke="#00e5ff" stroke-width="2" opacity="0.9">
      <rect x="560" y="50" width="80" height="50" rx="6">
        <animate attributeName="opacity" values="0.3;1;0.3" dur="2s" repeatCount="indefinite"/>
      </rect>
      <rect x="620" y="110" width="60" height="40" rx="6">
        <animate attributeName="opacity" values="0.3;1;0.3" dur="2.2s" begin="0.4s" repeatCount="indefinite"/>
      </rect>
    </g>

    <!-- Title and Tagline -->
    <text x="400" y="85" font-family="Arial, sans-serif" font-size="32" font-weight="bold" fill="#ffffff" text-anchor="middle">
      YOLOv8 Autonomous Drone
    </text>
    <text x="400" y="112" font-family="Arial, sans-serif" font-size="16" fill="rgba(255,255,255,0.95)" text-anchor="middle">
      🚁 See. Detect. Decide. — Airborne Perception in Real Time
    </text>
    <text x="400" y="136" font-family="Arial, sans-serif" font-size="12" fill="rgba(255,255,255,0.8)" text-anchor="middle">
      YOLOv8 • Multi-Target Tracking • Edge Acceleration
    </text>
  </svg>
</div>

---

## ☁️ Airy Glass Bouncy Loader
```html
<!DOCTYPE html>
<html>
<head>
  <style>
    .sky-loader-wrap { height: 220px; display:flex; align-items:center; justify-content:center; background: linear-gradient(180deg,#7fc7ff,#3a86ff); }
    .sky-glass { padding:22px 28px; border-radius:16px; border:1px solid rgba(255,255,255,0.35); background: rgba(255,255,255,0.22); backdrop-filter: blur(12px); -webkit-backdrop-filter: blur(12px); box-shadow: 0 10px 30px rgba(0,0,0,0.15); }
    .bubble { width:14px; height:14px; margin:0 6px; border-radius:50%; background: rgba(255,255,255,0.7); animation: updown 1.2s infinite ease-in-out; display:inline-block; }
    .bubble:nth-child(1){ animation-delay:-.24s; background: rgba(0,229,255,.8) }
    .bubble:nth-child(2){ animation-delay:-.12s; background: rgba(144,224,239,.85) }
    .bubble:nth-child(3){ background: rgba(255,255,255,.85) }
    @keyframes updown { 0%,80%,100%{ transform: translateY(0) scale(1)} 40%{ transform: translateY(-18px) scale(1.15)} }
    .hint { margin-top:10px; color:#fff; font-family: Arial, sans-serif; letter-spacing:2px; font-size:13px; text-align:center; }
  </style>
</head>
<body>
  <div class="sky-loader-wrap">
    <div class="sky-glass">
      <span class="bubble"></span>
      <span class="bubble"></span>
      <span class="bubble"></span>
      <div class="hint">DETECTING...</div>
    </div>
  </div>
</body>
</html>
```

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
```
