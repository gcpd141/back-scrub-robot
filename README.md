# Back Scrub Robot Simulation

MuJoCo-based back scrub robot simulation system with visual tracking and robotic arm control.

## 🎯 Project Overview

A proof-of-concept project exploring service robot applications in elderly care scenarios. Uses vision system to detect face position and controls robotic arm to simulate back scrubbing motion.

## 📦 Project Structure

```
back-scrub-robot/
├── simulation.py           # MuJoCo simulation control
├── vision.py              # OpenCV face detection
├── vision_driven_sim_fixed.py  # Fixed combined version
├── vision_driven_sim_v2.py     # Improved version
├── face_coord.json        # Coordinate file (runtime generated)
├── models/                # 3D model files
├── start.bat             # Windows quick start menu
└── README.md             # This file
```

## 🚀 Quick Start (Windows)

### Option 1: Double-click to run (Recommended)

1. **Double-click** `start.bat`
2. **Enter number** to select:
   - `[1]` Start Vision (face detection)
   - `[2]` Start Simulation (robot arm)
   - `[3]` Run Combined (recommended)

### Option 2: Command Line

```cmd
# 1. Install dependencies
pip install mujoco opencv-python numpy

# 2. Start vision module (in one terminal)
python vision.py

# 3. Start simulation module (in another terminal)
python simulation.py

# OR run combined version
python vision_driven_sim_fixed.py
```

## 🔧 How It Works

1. `vision.py` captures camera feed and detects face position
2. Coordinates are written to `face_coord.json`
3. `simulation.py` reads coordinates and moves robotic arm
4. Arm follows face position, simulating back scrubbing

## 📊 Technical Details

### Vision Module
- OpenCV Haar cascade classifier
- Real-time face detection (30 FPS)
- Coordinate mapping (pixels → simulation world)

### Simulation Module
- MuJoCo physics engine
- 6-DOF robotic arm control
- Force sensor feedback (contact detection)

## ⚠️ Known Issues

### Visual Drift
Current version has drift issues due to:
- File read/write latency
- Occasional face detection loss
- Coordinate mapping needs calibration

**Improvement directions**:
- Use TCP/IP instead of file communication
- Add Kalman filtering for smooth trajectory
- Implement target-loss recovery logic

## 💡 Application Scenarios

- Elderly care community assistance
- Hospital rehabilitation nursing
- Home intelligent service

## 📝 Project Status

**Current**: Simulation/Proof-of-concept stage

**Completed**:
- ✅ Basic simulation environment
- ✅ Vision detection module
- ✅ Robotic arm following control
- ✅ Force feedback detection

**TODO**:
- ❌ Fix visual drift
- ❌ Real robotic arm deployment
- ❌ Path planning optimization
- ❌ Safety mechanisms

## 📄 License

MIT License

---

**Author**: Song Tao  
**Email**: 296068696@qq.com  
**GitHub**: https://github.com/GCPD141/back-scrub-robot
