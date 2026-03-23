# Alvik-Factory

Using Arduino Alvik robots to coordinate routing deliveries and pickups with line following.

## Overview

This system drives a single Arduino Alvik robot along a track made of black duct tape. A **blue sticker** marks the fixed starting position, and **two red stickers** mark the drop-off locations (D and E). The robot follows the tape and stops at the correct destination.

Communication is handled over ROS2:
- **Supervisory.py** — User-facing node that checks Alvik status and sends commands.
- **FactoryAGV.py** — Receives commands from the supervisor and relays them to the robot.
- **BaseAGV.ino** — Runs on the Alvik; handles movement, line following, and start/stop logic.

**Example message exchange:**
```
# Supervisory sends:
{"command": "request_service", "destination": "D"}

# FactoryAGV responds:
{"response": "service_assigned", "agv_name": "Alvik1", ...}
```

---

## Requirements

- ROS2 Jazzy
- Python 3
- Arduino Alvik with micro-ROS
- Docker (for the micro-ROS agent)

---

## Setup

### Step 1: Flash the Arduino Alvik

1. Open `BaseAGV.ino` in the Arduino IDE.
2. Replace **lines 48–50** with your WiFi network name, WiFi password, and your computer's IP address.
3. Upload `BaseAGV.ino` to the Alvik's ESP32.

### Step 2: Install and Run the micro-ROS Agent

The micro-ROS agent bridges communication between the Alvik (micro-ROS) and your desktop (ROS2). It runs in Docker.

> **If you already have Docker and micro-ROS installed, skip to step 2e.**

In a new terminal:

**a)** Install Docker:
```bash
sudo apt update && sudo apt install -y docker.io
```

**b)** Start and enable Docker:
```bash
sudo systemctl start docker
sudo systemctl enable docker
```

**c)** Add your user to the Docker group:
```bash
sudo usermod -aG docker $USER
newgrp docker
```

**d)** Verify Docker is installed:
```bash
docker --version
```

**e)** Run the micro-ROS agent:
```bash
docker run -it --rm --net=host microros/micro-ros-agent:jazzy udp4 --port 8888 -v 6
```

Once the agent is running, turn the Alvik on. You should see new packets arriving in the terminal if the WiFi credentials and IP address are correct.

### Step 3: Start FactoryAGV

In a **new terminal**, navigate to the project folder and run:
```bash
source /opt/ros/jazzy/setup.bash
python3 FactoryAGV.py
```

### Step 4: Start Supervisory

In another **new terminal**, navigate to the project folder and run:
```bash
source /opt/ros/jazzy/setup.bash
python3 Supervisory.py
```

---

## Usage

### Startup Procedure

1. Place the robot on the **blue sticker** at the starting position.
2. The robot must remain still on the sticker for **2 seconds** before it registers as **IDLE** and is ready to accept commands.

### Sending a Delivery

In the Supervisory terminal:

| Input | Action |
|-------|--------|
| `1 D` | Request delivery to drop-off location **D** (first red sticker) |
| `1 E` | Request delivery to drop-off location **E** (second red sticker) |
| `2`   | Confirm the part is loaded — robot begins moving |
| `3`   | Confirm delivery complete — robot returns to start |

### Delivery Flow

1. **`1 D`** — FactoryAGV assigns an available robot and commands it to wait for loading.
2. **`2`** — Robot begins following the tape toward the destination.
3. The robot stops at the correct red sticker (first for D, second for E).
4. **`3`** — Robot is released and returns to the blue starting sticker.
