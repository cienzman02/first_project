```
# Vehicle Localization and Sector Timing Project

This ROS project computes real-time vehicle localization using GPS and wheel odometry, and segments the driving path into timed sectors. It integrates GPS data, Ackermann odometry estimation, and sector timing based on GPS intersections.

## 📦 ROS Nodes

### 1. `odometer`
Estimates vehicle position using wheel odometry and a simplified bicycle model.

- **Subscribed topics:**
  - `/speedsteer` – Vehicle speed (`y`: speed in km/h) and steering angle (`x`: steering wheel angle in degrees)

- **Published topics:**
  - `/odom` – Estimated odometry using Ackermann kinematics

### 2. `gps_odometer`
Converts GPS (WGS84) to ENU coordinates relative to a fixed reference point.

- **Subscribed topics:**
  - `/swiftnav/front/gps_pose` – Front GPS receiver data

- **Published topics:**
  - `/gps_odom` – ENU odometry computed from GPS
  - Broadcasts TF between `odom` → `gps`

- **Parameters:**
  - `lat_r`, `lon_r`, `alt_r` – Optional initial reference coordinates

### 3. `sector_times`
Tracks sector completion times based on crossing GPS-defined sector lines and publishes timing and speed statistics.

- **Subscribed topics:**
  - `/swiftnav/front/gps_pose`
  - `/speedsteer`

- **Published topics:**
  - `/sector_times` – Current sector ID, time, and mean speed

## 🧭 Vehicle Parameters

- **Rear wheel baseline:** 130 cm  
- **Front-to-rear wheel distance:** 176.5 cm  
- **Steering factor:** ~32 (can be tuned using GPS trajectory)

> ℹ️ The steering angle from `/speedsteer.x` is in degrees from the steering wheel.
> To use it in the bicycle model, convert to wheel angle in radians:
> double steering_rad = (steering_deg / STEERING_FACTOR) * M_PI / 180.0;

## ▶️ How to Run

1. **Start the roscore**
   roscore

2. **Play the bag file**
   rosbag play --clock project.bag

3. **Launch nodes**
   In separate terminals or a launch file:
   rosrun first_project odometer
   rosrun first_project gps_odometer
   rosrun first_project sector_times

> Make sure the `first_project` package is built and sourced:
> cd ~/catkin_ws
> catkin_make
> source devel/setup.bash

## 🛠️ Tuning the Steering Factor

Due to mechanical simplifications and possible configuration changes, the **steering factor** may need adjustment. Use `/odom` vs `/gps_odom` comparison to tune it:

- Launch `rqt_plot`:
  rqt_plot /odom/pose/pose/position/x:/gps_odom/pose/pose/position/x

- Or export and plot trajectories in Python/Matplotlib.

Look for path alignment especially on corners. If odometry turns too wide/narrow, adjust the steering factor accordingly.

## 📊 Sector Definitions

Sectors are defined by pairs of GPS coordinates forming lines. When the vehicle crosses one, the sector ID is incremented and timing restarts.

Example:
  { {45.630247, 9.289481}, {45.629969, 9.289499} },  // 1 → 2
  { {45.623658, 9.287174}, {45.623476, 9.287370} },  // 2 → 3
  { {45.616058, 9.280528}, {45.616018, 9.281218} },  // 3 → 1

## 📁 Data Topics

- `/speedsteer` – vehicle speed & steering (front wheel encoder on steering column)
- `/swiftnav/front/gps_pose` – high-accuracy front GPS (less than 1 cm error)
- `/swiftnav/rear/gps_pose` – rear GPS (not used in this project but available)

## 🧭 Notes

- GPS-based ENU coordinates are accurate and act as the ground truth.
- The odometry will **drift** over time (expected with open-loop models).
- SLAM or sensor fusion would be used in a real system to correct for this drift.

## 📄 License

This project is part of a university coursework. Adapt it freely for educational or research use.

