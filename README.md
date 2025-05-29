# ROS Vehicle Odometry and Sector Timing

This project consists of three ROS nodes for processing vehicle odometry, GPS data, and computing sector times on a predefined track using onboard sensors.

## Table of Contents

- [Overview](#overview)
- [Nodes](#nodes)
  - [1. `odometer`](#1-odometer)
  - [2. `gps_odometer`](#2-gps_odometer)
  - [3. `sector_times`](#3-sector_times)
- [Topics](#topics)
- [Custom Messages](#custom-messages)

---

## Overview

This ROS package provides:

- Vehicle odometry estimation from speed and steering angle (`odometer` node).
- Odometry estimation from GPS data using ENU transformation (`gps_odometer` node).
- Sector timing logic for a closed-loop track with defined GPS boundaries (`sector_times` node).

---

## Nodes

### 1. `odometer`

Subscribes to vehicle speed and steering angle to compute dead-reckoning odometry using the Ackermann model.

**Subscribed topics:**

- `/speedsteer` (`geometry_msgs/PointStamped`)  
  - `x`: steering angle in degrees  
  - `y`: speed in km/h

**Published topics:**

- `/odom` (`nav_msgs/Odometry`) – Odometry in the `odom` frame  
- TF transform: `odom → vehicle`

---

### 2. `gps_odometer`

Processes raw GPS data to compute position in the ENU (East-North-Up) frame relative to a reference point and publishes it as odometry.

**Subscribed topics:**

- `/swiftnav/front/gps_pose` (`sensor_msgs/NavSatFix`)

**Published topics:**

- `/gps_odom` (`nav_msgs/Odometry`) – Odometry based on GPS in the `odom` frame  
- TF transform: `odom → gps`

> If no GPS reference is given via parameters, it uses the first valid GPS message.

---

### 3. `sector_times`

Tracks progress through a set of predefined GPS-based sector boundaries and publishes sector timing and average speed.

**Subscribed topics:**

- `/speedsteer` (`geometry_msgs/PointStamped`)
- `/swiftnav/front/gps_pose` (`sensor_msgs/NavSatFix`)

**Published topics:**

- `/sector_times` (`first_project/sector_times`)

**Sector definitions:**

Three sectors are defined by line segments between GPS coordinates:

- **1 → 2:** `(45.630247, 9.289481)` to `(45.629969, 9.289499)`
- **2 → 3:** `(45.623658, 9.287174)` to `(45.623476, 9.287370)`
- **3 → 1:** `(45.616058, 9.280528)` to `(45.616018, 9.281218)`

> Crossing a segment from the correct direction triggers a sector transition.

---

## Topics

| Topic                       | Message Type                 | Published By     |
|----------------------------|------------------------------|------------------|
| `/speedsteer`              | `geometry_msgs/PointStamped` | External         |
| `/odom`                    | `nav_msgs/Odometry`          | `odometer`       |
| `/gps_odom`                | `nav_msgs/Odometry`          | `gps_odometer`   |
| `/sector_times`            | `first_project/sector_times` | `sector_times`   |
| `/swiftnav/front/gps_pose` | `sensor_msgs/NavSatFix`      | GPS Source       |

---

## Custom Messages

### `first_project/sector_times.msg`

```text
int32 current_sector
float64 current_sector_time
float32 current_sector_mean_speed

