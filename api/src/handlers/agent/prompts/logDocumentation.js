export const getSystemTimeDocumentation = () => `
## 🛰️ **SYSTEM_TIME**

| Field            | Type     | Unit                               | Description                     |
| ---------------- | -------- | ---------------------------------- | ------------------------------- |
| time_unix_usec | uint64 | microseconds (µs) since UNIX epoch | UTC time from onboard clock     |
| time_boot_ms   | uint32 | milliseconds (ms)                  | Time since boot (system uptime) |

📘 **Purpose:** Synchronizes flight log time with UTC. Useful for aligning with GPS or camera timestamps.
---
`;

export const getGpsDocumentation = () => `
## 🌍 **GLOBAL_POSITION_INT**

| Field          | Type     | Unit             | Description                   |
| -------------- | -------- | ---------------- | ----------------------------- |
| lat          | int32  | 1e-7 degrees     | Latitude                      |
| lon          | int32  | 1e-7 degrees     | Longitude                     |
| alt          | int32  | millimeters (mm) | Altitude above mean sea level |
| relative_alt | int32  | mm               | Altitude above home position  |
| vx           | int16  | cm/s             | Ground speed in X (North)     |
| vy           | int16  | cm/s             | Ground speed in Y (East)      |
| vz           | int16  | cm/s             | Ground speed in Z (Down)      |
| hdg          | uint16 | cdeg (0–35999)   | Heading, 0.01° units          |
| time_boot_ms | uint32 | ms               | Time since boot               |

📘 **Purpose:** Provides estimated global position (from EKF or GPS fusion).  
🗺️ **Common use:** 3D trajectory plotting or flight replay.

---

## 🧭 **GPS_RAW_INT**

| Field                                  | Type     | Unit       | Description                                 |
| -------------------------------------- | -------- | ---------- | ------------------------------------------- |
| time_usec                            | uint64 | µs         | Timestamp of the GPS fix                    |
| fix_type                             | uint8  | —          | 0=No fix, 2=2D fix, 3=3D fix, 4=DGPS, 5=RTK |
| lat                                  | int32  | 1e-7 deg   | Latitude                                    |
| lon                                  | int32  | 1e-7 deg   | Longitude                                   |
| alt                                  | int32  | mm         | Altitude (MSL)                              |
| eph                                  | uint16 | cm         | Horizontal position uncertainty             |
| epv                                  | uint16 | cm         | Vertical position uncertainty               |
| vel                                  | uint16 | cm/s       | Ground speed                                |
| cog                                  | uint16 | cdeg       | Course over ground (0.01°)                  |
| satellites_visible                   | uint8  | count      | Number of visible satellites                |
| alt_ellipsoid                        | int32  | mm         | Altitude above WGS84 ellipsoid              |
| h_acc, v_acc, vel_acc, hdg_acc | uint32 | mm or mm/s | Estimated accuracy metrics                  |
| time_boot_ms                         | uint32 | ms         | Time since boot                             |

📘 **Purpose:** Raw GPS readings directly from the GNSS module.  
🛰️ **Useful for:** Quality checks, GPS drift, and RTK analysis.

---
`;

export const getHeartbeatDocumentation = () => `
## 💓 **HEARTBEAT**

| Field             | Type     | Unit    | Description                                              |
| ----------------- | -------- | ------- | -------------------------------------------------------- |
| type            | uint8  | —       | Vehicle type (e.g., quadcopter = 2)                      |
| autopilot       | uint8  | —       | Autopilot type (e.g., ArduPilot = 3)                     |
| base_mode       | uint8  | bitmask | Flags: safety, manual, guided, armed, etc.               |
| custom_mode     | uint32 | —       | Flight mode identifier                                   |
| system_status   | uint8  | —       | System health (e.g., active, standby, critical)          |
| mavlink_version | uint8  | —       | MAVLink protocol version                                 |
| asText          | string | —       | Human-readable mode (e.g., QLOITER, GUIDED, QLAND) |
| craft           | string | —       | Vehicle name/alias                                       |
| time_boot_ms    | uint32 | ms      | Timestamp                                                |

📘 **Purpose:** Provides high-level system state and mode changes.  
⚙️ **Use case:** Mode timelines or detecting arm/disarm events.

---
`;

export const getAttitudeDocumentation = () => `
## ✈️ **ATTITUDE**

| Field          | Type     | Unit    | Description                     |
| -------------- | -------- | ------- | ------------------------------- |
| time_boot_ms | uint32 | ms      | Time since boot                 |
| roll         | float  | radians | Roll angle (rotation around X)  |
| pitch        | float  | radians | Pitch angle (rotation around Y) |
| yaw          | float  | radians | Yaw angle (rotation around Z)   |
| rollspeed    | float  | rad/s   | Angular rate around X           |
| pitchspeed   | float  | rad/s   | Angular rate around Y           |
| yawspeed     | float  | rad/s   | Angular rate around Z           |

📘 **Purpose:** Orientation and angular motion data.  
🧮 **Commonly used for:** Attitude plots, stabilization analysis, or 3D orientation visualization.

---

## ⚙️ **AHRS / AHRS2 / AHRS3**

| Field                   | Type     | Unit    | Description                             |
| ----------------------- | -------- | ------- | --------------------------------------- |
| roll, pitch, yaw  | float  | radians | Orientation from specific IMU/estimator |
| error_rp, error_yaw | float  | radians | Attitude estimation errors              |
| time_boot_ms          | uint32 | ms      | Timestamp                               |

📘 **Purpose:** Alternate attitude estimates from multiple sources or sensors (redundant IMUs).

---
`;

export const getParamValueDocumentation = () => `
## 🧩 **PARAM_VALUE**

| Field          | Type         | Unit   | Description                              |
| -------------- | ------------ | ------ | ---------------------------------------- |
| param_id     | string[16] | —      | Parameter name (e.g., "ATC_ANG_RLL_P") |
| param_value  | float      | varies | Parameter value                          |
| param_type   | uint8      | —      | Type enum (float, int, etc.)             |
| param_index  | uint16     | —      | Index in total parameter list            |
| time_boot_ms | uint32     | ms     | Timestamp                                |

📘 **Purpose:** Configuration snapshot — helps recreate flight control setup or detect mid-flight parameter changes.

---
`;

export const getStatusTextDocumentation = () => `
## 🗨️ **STATUSTEXT**

| Field          | Type         | Unit | Description                           |
| -------------- | ------------ | ---- | ------------------------------------- |
| severity     | uint8      | —    | Log severity (0–7: Emergency → Debug) |
| text         | string[50] | —    | Status or warning message             |
| time_boot_ms | uint32     | ms   | Timestamp                             |

📘 **Purpose:** Textual system messages (arming warnings, failsafe alerts, GPS lost, etc.).

---
`;

export const getTrajectoriesDocumentation = () => `
### **trajectories**

| Field                 | Type        | Unit                    | Description                                |
| --------------------- | ----------- | ----------------------- | ------------------------------------------ |
| GLOBAL_POSITION_INT | object      | —                       | Derived path from GPS/position data        |
| startAltitude       | float     | meters                  | Starting altitude                          |
| trajectory          | float[][] | [lon, lat, alt, time] | Flight path                                |
| timeTrajectory      | object      | map                     | Timestamp → [lon, lat, alt, time] lookup |

📘 **Purpose:** Precomputed geospatial path for efficient rendering (e.g., 3D flight viewer).
`;

export const getAllLogDocumentation = () => `
# 📄 UAV Log Message Documentation

This document provides an overview of the key UAV log message types analyzed by the UAV Log Viewer tool. Each section describes the fields, purpose, and common use cases for the respective log messages.

---
${getSystemTimeDocumentation()}
${getGpsDocumentation()}
${getHeartbeatDocumentation()}
${getAttitudeDocumentation()}
${getParamValueDocumentation()}
${getStatusTextDocumentation()}
${getTrajectoriesDocumentation()}

`;
