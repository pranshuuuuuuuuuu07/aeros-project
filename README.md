# aeros-project

An autonomous UAV mission control and global inter-city airport routing system built with Python and `pymavlink` for ArduPlane SITL (Software-In-The-Loop) via Mission Planner MAVLink forwarding.

## Overview

`aeros-project` provides scripts and a mission-control module to plan, upload, and monitor autonomous fixed-wing UAV missions using the MAVLink protocol. It supports real-time telemetry tracking, CSV flight logging, and interactive global airport-to-airport waypoint routing backed by the `airportsdata` ICAO airport database.

## File Breakdown

- **`uav_mission_control.py`**: Core `UAVMissionControl` class built on `pymavlink`. Connects to an external MAVLink endpoint (`tcp:127.0.0.1:5762`), runs a background telemetry listener thread (`lat`, `lon`, `alt`, `vx`, `vy`, `vz`, `gps_fix`, `is_armed`, `mode`), manages mission execution, and logs flight telemetry to CSV.
- **`global_airport_router.py`**: Inter-city global airport routing script that connects to ArduPlane SITL via Mission Planner (`tcp:127.0.0.1:5762`) and executes an autonomous waypoint mission between real-world airport coordinates (such as VASU Surat Airport).
- **`interactive_global_router.py`**: Interactive CLI routing system for ArduPlane using the `airportsdata` ICAO database. Allows the operator to select any two airports worldwide, generates and uploads a MAVLink waypoint mission, and monitors the flight in real time.
- **`requirements.txt`**: Python dependency list (including `pymavlink`, `airportsdata`, `dronekit`, `dronekit-sitl`, and supporting packages).
- **`flight_telemetry.csv`**: Recorded UAV flight telemetry log generated during mission runs.
- **`LICENSE`**: MIT License.

## Setup & Run Instructions

### 1. Prerequisites
- Python 3.8+
- ArduPlane SITL and Mission Planner (configured to forward MAVLink over TCP on `127.0.0.1:5762`)

### 2. Installation
```bash
git clone <repository-url>
cd aeros-project
pip install -r requirements.txt
```

### 3. Running the Scripts
Start ArduPlane SITL and ensure Mission Planner is forwarding MAVLink on `tcp:127.0.0.1:5762`, then run any of the mission scripts:

```bash
# Run the interactive global airport router
python interactive_global_router.py

# Run the pre-configured airport router
python global_airport_router.py

# Run the standalone UAV mission control module
python uav_mission_control.py
```

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
