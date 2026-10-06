import time
import math
import csv
import threading
from pymavlink import mavutil

class UAVMissionControl:
    """
    A class to encapsulate UAV mission control logic using purely pymavlink.
    """
    def __init__(self, connection_string='tcp:127.0.0.1:5762', target_altitude=15):
        """
        Initialize the UAV Mission Control instance.

        Connects to an external MAVLink endpoint (e.g. Mission Planner
        forwarding port) rather than managing its own SITL process.
        """
        self.connection_string = connection_string
        self.target_altitude = target_altitude
        self.master = None

        # State variables updated by the listener thread
        self.state_lock = threading.Lock()
        self.is_armed = False
        self.mode = -1
        self.gps_fix = 0
        self.lat = 0.0
        self.lon = 0.0
        self.alt = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        self.listener_thread = None
        self.stop_event = threading.Event()

        self.telemetry_filename = 'flight_telemetry.csv'
        self.telemetry_thread = None
        self.telemetry_stop_event = threading.Event()

    def connect_vehicle(self, max_retries=3):
        """
        Connect to the SITL vehicle via TCP/UDP and start background parsing.
        """
        print(f"Connecting to vehicle on: {self.connection_string}")
        for attempt in range(max_retries):
            try:
                self.master = mavutil.mavlink_connection(self.connection_string)
                self.master.wait_heartbeat(timeout=10)
                print(f"Heartbeat from system (system {self.master.target_system} component {self.master.target_component})")
                break
            except Exception as e:
                print(f"Connection attempt {attempt+1} failed: {e}")
                time.sleep(5)
        else:
            raise Exception("Failed to connect via PyMAVLink.")

        # Start the background listener
        self.listener_thread = threading.Thread(target=self._mavlink_listener, daemon=True)
        self.listener_thread.start()

        # Request data streams to ensure we get position and GPS data frequently
        print("Requesting data streams...")
        self.master.mav.request_data_stream_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)

    def _mavlink_listener(self):
        """Background thread to continuously read MAVLink messages and update state."""
        while not self.stop_event.is_set():
            msg = self.master.recv_match(blocking=True, timeout=0.1)
            if not msg:
                continue
                
            msg_type = msg.get_type()
            with self.state_lock:
                if msg_type == 'HEARTBEAT':
                    self.is_armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                    self.mode = msg.custom_mode
                elif msg_type == 'GPS_RAW_INT':
                    self.gps_fix = msg.fix_type
                elif msg_type == 'GLOBAL_POSITION_INT':
                    self.lat = msg.lat / 1e7
                    self.lon = msg.lon / 1e7
                    self.alt = msg.relative_alt / 1000.0  # mm to meters
                    self.vx = msg.vx / 100.0              # cm/s to m/s
                    self.vy = msg.vy / 100.0
                    self.vz = msg.vz / 100.0

    def get_state(self):
        """Thread-safe state reader."""
        with self.state_lock:
            return {
                'armed': self.is_armed,
                'mode': self.mode,
                'gps_fix': self.gps_fix,
                'alt': self.alt
            }

    def wait_for_gps(self, timeout=60):
        """Wait until a 3D GPS lock is acquired."""
        print("Waiting for GPS lock...")
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.get_state()['gps_fix'] >= 3: # 3D fix
                print("GPS 3D lock acquired.")
                return True
            time.sleep(1)
        print(f"CRITICAL FAILSAFE: Failed to acquire GPS lock within {timeout}s.")
        return False

    def arm_and_takeoff(self):
        """
        Sequence: GPS Fix -> Mode GUIDED -> Arm -> Takeoff
        """
        # 1. GPS Check
        if not self.wait_for_gps(timeout=60):
            return False

        # In pure PyMAVLink without dronekit, waiting for GPS lock is usually enough
        # to ensure home location is set on Copter v3.3 SITL. We give it 2 seconds buffer.
        print(" Stabilizing GPS...")
        time.sleep(2)

        # 2. Mode Switch (While Disarmed)
        print("Switching to GUIDED mode...")
        while self.get_state()['mode'] != 4: # 4 = GUIDED in ArduCopter
            print(f" Current mode: {self.get_state()['mode']}. Forcing GUIDED...")
            
            # Send SET_MODE packet
            self.master.mav.set_mode_send(
                self.master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                4) # 4 = GUIDED
            
            # Poll for up to 3 seconds before resending the mode command
            for _ in range(6):
                if self.get_state()['mode'] == 4:
                    break
                time.sleep(0.5)

        # 3. Arming
        print("Arming motors...")
        while not self.get_state()['armed']:
            print(" Sending ARM command...")
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                1, 21196, 0, 0, 0, 0, 0)
            time.sleep(1)
            
        # 4. Takeoff
        print("Vehicle armed. Stabilizing physics engine for 2 seconds...")
        time.sleep(2)
            
        print(f"Taking off to {self.target_altitude} meters!")
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
            0, 0, 0, 0, 0, 0, self.target_altitude)

        # Wait until the vehicle reaches a safe height
        while True:
            current_altitude = self.get_state()['alt']
            print(f" Altitude: {current_altitude:.1f} m")
            
            # Break and return from function just below target altitude (within 5%)
            if current_altitude >= self.target_altitude * 0.95:
                print("Reached target altitude.")
                break
            time.sleep(1)
            
        return True

    def start_telemetry_logging(self):
        """Start logging telemetry to a CSV file in a background thread."""
        self.telemetry_stop_event.clear()
        self.telemetry_thread = threading.Thread(target=self._log_telemetry, daemon=True)
        self.telemetry_thread.start()

    def stop_telemetry_logging(self):
        """Signal telemetry logging to stop and wait for it."""
        self.telemetry_stop_event.set()
        if self.telemetry_thread:
            self.telemetry_thread.join()

    def _log_telemetry(self):
        with open(self.telemetry_filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Timestamp', 'Lat (Y)', 'Lon (X)', 'Alt (Z)', 'Vx', 'Vy', 'Vz'])
            
            while not self.telemetry_stop_event.is_set() and self.master is not None:
                timestamp = time.time()
                with self.state_lock:
                    writer.writerow([
                        f"{timestamp:.2f}", 
                        self.lat, self.lon, self.alt,
                        self.vx, self.vy, self.vz
                    ])
                self.telemetry_stop_event.wait(0.5)

    def send_ned_velocity(self, velocity_x, velocity_y, velocity_z, duration):
        """
        Move vehicle in direction based on specified velocity vectors using SET_POSITION_TARGET_LOCAL_NED.
        """
        print(f"Commanding velocity Vx: {velocity_x}, Vy: {velocity_y}, Vz: {velocity_z} for {duration:.1f} seconds.")
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.master.mav.set_position_target_local_ned_send(
                0,       # time_boot_ms (not used)
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED, # coordinate frame
                0b0000111111000111, # type_mask (enable only 3D velocity)
                0, 0, 0, # x, y, z positions (not used)
                velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
                0, 0, 0, # x, y, z acceleration (not used)
                0, 0)    # yaw, yaw_rate (not used)
                
            time.sleep(0.1)

    def fly_square_pattern(self, side_length=20, speed=5):
        """
        Fly a square pattern using kinematic velocity commands.
        """
        duration = side_length / speed
        print(f"\n--- Starting {side_length}x{side_length}m square pattern at {speed} m/s ---")
        
        # Leg 1: Fly North (+X direction in NED)
        print("Leg 1: Flying North")
        self.send_ned_velocity(speed, 0, 0, duration)
        
        # Leg 2: Fly East (+Y direction in NED)
        print("Leg 2: Flying East")
        self.send_ned_velocity(0, speed, 0, duration)
        
        # Leg 3: Fly South (-X direction in NED)
        print("Leg 3: Flying South")
        self.send_ned_velocity(-speed, 0, 0, duration)
        
        # Leg 4: Fly West (-Y direction in NED)
        print("Leg 4: Flying West")
        self.send_ned_velocity(0, -speed, 0, duration)
        
        # Stop movement by commanding zero velocity
        print("Stopping vehicle...")
        self.send_ned_velocity(0, 0, 0, 2)
        print("--- Square pattern complete ---\n")

    def return_to_launch(self):
        """
        Trigger RTL and monitor altitude until disarm.
        """
        print("Returning to Launch (RTL)...")
        # Custom Mode 9 is RTL for ArduCopter
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            9) # 9 = RTL
            
        while self.get_state()['armed']:
            current_altitude = self.get_state()['alt']
            print(f" RTL Descending... Altitude: {current_altitude:.1f} m")
            time.sleep(2)
            
        print("Vehicle landed and disarmed successfully.")
        
    def close(self):
        """
        Clean up the vehicle connection.
        """
        self.stop_event.set()
        if self.listener_thread:
            self.listener_thread.join(timeout=2)

        if self.master:
            self.master.close()
            print("Vehicle connection closed.")


def main():
    # 1. Initialize mission controller
    mission = UAVMissionControl(target_altitude=15)
    
    try:
        # 2. Connect to the simulation via raw PyMAVLink
        mission.connect_vehicle()
        
        # 3. Takeoff
        if not mission.arm_and_takeoff():
            return
            
        # 4. Start background telemetry logging at 2 Hz
        print("Starting background telemetry logging...")
        mission.start_telemetry_logging()
        
        # 5. Fly the 20x20 meter square pattern
        mission.fly_square_pattern(side_length=20, speed=5)
        
        # 6. Return to launch and land
        mission.return_to_launch()
        
    except KeyboardInterrupt:
        print("\nMission interrupted by user (Ctrl+C). Triggering RTL failsafe...")
        if mission.get_state()['armed']:
            mission.return_to_launch()
            
    finally:
        # 7. Always stop logging and close connection cleanly
        mission.stop_telemetry_logging()
        mission.close()
        print("Mission script terminated.")


if __name__ == '__main__':
    main()