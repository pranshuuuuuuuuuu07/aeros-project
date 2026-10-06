"""
interactive_global_router.py

Interactive Inter-City Global Routing System for ArduPlane
──────────────────────────────────────────────────────────
Uses the `airportsdata` ICAO database to let the operator pick any two
airports on Earth, then uploads a MAVLink waypoint mission and monitors
the flight in real time.

Architecture
────────────
  Operator CLI  →  This Script  ──tcp:5762──►  Mission Planner  ──►  ArduPlane SITL

Dependencies
────────────
  pip install pymavlink airportsdata
"""

import time
import math
import threading

from pymavlink import mavutil

try:
    import airportsdata
except ImportError:
    print("\n  ERROR: The 'airportsdata' package is required.")
    print("  Install it with:  pip install airportsdata\n")
    raise SystemExit(1)


# ═══════════════════════════════════════════════════════════════════════
#  Constants
# ═══════════════════════════════════════════════════════════════════════

EARTH_RADIUS_M = 6_371_000  # Mean Earth radius (metres)


# ═══════════════════════════════════════════════════════════════════════
#  Utility – Haversine great-circle distance
# ═══════════════════════════════════════════════════════════════════════

def haversine(lat1, lon1, lat2, lon2):
    """Return the great-circle distance in metres between two WGS-84 points."""
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = (math.sin(dphi / 2) ** 2
         + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2)
    return EARTH_RADIUS_M * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


# ═══════════════════════════════════════════════════════════════════════
#  1. Interactive CLI – Flight Parameter Input
# ═══════════════════════════════════════════════════════════════════════

def get_flight_parameters():
    """
    Prompt the operator for origin/destination ICAO codes, cruising
    altitude, and simulation speed multiplier.  Validates every input
    against the airportsdata dictionary before continuing.
    """
    airports = airportsdata.load('icao')

    print("\n" + "=" * 70)
    print("  ✈  INTERACTIVE GLOBAL AIRPORT ROUTER")
    print("     ArduPlane · pymavlink · MAVLink Mission Protocol")
    print("=" * 70)
    print(f"  {len(airports):,} airports loaded from ICAO database.\n")

    # ── Origin ────────────────────────────────────────────────────
    while True:
        origin_icao = input("  Enter ORIGIN airport ICAO code      : ").strip().upper()
        if origin_icao in airports:
            origin = airports[origin_icao]
            print(f"    ✓ {origin['name']}")
            print(f"      Lat {origin['lat']:.4f}   Lon {origin['lon']:.4f}\n")
            break
        print(f"    ✗ '{origin_icao}' not found in database. Try again.\n")

    # ── Destination ───────────────────────────────────────────────
    while True:
        dest_icao = input("  Enter DESTINATION airport ICAO code  : ").strip().upper()
        if dest_icao not in airports:
            print(f"    ✗ '{dest_icao}' not found in database. Try again.\n")
            continue
        if dest_icao == origin_icao:
            print("    ✗ Destination must differ from origin.\n")
            continue
        dest = airports[dest_icao]
        print(f"    ✓ {dest['name']}")
        print(f"      Lat {dest['lat']:.4f}   Lon {dest['lon']:.4f}\n")
        break

    # ── Cruising Altitude ─────────────────────────────────────────
    while True:
        raw = input("  Enter CRUISING ALTITUDE in metres    : ").strip()
        try:
            altitude = int(raw)
            if 100 <= altitude <= 20_000:
                print(f"    ✓ {altitude:,} m AGL\n")
                break
            print("    ✗ Must be between 100 and 20,000 m.\n")
        except ValueError:
            print("    ✗ Please enter a valid integer.\n")

    # ── Simulation Speed ──────────────────────────────────────────
    while True:
        raw = input("  Enter SIMULATION SPEED multiplier (1–12) : ").strip()
        try:
            sim_speed = int(raw)
            if 1 <= sim_speed <= 12:
                print(f"    ✓ {sim_speed}× real-time\n")
                break
            print("    ✗ Must be between 1 and 12.\n")
        except ValueError:
            print("    ✗ Please enter a valid integer.\n")

    # ── Route Summary ─────────────────────────────────────────────
    dist_km = haversine(
        origin['lat'], origin['lon'],
        dest['lat'],   dest['lon']) / 1000.0

    print("─" * 70)
    print(f"  ROUTE       {origin_icao}  →  {dest_icao}")
    print(f"  ORIGIN      {origin['name']}")
    print(f"  DESTINATION {dest['name']}")
    print(f"  DISTANCE    {dist_km:,.1f} km   (great-circle)")
    print(f"  ALTITUDE    {altitude:,} m AGL")
    print(f"  SIM SPEED   {sim_speed}×")
    print("─" * 70 + "\n")

    return {
        'origin_icao': origin_icao,
        'origin_lat':  origin['lat'],
        'origin_lon':  origin['lon'],
        'origin_name': origin['name'],
        'dest_icao':   dest_icao,
        'dest_lat':    dest['lat'],
        'dest_lon':    dest['lon'],
        'dest_name':   dest['name'],
        'altitude':    altitude,
        'sim_speed':   sim_speed,
        'distance_km': dist_km,
    }


# ═══════════════════════════════════════════════════════════════════════
#  2. InteractiveGlobalRouter – core GNC client
# ═══════════════════════════════════════════════════════════════════════

class InteractiveGlobalRouter:
    """
    Pure-pymavlink client that connects to an external ArduPlane SITL
    via Mission Planner, uploads a two-waypoint global mission, and
    monitors its execution with live telemetry.
    """

    # ArduPlane custom-mode constant
    MODE_AUTO = 10

    def __init__(self, connection_string='tcp:127.0.0.1:5762'):
        self.connection_string = connection_string
        self.master = None

        # ── Telemetry state  (written by the listener thread) ─────
        self.state_lock   = threading.Lock()
        self.lat          = 0.0
        self.lon          = 0.0
        self.alt          = 0.0
        self.heading      = 0.0
        self.groundspeed  = 0.0
        self.wp_seq       = 0
        self.is_armed     = False
        self.mode         = -1

        self.listener_thread = None
        self.stop_event      = threading.Event()

    # ──────────────────────────────────────────────────────────────
    #  Connection
    # ──────────────────────────────────────────────────────────────

    def connect(self, max_retries=3):
        """Establish a MAVLink connection and wait for a heartbeat."""
        print(f"Connecting to autopilot on {self.connection_string} ...")
        for attempt in range(max_retries):
            try:
                self.master = mavutil.mavlink_connection(self.connection_string)
                self.master.wait_heartbeat(timeout=10)
                print(f"  Heartbeat received  "
                      f"(sys {self.master.target_system}  "
                      f"comp {self.master.target_component})")
                break
            except Exception as exc:
                print(f"  Attempt {attempt + 1}/{max_retries} failed: {exc}")
                time.sleep(5)
        else:
            raise ConnectionError("Could not connect to the autopilot.")

        # Request all telemetry streams at 4 Hz
        self.master.mav.request_data_stream_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
        print("  Data streams requested (4 Hz).\n")

    # ──────────────────────────────────────────────────────────────
    #  3. Time Dilation – PARAM_SET for SIM_SPEEDUP
    # ──────────────────────────────────────────────────────────────

    def set_sim_speedup(self, multiplier):
        """
        Send a PARAM_SET MAVLink message to change the ArduPilot
        SIM_SPEEDUP parameter, then wait for the PARAM_VALUE echo
        to confirm the new value.
        """
        print(f"Setting SIM_SPEEDUP to {multiplier}× ...")

        self.master.mav.param_set_send(
            self.master.target_system,
            self.master.target_component,
            b'SIM_SPEEDUP',                         # param_id  (16-byte field)
            float(multiplier),                       # param_value
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32)   # param_type

        # Wait for the autopilot's PARAM_VALUE confirmation
        ack = self.master.recv_match(
            type='PARAM_VALUE', blocking=True, timeout=5)
        if ack and 'SIM_SPEEDUP' in str(ack.param_id):
            print(f"  ✓ SIM_SPEEDUP confirmed at {ack.param_value:.0f}×\n")
        else:
            print("  ⚠ No confirmation received – proceeding anyway.\n")

    # ──────────────────────────────────────────────────────────────
    #  Background MAVLink Listener
    # ──────────────────────────────────────────────────────────────

    def start_listener(self):
        """Spin up the background message-parser thread."""
        self.listener_thread = threading.Thread(
            target=self._mavlink_listener, daemon=True)
        self.listener_thread.start()

    def _mavlink_listener(self):
        """Continuously parse MAVLink messages into shared state."""
        while not self.stop_event.is_set():
            msg = self.master.recv_match(blocking=True, timeout=0.5)
            if msg is None:
                continue
            mtype = msg.get_type()

            with self.state_lock:
                if mtype == 'HEARTBEAT':
                    self.is_armed = bool(
                        msg.base_mode
                        & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    self.mode = msg.custom_mode

                elif mtype == 'GLOBAL_POSITION_INT':
                    self.lat     = msg.lat / 1e7
                    self.lon     = msg.lon / 1e7
                    self.alt     = msg.relative_alt / 1000.0
                    self.heading = (msg.hdg / 100.0
                                    if msg.hdg != 65535 else 0.0)

                elif mtype == 'VFR_HUD':
                    self.groundspeed = msg.groundspeed

                elif mtype == 'MISSION_CURRENT':
                    self.wp_seq = msg.seq

    def get_state(self):
        """Thread-safe snapshot of current vehicle telemetry."""
        with self.state_lock:
            return {
                'armed':       self.is_armed,
                'mode':        self.mode,
                'lat':         self.lat,
                'lon':         self.lon,
                'alt':         self.alt,
                'heading':     self.heading,
                'groundspeed': self.groundspeed,
                'wp_seq':      self.wp_seq,
            }

    # ──────────────────────────────────────────────────────────────
    #  4. Flight-Plan Builder
    # ──────────────────────────────────────────────────────────────

    @staticmethod
    def build_flight_plan(params):
        """
        Generate MISSION_ITEM_INT entries:

          WP 0 – Dummy home      (origin,  0 m AGL)
          WP 1 – NAV_TAKEOFF     (origin, 100 m AGL, 15° pitch)
          WP 2 – NAV_WAYPOINT    (destination, cruising alt)
        """
        plan = [
            # ── WP 0: Dummy Home ──────────────────────────────────
            {
                'seq':     0,
                'frame':   mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                'command': mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                'current': 0,  'autocontinue': 1,
                'param1': 0, 'param2': 0, 'param3': 0, 'param4': 0,
                'x': int(params['origin_lat'] * 1e7),
                'y': int(params['origin_lon'] * 1e7),
                'z': 0,
                'label': f"HOME / {params['origin_icao']} – {params['origin_name']}",
            },
            # ── WP 1: Takeoff ─────────────────────────────────────
            {
                'seq':     1,
                'frame':   mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                'command': mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                'current': 1,  'autocontinue': 1,
                'param1': 15,   # minimum pitch (degrees)
                'param2': 0, 'param3': 0, 'param4': 0,
                'x': int(params['origin_lat'] * 1e7),
                'y': int(params['origin_lon'] * 1e7),
                'z': 100,       # initial climb target (m AGL)
                'label': (f"TAKEOFF / {params['origin_icao']} "
                          f"– {params['origin_name']}  →  100 m AGL"),
            },
            # ── WP 2: Destination ─────────────────────────────────
            {
                'seq':     2,
                'frame':   mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                'command': mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                'current': 0,  'autocontinue': 1,
                'param1': 0, 'param2': 0, 'param3': 0, 'param4': 0,
                'x': int(params['dest_lat'] * 1e7),
                'y': int(params['dest_lon'] * 1e7),
                'z': params['altitude'],
                'label': (f"{params['dest_icao']} – {params['dest_name']}"
                          f"  @  {params['altitude']:,} m AGL"),
            },
        ]
        return plan

    # ──────────────────────────────────────────────────────────────
    #  Mission Protocol Upload  (MAVLink handshake)
    # ──────────────────────────────────────────────────────────────

    def upload_mission(self, mission_items):
        """
        Full MAVLink mission-upload handshake:

          MISSION_CLEAR_ALL    →  ACK
          MISSION_COUNT(n)     →  autopilot sends MISSION_REQUEST × n
                               ←  respond with MISSION_ITEM_INT each time
          Final MISSION_ACK

        NOTE: Must be called *before* starting the background listener
        so that recv_match calls are not racing with the listener thread.
        """
        n = len(mission_items)

        # ── Step 1: Clear ─────────────────────────────────────────
        print("[UPLOAD] Clearing existing mission...")
        self.master.mav.mission_clear_all_send(
            self.master.target_system,
            self.master.target_component)
        ack = self.master.recv_match(
            type='MISSION_ACK', blocking=True, timeout=5)
        if ack is None:
            raise TimeoutError("No ACK received for MISSION_CLEAR_ALL.")
        print(f"  Mission cleared  (ACK type={ack.type})")

        # ── Step 2: Count ─────────────────────────────────────────
        print(f"[UPLOAD] Sending MISSION_COUNT = {n}")
        self.master.mav.mission_count_send(
            self.master.target_system,
            self.master.target_component,
            n)

        # ── Step 3: Respond to each MISSION_REQUEST ───────────────
        for _ in range(n):
            req = self.master.recv_match(
                type=['MISSION_REQUEST', 'MISSION_REQUEST_INT'],
                blocking=True, timeout=10)
            if req is None:
                raise TimeoutError(
                    "Timed out waiting for MISSION_REQUEST from autopilot.")

            wp = mission_items[req.seq]

            self.master.mav.mission_item_int_send(
                self.master.target_system,
                self.master.target_component,
                wp['seq'],
                wp['frame'],
                wp['command'],
                wp['current'],
                wp['autocontinue'],
                wp['param1'], wp['param2'], wp['param3'], wp['param4'],
                wp['x'], wp['y'], wp['z'],
                0)  # MAV_MISSION_TYPE_MISSION

            print(f"  → WP {req.seq}: {wp['label']}")

        # ── Step 4: Final ACK ─────────────────────────────────────
        ack = self.master.recv_match(
            type='MISSION_ACK', blocking=True, timeout=10)
        if ack is None:
            raise TimeoutError("No final MISSION_ACK received.")
        if ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
            raise RuntimeError(
                f"Mission rejected by autopilot (ACK type={ack.type}).")
        print("[UPLOAD] ✓ Mission accepted by autopilot.\n")

    # ──────────────────────────────────────────────────────────────
    #  5. Execution – AUTO mode, Force-Arm, Mission Start
    # ──────────────────────────────────────────────────────────────

    def switch_to_auto_and_arm(self):
        """
        Set ArduPlane AUTO mode (custom_mode 10), force-arm the throttle
        with the 21196 magic parameter, and issue MAV_CMD_MISSION_START.
        """
        # ── AUTO mode ────────────────────────────────────────────
        print("Setting AUTO mode (custom_mode=10)...")
        for _ in range(30):
            if self.get_state()['mode'] == self.MODE_AUTO:
                break
            self.master.mav.set_mode_send(
                self.master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                self.MODE_AUTO)
            time.sleep(1)
        else:
            raise TimeoutError("Could not switch to AUTO mode.")
        print("  AUTO mode confirmed.")

        # ── Force-arm ────────────────────────────────────────────
        print("Arming throttle (force-arm 21196)...")
        for _ in range(30):
            if self.get_state()['armed']:
                break
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                1,       # arm
                21196,   # force-arm magic number
                0, 0, 0, 0, 0)
            time.sleep(1)
        else:
            raise TimeoutError("Could not arm the vehicle.")
        print("  Vehicle ARMED.")

        # ── Mission Start ────────────────────────────────────────
        print("Sending MISSION_START command...\n")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_MISSION_START, 0,
            0, 0, 0, 0, 0, 0, 0)
        time.sleep(1)

    # ──────────────────────────────────────────────────────────────
    #  6. Live Telemetry Monitor
    # ──────────────────────────────────────────────────────────────

    def monitor_flight(self, params):
        """
        Blocking loop that calculates and prints the remaining
        great-circle distance to the destination, altitude, ground
        speed, and heading every 3 seconds.  Exits when the vehicle
        transitions from armed → disarmed (mission complete) or
        the operator presses Ctrl-C.
        """
        dest_lat   = params['dest_lat']
        dest_lon   = params['dest_lon']
        dest_icao  = params['dest_icao']
        total_dist = params['distance_km']

        print("=" * 74)
        print(f"  ✈  LIVE TELEMETRY   {params['origin_icao']}  →  {dest_icao}")
        print(f"     Route {total_dist:,.1f} km  ·  "
              f"Cruise {params['altitude']:,} m  ·  "
              f"Sim {params['sim_speed']}×")
        print("=" * 74)
        print(f"  {'Remaining':>12}  {'Progress':<24}  "
              f"{'Alt (m)':>9}  {'GS (m/s)':>9}  {'Hdg':>6}")
        print("─" * 74)

        was_armed = False

        while True:
            state = self.get_state()

            # Detect mission completion (armed → disarmed transition)
            if state['armed']:
                was_armed = True
            elif was_armed:
                print(f"\n  ✓ Vehicle disarmed – arrived at {dest_icao}.")
                break

            # Great-circle distance to destination
            remaining_m  = haversine(
                state['lat'], state['lon'], dest_lat, dest_lon)
            remaining_km = remaining_m / 1000.0

            # Progress percentage (clamped 0–100)
            if total_dist > 0:
                progress = max(0.0, min(100.0,
                    (1.0 - remaining_km / total_dist) * 100.0))
            else:
                progress = 0.0

            # Visual progress bar
            bar_width = 16
            filled    = int(bar_width * progress / 100.0)
            bar       = "█" * filled + "░" * (bar_width - filled)

            print(f"  {remaining_km:>10.1f} km  {bar} {progress:>5.1f}%  "
                  f"{state['alt']:>9.1f}  {state['groundspeed']:>9.1f}  "
                  f"{state['heading']:>5.0f}°")

            time.sleep(3)

    # ──────────────────────────────────────────────────────────────
    #  Cleanup
    # ──────────────────────────────────────────────────────────────

    def close(self):
        """Shut down the listener thread and close the MAVLink connection."""
        self.stop_event.set()
        if self.listener_thread:
            self.listener_thread.join(timeout=2)
        if self.master:
            self.master.close()
            print("Connection closed.")


# ═══════════════════════════════════════════════════════════════════════
#  Entry Point
# ═══════════════════════════════════════════════════════════════════════

def main():
    # ── 1. Interactive flight-parameter input (before any MAVLink) ─
    params = get_flight_parameters()

    router = InteractiveGlobalRouter()

    try:
        # ── 2. Connect to external ArduPlane autopilot / SITL ─────
        router.connect()

        # ── 3. Set simulation time dilation via PARAM_SET ─────────
        #       (done before listener so recv_match works cleanly)
        router.set_sim_speedup(params['sim_speed'])

        # ── 4. Build and upload the mission ───────────────────────
        flight_plan = router.build_flight_plan(params)
        router.upload_mission(flight_plan)

        # ── 5. Start the background telemetry listener ────────────
        router.start_listener()

        # ── 6. Switch to AUTO, force-arm, and launch ──────────────
        router.switch_to_auto_and_arm()

        # ── 7. Monitor in-flight telemetry until mission complete ─
        router.monitor_flight(params)

    except KeyboardInterrupt:
        print("\n\n  Mission interrupted by operator (Ctrl+C).")
    finally:
        router.close()
        print("Interactive Global Router terminated.")


if __name__ == '__main__':
    main()
