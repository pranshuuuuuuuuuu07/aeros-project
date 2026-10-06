"""
global_airport_router.py

Inter-City Global Airport Routing System
─────────────────────────────────────────
Connects to an external ArduPlane SITL (via Mission Planner MAVLink
forwarding on tcp:127.0.0.1:5762) and executes an autonomous waypoint
mission between real-world Indian airport coordinates.

Architecture
────────────
  ┌──────────────┐   tcp:5762   ┌──────────────┐   tcp:5760   ┌──────────┐
  │  This Script │◄────────────►│Mission Planner│◄────────────►│ArduPlane │
  │  (GNC Client)│   MAVLink    │  (GCS Relay)  │   MAVLink    │   SITL   │
  └──────────────┘              └──────────────┘              └──────────┘
"""

import time
import math
import threading
from pymavlink import mavutil


# ═══════════════════════════════════════════════════════════════════════
#  1. Airport Coordinate Database  (ICAO → WGS-84)
# ═══════════════════════════════════════════════════════════════════════

AIRPORTS = {
    'VASU': {
        'name': 'Surat Airport',
        'lat':  21.1155,
        'lon':  72.7433,
    },
    'VABB': {
        'name': 'Mumbai – Chhatrapati Shivaji Maharaj Intl',
        'lat':  19.0896,
        'lon':  72.8656,
    },
    'VIDP': {
        'name': 'New Delhi – Indira Gandhi Intl',
        'lat':  28.5562,
        'lon':  77.1000,
    },
}

EARTH_RADIUS_M = 6_371_000  # Mean Earth radius (metres)


# ═══════════════════════════════════════════════════════════════════════
#  Utility – Haversine great-circle distance
# ═══════════════════════════════════════════════════════════════════════

def haversine(lat1, lon1, lat2, lon2):
    """Return the great-circle distance in metres between two WGS-84 points."""
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi  = math.radians(lat2 - lat1)
    dlam  = math.radians(lon2 - lon1)
    a = (math.sin(dphi / 2) ** 2
         + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2)
    return EARTH_RADIUS_M * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


# ═══════════════════════════════════════════════════════════════════════
#  2. GlobalAirportRouter – core GNC client
# ═══════════════════════════════════════════════════════════════════════

class GlobalAirportRouter:
    """
    Pure-pymavlink client that uploads a global waypoint mission to an
    ArduPlane autopilot (SITL or real) and monitors its execution.
    """

    # ArduPlane custom-mode constants
    MODE_MANUAL  = 0
    MODE_AUTO    = 10
    MODE_GUIDED  = 15

    def __init__(self, connection_string='tcp:127.0.0.1:5762'):
        """
        Initialise the router.

        Parameters
        ----------
        connection_string : str
            MAVLink endpoint, e.g. Mission Planner's forwarding port.
        """
        self.connection_string = connection_string
        self.master = None

        # ── Telemetry state (written by the listener thread) ──
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

        # Stored flight plan for distance look-ups in the monitor
        self.flight_plan = []

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
    #  Background MAVLink Listener
    # ──────────────────────────────────────────────────────────────

    def start_listener(self):
        """Spin up the background message-parser thread."""
        self.listener_thread = threading.Thread(
            target=self._mavlink_listener, daemon=True)
        self.listener_thread.start()

    def _mavlink_listener(self):
        """Continuously read MAVLink messages and update shared state."""
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
                    self.alt     = msg.relative_alt / 1000.0   # mm → m
                    self.heading = (msg.hdg / 100.0
                                    if msg.hdg != 65535 else 0.0)

                elif mtype == 'VFR_HUD':
                    self.groundspeed = msg.groundspeed

                elif mtype == 'MISSION_CURRENT':
                    self.wp_seq = msg.seq

    def get_state(self):
        """Thread-safe snapshot of vehicle telemetry."""
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
    #  3. Flight-Plan Builder
    # ──────────────────────────────────────────────────────────────

    def build_flight_plan(self):
        """
        Generate MISSION_ITEM_INT entries for the inter-city route.

          WP 0 – Dummy home      (VASU,  0 m AGL)
          WP 1 – NAV_TAKEOFF     (VASU, 100 m AGL)
          WP 2 – NAV_WAYPOINT    (VABB, 5 000 m AGL)
          WP 3 – NAV_WAYPOINT    (VIDP, 5 000 m AGL)
        """
        home   = AIRPORTS['VASU']
        mumbai = AIRPORTS['VABB']
        delhi  = AIRPORTS['VIDP']

        plan = [
            # ── WP 0: Dummy Home ──────────────────────────────────
            {
                'seq':     0,
                'frame':   mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                'command': mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                'current': 0,  'autocontinue': 1,
                'param1': 0, 'param2': 0, 'param3': 0, 'param4': 0,
                'x': int(home['lat'] * 1e7),
                'y': int(home['lon'] * 1e7),
                'z': 0,
                'label': f"HOME / VASU – {home['name']}",
                'icao':  'VASU',
            },
            # ── WP 1: Takeoff at Surat ────────────────────────────
            {
                'seq':     1,
                'frame':   mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                'command': mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                'current': 1,  'autocontinue': 1,
                'param1': 15,   # minimum pitch (deg)
                'param2': 0, 'param3': 0, 'param4': 0,
                'x': int(home['lat'] * 1e7),
                'y': int(home['lon'] * 1e7),
                'z': 100,       # initial climb altitude (m AGL)
                'label': f"TAKEOFF / VASU – {home['name']}  →  100 m AGL",
                'icao':  'VASU',
            },
            # ── WP 2: Mumbai ─────────────────────────────────────
            {
                'seq':     2,
                'frame':   mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                'command': mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                'current': 0,  'autocontinue': 1,
                'param1': 0, 'param2': 0, 'param3': 0, 'param4': 0,
                'x': int(mumbai['lat'] * 1e7),
                'y': int(mumbai['lon'] * 1e7),
                'z': 5000,
                'label': f"VABB – {mumbai['name']}  @  5 000 m",
                'icao':  'VABB',
            },
            # ── WP 3: New Delhi ───────────────────────────────────
            {
                'seq':     3,
                'frame':   mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                'command': mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                'current': 0,  'autocontinue': 1,
                'param1': 0, 'param2': 0, 'param3': 0, 'param4': 0,
                'x': int(delhi['lat'] * 1e7),
                'y': int(delhi['lon'] * 1e7),
                'z': 5000,
                'label': f"VIDP – {delhi['name']}  @  5 000 m",
                'icao':  'VIDP',
            },
        ]

        self.flight_plan = plan
        return plan

    # ──────────────────────────────────────────────────────────────
    #  Flight-Plan Pretty Printer
    # ──────────────────────────────────────────────────────────────

    @staticmethod
    def print_flight_plan(plan):
        """Display the filed route with per-leg distances."""
        print("=" * 74)
        print("  FLIGHT PLAN  –  Global Airport Router")
        print("=" * 74)

        total_km = 0.0
        prev = None
        for wp in plan:
            lat = wp['x'] / 1e7
            lon = wp['y'] / 1e7
            leg = ""
            if prev is not None:
                d = haversine(prev[0], prev[1], lat, lon)
                total_km += d / 1000.0
                leg = f"  [{d / 1000:.1f} km]"
            print(f"  WP {wp['seq']}  │  {wp['label']}{leg}")
            prev = (lat, lon)

        print("─" * 74)
        print(f"  Total route distance: {total_km:.1f} km")
        print("=" * 74 + "\n")

    # ──────────────────────────────────────────────────────────────
    #  2. Mission Protocol Upload  (MAVLink handshake)
    # ──────────────────────────────────────────────────────────────

    def upload_mission(self, mission_items):
        """
        Full MAVLink mission-upload handshake:

          MISSION_CLEAR_ALL  →  ACK
          MISSION_COUNT(n)   →  autopilot sends MISSION_REQUEST × n
                             ←  respond with MISSION_ITEM_INT each time
          Final MISSION_ACK

        NOTE: Must be called *before* starting the background listener,
        so that recv_match calls here are not racing with the listener.
        """
        n = len(mission_items)

        # ── Step 1: Clear any existing mission ────────────────────
        print("[UPLOAD] Clearing existing mission...")
        self.master.mav.mission_clear_all_send(
            self.master.target_system,
            self.master.target_component)
        ack = self.master.recv_match(
            type='MISSION_ACK', blocking=True, timeout=5)
        if ack is None:
            raise TimeoutError("No ACK received for MISSION_CLEAR_ALL.")
        print(f"  Mission cleared  (ACK type={ack.type})")

        # ── Step 2: Announce waypoint count ───────────────────────
        print(f"[UPLOAD] Sending MISSION_COUNT = {n}")
        self.master.mav.mission_count_send(
            self.master.target_system,
            self.master.target_component,
            n)

        # ── Step 3: Respond to each MISSION_REQUEST ──────────────
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

        # ── Step 4: Await final acceptance ────────────────────────
        ack = self.master.recv_match(
            type='MISSION_ACK', blocking=True, timeout=10)
        if ack is None:
            raise TimeoutError("No final MISSION_ACK received.")
        if ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
            raise RuntimeError(
                f"Mission rejected by autopilot (ACK type={ack.type}).")
        print("[UPLOAD] ✓ Mission accepted by autopilot.\n")

    # ──────────────────────────────────────────────────────────────
    #  4. Execution – AUTO mode, Arm, Mission Start
    # ──────────────────────────────────────────────────────────────

    def switch_to_auto_and_arm(self):
        """
        Set ArduPlane AUTO mode (custom_mode 10), force-arm the throttle
        with the 21196 magic parameter, and issue MISSION_START.
        """
        # ── AUTO mode ────────────────────────────────────────────
        print("Setting AUTO mode (custom_mode=10)...")
        for _ in range(30):                         # up to ~30 s
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

        # ── Arm (force-arm with 21196 magic) ─────────────────────
        print("Arming throttle (force-arm)...")
        for _ in range(30):
            if self.get_state()['armed']:
                break
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                1,      # arm
                21196,  # force-arm magic number
                0, 0, 0, 0, 0)
            time.sleep(1)
        else:
            raise TimeoutError("Could not arm the vehicle.")
        print("  Vehicle ARMED.")

        # ── Trigger MISSION_START ────────────────────────────────
        print("Sending MISSION_START command...\n")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_MISSION_START, 0,
            0, 0, 0, 0, 0, 0, 0)
        time.sleep(1)

    # ──────────────────────────────────────────────────────────────
    #  5. Live Telemetry Monitor
    # ──────────────────────────────────────────────────────────────

    def monitor_flight(self):
        """
        Blocking loop that prints distance-to-next-waypoint, altitude,
        groundspeed, and heading every 3 seconds until the vehicle
        disarms (mission complete) or the operator presses Ctrl-C.
        """
        print("=" * 74)
        print("  LIVE TELEMETRY  –  Inter-City Route Monitor")
        print("=" * 74)
        header = (f"  {'WP':>3}  {'Target ICAO':<8}  {'Dist (km)':>10}  "
                  f"{'Alt (m)':>10}  {'GS (m/s)':>9}  {'Hdg':>6}")
        print(header)
        print("─" * 74)

        was_armed = False

        while True:
            state = self.get_state()

            # Detect mission completion (armed → disarmed transition)
            if state['armed']:
                was_armed = True
            elif was_armed:
                print("\n  ✓ Vehicle disarmed – mission complete.")
                break

            seq      = state['wp_seq']
            icao     = "—"
            dist_km  = 0.0

            # Look up the target waypoint in our stored flight plan
            if seq < len(self.flight_plan):
                wp   = self.flight_plan[seq]
                icao = wp.get('icao', f'WP{seq}')
                target_lat = wp['x'] / 1e7
                target_lon = wp['y'] / 1e7
                dist_m  = haversine(state['lat'], state['lon'],
                                    target_lat, target_lon)
                dist_km = dist_m / 1000.0

            print(f"  {seq:>3}  {icao:<8}  {dist_km:>10.2f}  "
                  f"{state['alt']:>10.1f}  {state['groundspeed']:>9.1f}  "
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
    router = GlobalAirportRouter()

    try:
        # 1. Connect to external ArduPlane autopilot / SITL
        router.connect()

        # 2. Build and display the flight plan
        flight_plan = router.build_flight_plan()
        router.print_flight_plan(flight_plan)

        # 3. Upload mission via MAVLink protocol handshake
        #    (done BEFORE starting listener to avoid recv_match races)
        router.upload_mission(flight_plan)

        # 4. Start background telemetry listener
        router.start_listener()

        # 5. Switch to AUTO, arm, and start the mission
        router.switch_to_auto_and_arm()

        # 6. Monitor in-flight telemetry until landing / disarm
        router.monitor_flight()

    except KeyboardInterrupt:
        print("\n\n  Mission interrupted by operator (Ctrl+C).")
    finally:
        router.close()
        print("Global Airport Router terminated.")


if __name__ == '__main__':
    main()
