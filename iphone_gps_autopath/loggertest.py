"""SensorLog UDP reader with optional waypoint navigation commands."""

import atexit
import csv
import json
import math
import select
import socket
import sys
import termios
import tty

# SensorLog stream settings
SENSORLOG_IP = "172.20.10.3"  # informational only (UDP source is the phone)
SENSORLOG_PORT = 505
MAX_BYTES = 65535

# Navigation
WAYPOINT_RADIUS = 8.0
TURN_THRESHOLD = 12.0

# Header orders seen in SensorLog CSV export/stream
FULL_HEADER = [
    "loggingTime",
    "loggingSample",
    "locationTimestamp_since1970",
    "locationLatitude",
    "locationLongitude",
    "locationAltitude",
    "locationSpeed",
    "locationSpeedAccuracy",
    "locationCourse",
    "locationCourseAccuracy",
    "locationVerticalAccuracy",
    "locationHorizontalAccuracy",
    "locationFloor",
    "locationHeadingTimestamp_since1970",
    "locationHeadingX",
    "locationHeadingY",
    "locationHeadingZ",
    "locationTrueHeading",
    "locationMagneticHeading",
    "locationHeadingAccuracy",
]

HEADING_ONLY_HEADER = [
    "loggingTime",
    "loggingSample",
    "locationHeadingTimestamp_since1970",
    "locationHeadingX",
    "locationHeadingY",
    "locationHeadingZ",
    "locationTrueHeading",
    "locationMagneticHeading",
    "locationHeadingAccuracy",
]

DISPLAY_FIELDS = [
    ("locationTimestamp_since1970", "locationTimestamp(s)"),
    ("locationLatitude", "lat"),
    ("locationLongitude", "lon"),
    ("locationAltitude", "alt(m)"),
    ("locationSpeed", "speed(m/s)"),
    ("locationCourse", "course(°)"),
    ("locationTrueHeading", "trueHeading(°)"),
    ("locationMagneticHeading", "magHeading(°)"),
    ("locationHeadingAccuracy", "headingAcc(°)"),
]

header = None
route = []
wp_index = 0
last_print_wp = -1
captured_waypoints = []
latest_gps = None
term_state = None


def parse_csv(line):
    try:
        return next(csv.reader([line.strip()]))
    except Exception:
        return None


def looks_like_timestamp(value):
    return isinstance(value, str) and "T" in value and ":" in value


def is_int_like(value):
    try:
        int(str(value).strip())
        return True
    except Exception:
        return False


def looks_like_header(items):
    if not items:
        return False
    # header rows usually contain names (letters), not timestamp+integer pair.
    if looks_like_timestamp(items[0]) and len(items) > 1 and is_int_like(items[1]):
        return False
    return any(any(ch.isalpha() for ch in item) for item in items[:2])


def normalize_field_name(name):
    if not isinstance(name, str):
        return ""
    return "".join(ch.lower() for ch in name if ch.isalnum())


def lookup_field(sample, names):
    """Find a value by allowing header variants like units/spacing/parentheses."""
    if sample is None:
        return None

    normalized_map = {normalize_field_name(k): v for k, v in sample.items()}
    for candidate in names:
        value = normalized_map.get(normalize_field_name(candidate))
        if value is not None:
            return value
    return None


def infer_header(values):
    if looks_like_timestamp(values[0]) and len(values) > 1 and is_int_like(values[1]):
        if len(values) <= len(FULL_HEADER):
            if len(values) >= len(HEADING_ONLY_HEADER):
                return FULL_HEADER[: len(values)]
        return None
    return None


def parse_sample(values):
    global header
    if header is None:
        return None
    if len(values) != len(header):
        if len(values) > len(header):
            values = values[: len(header)]
        else:
            return None
    return dict(zip(header, values))


def to_float(value):
    try:
        return float(value)
    except Exception:
        return None


def norm_angle(v):
    return v % 360.0


def angle_diff(target, current):
    return (target - current + 180.0) % 360.0 - 180.0


def pretty_print(sample):
    print("-" * 78)
    if sample is None:
        return
    for key, label in DISPLAY_FIELDS:
        value = lookup_field(sample, [key])
        if value is not None:
            if key in {"locationLatitude", "locationLongitude", "locationAltitude", "locationSpeed", "locationCourse", "locationTrueHeading", "locationMagneticHeading", "locationHeadingAccuracy"}:
                value = to_float(value)
                if value is None:
                    value = lookup_field(sample, [key])
            print(f"{label:20}: {value}")


def enable_raw_mode():
    if not sys.stdin.isatty():
        return None
    try:
        state = termios.tcgetattr(sys.stdin.fileno())
        tty.setcbreak(sys.stdin.fileno())
        return state
    except Exception:
        return None


def disable_raw_mode(state):
    if state is None:
        return
    try:
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, state)
    except Exception:
        pass


def read_key():
    if not sys.stdin.isatty():
        return None
    ready, _, _ = select.select([sys.stdin], [], [], 0.0)
    if not ready:
        return None
    return sys.stdin.read(1)


def capture_waypoint():
    global latest_gps, captured_waypoints
    if latest_gps is None:
        print("No valid GPS fix available to capture.")
        return
    lat, lon = latest_gps
    captured_waypoints.append({"lat": lat, "lon": lon})
    print(f"Captured waypoint #{len(captured_waypoints)}: lat={lat:.6f}, lon={lon:.6f}")


def dump_captured_waypoints_and_exit():
    print("\nCaptured GPS waypoints:")
    print(json.dumps({"waypoints": captured_waypoints}, indent=2))
    print(f"Total captured: {len(captured_waypoints)}")
    disable_raw_mode(term_state)
    sys.exit(0)


def haversine_m(lat1, lon1, lat2, lon2):
    radius = 6371000.0
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2.0) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dlambda / 2.0) ** 2
    return radius * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def bearing_to(lat1, lon1, lat2, lon2):
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dlon = math.radians(lon2 - lon1)
    x = math.sin(dlon) * math.cos(p2)
    y = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dlon)
    return norm_angle(math.degrees(math.atan2(x, y)))


def format_command(turn_angle):
    if abs(turn_angle) <= TURN_THRESHOLD:
        return "FORWARD"
    return ("LEFT" if turn_angle < 0 else "RIGHT", abs(turn_angle))


def load_waypoints(path):
    with open(path) as f:
        data = json.load(f)
    points = data.get("waypoints", [])
    parsed = []
    for p in points:
        if "lat" in p and "lon" in p:
            parsed.append({"lat": float(p["lat"]), "lon": float(p["lon"])})
    return parsed


def print_status(sample, heading, dist, bearing, wp_idx):
    print("\n--- Navigation ---")
    if dist is not None:
        print(f"Waypoint: {wp_idx}/{len(route)} | Dist: {dist:.1f} m")
    print(f"Heading: {heading:.1f}°")
    if bearing is not None:
        turn = angle_diff(bearing, heading)
        if abs(turn) <= TURN_THRESHOLD:
            print("Command: STRAIGHT")
        elif turn < 0:
            print(f"Command: LEFT {abs(turn):.0f}°")
        else:
            print(f"Command: RIGHT {turn:.0f}°")


route = []
route_path = None
if len(sys.argv) >= 2:
    route_path = sys.argv[1]
    try:
        route = load_waypoints(route_path)
        print(f"Loaded {len(route)} waypoints from {route_path}")
    except Exception:
        print("No valid route file supplied. Running in sensor-only print mode.")
        route = []

if not route:
    print("No route loaded. Showing sensor stream only.")

print("Controls: press 'w' to save current GPS point, 'q' to print captured waypoints and quit.")

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", SENSORLOG_PORT))
sock.settimeout(0.2)
term_state = enable_raw_mode()
atexit.register(disable_raw_mode, term_state)

print("Listening...")
print(f"SensorLog remote server: {SENSORLOG_IP}:{SENSORLOG_PORT}")
print(f"UDP listen: 0.0.0.0:{SENSORLOG_PORT}")

while True:
    key = read_key()
    if key == "w":
        capture_waypoint()
    elif key == "q":
        dump_captured_waypoints_and_exit()

    try:
        data, addr = sock.recvfrom(MAX_BYTES)
    except socket.timeout:
        continue

    text = data.decode("utf-8", errors="ignore").strip()
    if not text:
        continue

    for line in text.splitlines():
        if not line.strip():
            continue

        values = parse_csv(line)
        if not values:
            continue

        if header is None:
            detected = looks_like_header(values)
            if detected:
                header = values
                print("CSV header detected.")
                print(", ".join(header))
                continue

            inferred = infer_header(values)
            if inferred is not None:
                header = inferred
                print("No header received; inferred schema from stream length.")
            else:
                print(f"Waiting for header/known schema, got: {line}")
                continue

        sample = parse_sample(values)
        if sample is None:
            continue

        heading = to_float(lookup_field(sample, ["locationTrueHeading", "locationheading", "trueHeading"]))
        if heading is None or math.isnan(heading):
            heading = to_float(lookup_field(sample, ["locationCourse", "course"]))
        if heading is None:
            print("Waiting for valid heading.")
            pretty_print(sample)
            continue
        heading = norm_angle(heading)

        lat = to_float(lookup_field(sample, ["locationLatitude", "latitude", "lat"]))
        lon = to_float(lookup_field(sample, ["locationLongitude", "longitude", "lon"]))
        if lat is not None and lon is not None:
            latest_gps = (lat, lon)

        key = read_key()
        if key == "w":
            capture_waypoint()
        elif key == "q":
            dump_captured_waypoints_and_exit()

        if not route:
            pretty_print(sample)
            print(f"Current heading: {heading:.1f}°")
            continue

        if wp_index >= len(route):
            print("ARRIVED at final waypoint.")
            print("Navigation complete.")
            sys.exit(0)

        target = route[wp_index]
        if lat is None or lon is None:
            pretty_print(sample)
            print("Waiting for valid GPS fix...")
            continue

        dist = haversine_m(lat, lon, target["lat"], target["lon"])
        if dist <= WAYPOINT_RADIUS:
            wp_index += 1
            if wp_index >= len(route):
                print(f"Reached final waypoint {len(route)}/{len(route)}")
                print("ARRIVED at destination.")
                sys.exit(0)
            print(f"Reached waypoint {wp_index}/{len(route)}")
            if wp_index != last_print_wp:
                last_print_wp = wp_index
                continue

        target_bearing = bearing_to(lat, lon, route[wp_index]["lat"], route[wp_index]["lon"])
        pretty_print(sample)
        print_status(sample, heading, dist, target_bearing, wp_index + 1)
