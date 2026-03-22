"""Waypoint navigation using live SensorLog UDP CSV stream."""

import csv
import json
import math
import os
import socket
import sys
from collections import deque

# Default listener. In SensorLog Client mode, your phone streams to this address.
DEFAULT_LISTEN_IP = "0.0.0.0"
DEFAULT_LISTEN_PORT = 505
MAX_BYTES = 65535

WAYPOINT_RADIUS = 8.0
TURN_THRESHOLD = 12.0
GPS_SMOOTHING_WINDOW = 5

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

UNBIASED_MOTION_HEADER = [
    "motionTimestamp_sinceReboot",
    "motionYaw",
    "motionRoll",
    "motionPitch",
    "motionRotationRateX",
    "motionRotationRateY",
    "motionRotationRateZ",
    "motionUserAccelerationX",
    "motionUserAccelerationY",
    "motionUserAccelerationZ",
    "motionAttitudeReferenceFrame",
    "motionQuaternionX",
    "motionQuaternionY",
    "motionQuaternionZ",
    "motionQuaternionW",
    "motionGravityX",
    "motionGravityY",
    "motionGravityZ",
    "motionMagneticFieldX",
    "motionMagneticFieldY",
    "motionMagneticFieldZ",
    "motionHeading",
    "motionMagneticFieldCalibrationAccuracy",
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

FULL_HEADER_WITH_MOTION = FULL_HEADER + UNBIASED_MOTION_HEADER
HEADING_ONLY_WITH_MOTION = HEADING_ONLY_HEADER + UNBIASED_MOTION_HEADER

header = None
route = []
wp_index = 0
gps_window = deque(maxlen=GPS_SMOOTHING_WINDOW)


def parse_csv(line):
    try:
        return next(csv.reader([line]))
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


def canonical_field_name(name):
    if not isinstance(name, str):
        return ""
    # remove unit annotations and spaces, keep only the base field token characters
    base = name.split("(")[0].strip()
    return "".join(ch.lower() for ch in base if ch.isalnum())


def looks_like_header(values):
    if not values:
        return False
    if looks_like_timestamp(values[0]) and len(values) > 1 and is_int_like(values[1]):
        return False
    return any(any(ch.isalpha() for ch in item) for item in values[:2])


def infer_header(values):
    if len(values) >= len(HEADING_ONLY_HEADER) and looks_like_timestamp(values[0]) and is_int_like(values[1]):
        for candidate in (
            HEADING_ONLY_WITH_MOTION,
            HEADING_ONLY_HEADER,
            FULL_HEADER_WITH_MOTION,
            FULL_HEADER,
        ):
            if len(values) == len(candidate):
                return candidate
        return None
    return None


def parse_sample(values):
    global header
    if header is None:
        return None
    if len(values) > len(header):
        values = values[: len(header)]
    if len(values) != len(header):
        return None
    return dict(zip(header, values))


def normalize_field_name(name):
    if not isinstance(name, str):
        return ""
    return canonical_field_name(name)


def get_field(sample, candidates):
    if sample is None:
        return None
    normalized = {normalize_field_name(key): value for key, value in sample.items()}
    for candidate in candidates:
        value = normalized.get(normalize_field_name(candidate))
        if value is not None:
            return value
    return None


def to_float(value):
    try:
        return float(value)
    except Exception:
        return None


def haversine_m(lat1, lon1, lat2, lon2):
    radius = 6371000.0
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2.0) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dlambda / 2.0) ** 2
    return radius * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def bearing_deg(lat1, lon1, lat2, lon2):
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dlon = math.radians(lon2 - lon1)
    x = math.sin(dlon) * math.cos(p2)
    y = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dlon)
    return (math.degrees(math.atan2(x, y)) + 360) % 360


def angle_diff(target, current):
    return (target - current + 180.0) % 360.0 - 180.0


def norm_angle(value):
    return value % 360.0


def format_command(turn_angle):
    if abs(turn_angle) <= TURN_THRESHOLD:
        return "FORWARD", 0.0
    return ("LEFT", abs(turn_angle)) if turn_angle < 0 else ("RIGHT", abs(turn_angle))


def smooth_position(samples):
    if not samples:
        return None, None
    lat_sum = 0.0
    lon_sum = 0.0
    for lat_value, lon_value in samples:
        lat_sum += lat_value
        lon_sum += lon_value
    count = len(samples)
    return lat_sum / count, lon_sum / count


def load_waypoints(path):
    with open(path) as f:
        data = json.load(f)
    waypoints = data.get("waypoints", data if isinstance(data, list) else None)
    if not waypoints:
        raise ValueError("No waypoints in route file.")
    parsed = []
    for point in waypoints:
        parsed.append({"lat": float(point["lat"]), "lon": float(point["lon"])})
    return parsed


def clear_screen():
    os.system("clear" if os.name == "posix" else "cls")


def print_status(heading, target_bearing, dist, current_wp, total_wps):
    if target_bearing is None or dist is None:
        return
    turn = angle_diff(target_bearing, heading)
    action, amount = format_command(turn)
    clear_screen()
    print(f"Waypoint {current_wp}/{total_wps} | distance: {dist:.1f} m")
    if amount > 0:
        print(f"Command: {action} {amount:.0f}°")
    else:
        print("Command: FORWARD")
    print(f"Heading: {heading:.1f}° | Target: {target_bearing:.1f}°")


def print_usage():
    print("Usage: python robot_nav.py <route.json> [listen_port]")
    print("Default listen port: 505")


if len(sys.argv) < 2:
    print_usage()
    sys.exit(1)

route_path = sys.argv[1]
listen_port = int(sys.argv[2]) if len(sys.argv) >= 3 else DEFAULT_LISTEN_PORT

route = load_waypoints(route_path)

print(f"Loaded {len(route)} waypoints from {route_path}")
for i, wp in enumerate(route, start=1):
    print(f"  {i}: lat={wp['lat']:.6f}  lon={wp['lon']:.6f}")
print(f"Listening on UDP {DEFAULT_LISTEN_IP}:{listen_port}")
print("Waiting for SensorLog stream...")

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((DEFAULT_LISTEN_IP, listen_port))

while True:
    data, addr = sock.recvfrom(MAX_BYTES)
    text = data.decode("utf-8", errors="ignore").strip()
    if not text:
        continue

    for line in text.splitlines():
        if not line.strip():
            continue

        values = parse_csv(line.strip())
        if not values:
            continue

        if header is None:
            inferred = None
            if looks_like_header(values):
                header = [canonical_field_name(v) for v in values]
                print("CSV header detected.")
                print(", ".join(header))
            else:
                inferred = infer_header(values)
                if inferred is not None:
                    header = inferred
                    print(f"No header detected. Inferred schema with {len(header)} columns.")
                else:
                    print(f"Waiting for valid schema, got: {line}")
                    continue

        sample = parse_sample(values)
        if sample is None:
            continue

        lat = to_float(get_field(sample, ["locationLatitude", "latitude", "lat"]))
        lon = to_float(get_field(sample, ["locationLongitude", "longitude", "lon"]))
        if lat is None or lon is None:
            print("Waiting for valid GPS fix.")
            continue
        gps_window.append((lat, lon))
        sm_lat, sm_lon = smooth_position(gps_window)
        if sm_lat is None or sm_lon is None:
            continue

        heading = to_float(get_field(sample, ["locationTrueHeading", "locationtrueheading"]))
        if heading is None:
            heading = to_float(get_field(sample, ["motionHeading"]))
        if heading is None:
            heading = to_float(get_field(sample, ["locationCourse", "course"]))
        if heading is None:
            print("Waiting for valid heading.")
            continue
        heading = norm_angle(heading)

        if wp_index >= len(route):
            print("ARRIVED at destination.")
            sys.exit(0)

        target = route[wp_index]
        dist = haversine_m(sm_lat, sm_lon, target["lat"], target["lon"])
        if dist <= WAYPOINT_RADIUS:
            wp_index += 1
            if wp_index >= len(route):
                print("ARRIVED at destination.")
                sys.exit(0)
            print(f"Reached waypoint {wp_index}/{len(route)}")
            continue

        target_bearing = bearing_deg(sm_lat, sm_lon, target["lat"], target["lon"])
        print_status(heading, target_bearing, dist, wp_index + 1, len(route))
