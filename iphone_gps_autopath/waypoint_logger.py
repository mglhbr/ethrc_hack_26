"""Interactive waypoint logger for SensorLog UDP stream.

Run this in its own terminal and press:
- `w`: capture current GPS coordinate
- `q`: print all captured waypoints and exit

Usage:
  python waypoint_logger.py [listen_port] [output_json_path]
"""

import atexit
import csv
import json
import math
import select
import socket
import sys
import termios
import tty
import threading


SENSORLOG_PORT = 505
MAX_BYTES = 65535

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

FULL_HEADER_WITH_MOTION = FULL_HEADER + UNBIASED_MOTION_HEADER
HEADING_ONLY_WITH_MOTION = HEADING_ONLY_HEADER + UNBIASED_MOTION_HEADER


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


def looks_like_header(values):
    if not values:
        return False
    if looks_like_timestamp(values[0]) and len(values) > 1 and is_int_like(values[1]):
        return False
    return any(any(ch.isalpha() for ch in item) for item in values[:2])


def infer_header(values):
    if looks_like_timestamp(values[0]) and len(values) > 1 and is_int_like(values[1]):
        for candidate in (
            HEADING_ONLY_WITH_MOTION,
            HEADING_ONLY_HEADER,
            FULL_HEADER_WITH_MOTION,
            FULL_HEADER,
        ):
            if len(values) == len(candidate):
                return candidate
    return None


def normalize_field_name(name):
    if not isinstance(name, str):
        return ""
    return "".join(ch.lower() for ch in name if ch.isalnum())


def get_field(sample, candidates):
    if sample is None:
        return None
    normalized = {normalize_field_name(k): v for k, v in sample.items()}
    for candidate in candidates:
        candidate_norm = normalize_field_name(candidate)
        direct = normalized.get(candidate_norm)
        if direct is not None:
            return direct
        if not candidate_norm or len(candidate_norm) < 6:
            continue
        for key, value in normalized.items():
            if key.startswith(candidate_norm) or candidate_norm.startswith(key):
                return value
    return None


def to_float(value):
    try:
        return float(value)
    except Exception:
        return None


def print_status(lat, lon, heading, captured_count):
    print(f"Current: lat={lat:.6f}, lon={lon:.6f}, heading={heading:.1f}° | captured={captured_count}", end="\r")


def extract_gps(sample):
    lat = to_float(get_field(sample, ["locationLatitude", "latitude", "lat"]))
    lon = to_float(get_field(sample, ["locationLongitude", "longitude", "lon"]))
    if lat is None or lon is None:
        return None, None
    return lat, lon


def extract_heading(sample):
    heading = to_float(get_field(sample, ["locationTrueHeading", "locationheading", "trueHeading"]))
    if heading is None:
        heading = to_float(get_field(sample, ["motionHeading"]))
    if heading is None:
        heading = to_float(get_field(sample, ["locationCourse", "course"]))
    return heading


def run_logger(listen_port, output_path=None):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", listen_port))
    sock.settimeout(0.2)

    header = None
    latest_gps = None
    captured = []
    stop_event = threading.Event()
    terminal_state = [None]

    def disable_raw_mode():
        if terminal_state[0] is not None:
            try:
                termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, terminal_state[0])
            except Exception:
                pass

    def restore_exit():
        disable_raw_mode()

    atexit.register(restore_exit)

    def key_loop():
        if not sys.stdin.isatty():
            print("Waypoint capture needs interactive terminal input (tty).")
            return
        try:
            terminal_state[0] = termios.tcgetattr(sys.stdin.fileno())
            tty.setcbreak(sys.stdin.fileno())
            while not stop_event.is_set():
                r, _, _ = select.select([sys.stdin], [], [], 0.1)
                if not r:
                    continue
                ch = sys.stdin.read(1).lower()
                if ch == "w":
                    if latest_gps is None:
                        print("\nNo valid GPS fix yet.")
                        continue
                    lat, lon = latest_gps
                    captured.append({"lat": lat, "lon": lon})
                    print(f"\nCaptured waypoint #{len(captured)}: {lat:.6f}, {lon:.6f}")
                elif ch == "q":
                    stop_event.set()
                else:
                    if ch.strip():
                        print(f"\nUnknown key '{ch}'. Use w=save, q=quit.")
        except Exception:
            print("Key listener disabled. Falling back to Ctrl+C exit.")

    t = threading.Thread(target=key_loop, daemon=True)
    t.start()

    print(f"Listening on UDP 0.0.0.0:{listen_port}")
    print("Press 'w' to save a waypoint, 'q' to print and quit.")

    while not stop_event.is_set():
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
                if looks_like_header(values):
                    header = values
                    print("CSV header detected.")
                    print(", ".join(header))
                    continue
                inferred = infer_header(values)
                if inferred is None:
                    print(f"Waiting for valid schema, got: {line}")
                    continue
                header = inferred
                print("Inferred schema from stream length.")

            if len(values) != len(header):
                continue
            sample = dict(zip(header, values))

            lat, lon = extract_gps(sample)
            if lat is None or lon is None:
                continue
            latest_gps = (lat, lon)

            heading = extract_heading(sample)
            if heading is None:
                heading = float("nan")

            if not math.isnan(heading):
                print_status(lat, lon, heading, len(captured))
            else:
                print(f"Current: lat={lat:.6f}, lon={lon:.6f} (heading missing) | captured={len(captured)}", end="\r")

    disable_raw_mode()
    print("\n--- Captured waypoints ---")
    payload = {"waypoints": captured}
    print(json.dumps(payload, indent=2))
    print(f"Total captured: {len(captured)}")
    if output_path:
        with open(output_path, "w") as f:
            json.dump(payload, f, indent=2)
        print(f"Saved to {output_path}")


def main():
    listen_port = int(sys.argv[1]) if len(sys.argv) >= 2 else SENSORLOG_PORT
    output_path = sys.argv[2] if len(sys.argv) >= 3 else None
    run_logger(listen_port, output_path)


if __name__ == "__main__":
    main()
