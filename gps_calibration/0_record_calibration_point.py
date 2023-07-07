import pandas as pd

from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2


FIELDS = {
    "time_usec": "uint64",
    "fix_type": "uint8",
    "lat": "int32",
    "lon": "int32",
    "alt": "int32",
    "eph": "uint16",
    "epv": "uint16",
    "vel": "uint16",
    "cog": "uint16",
    "satellites_visible": "uint8",
}


def main():
    data = {f: [] for f in FIELDS}

    connection = mavutil.mavlink_connection("udpin:127.0.0.1:14551")
    connection.wait_heartbeat()
    print(
        f"Heartbeat from system (system {connection.target_system} component {connection.target_component})"
    )

    i = 0
    while i < 128:
        msg = connection.recv_match(type="GPS_RAW_INT", blocking=True)
        if not msg:
            continue
        if msg.get_type() != "GPS_RAW_INT":
            continue

        print(
            f"{i} {msg.get_type()} {msg.time_usec} {msg.fix_type} {msg.lat} {msg.lon}"
        )

        for f in FIELDS:
            data[f].append(getattr(msg, f))
        i += 1

    df = pd.DataFrame(data, columns=FIELDS.keys())
    df.to_csv("raw_gps.csv")


if __name__ == "__main__":
    main()

# .\mavp2p.exe serial:COM3:57600 udpc:127.0.0.1:14550 udpc:127.0.0.1:14551
