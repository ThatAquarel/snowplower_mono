from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2


def main():
    connection = mavutil.mavlink_connection("udpin:127.0.0.1:14550")

    connection.wait_heartbeat()
    print(
        f"Heartbeat from system (system {connection.target_system} component {connection.target_component})"
    )

    while True:
        msg = connection.recv_match(type="GPS_RAW_INT", blocking=True)
        if not msg:
            continue
        if msg.get_type() == "BAD_DATA":
            if mavutil.all_printable(msg.data):
                print(msg.data)
            continue

        print(f"{msg.get_type()} {msg.time_usec} {msg.fix_type} {msg.lat} {msg.lon}")


if __name__ == "__main__":
    main()
