import numpy as np
import pandas as pd

TRANSFORM = [
    [0.8688928761949993, -0.19869541627181156, 0.0],
    [-0.031323245401103574, 0.9704946541235367, 0.0],
    [36560569.15538452, 68609024.83707082, 1.0],
]

SOURCE = "full.waypoints"

SOURCE_COLS = {
    "INDEX": int,
    "CURRENT_WP": int,
    "COORD_FRAME": int,
    "COMMAND": int,
    "PARAM1": float,
    "PARAM2": float,
    "PARAM3": float,
    "PARAM4": float,
    "LATITUDE": float,
    "LONGITUDE": float,
    "ALTITUDE": float,
    "AUTOCONTINUE": int,
}


def main():
    df = pd.read_csv(SOURCE, sep="\t", names=list(SOURCE_COLS.keys()), skiprows=1)

    points = np.ones((len(df), 3), dtype=np.float64)
    points[:, 0:2] = np.transpose([df.LATITUDE, df.LONGITUDE]) * 10**7

    print(points)
    print()
    points = np.dot(points, TRANSFORM)
    print(points)

    df.LATITUDE, df.LONGITUDE = np.transpose(points[:, 0:2]) / 10**7

    out = "QGC WPL 110\n" + df.to_csv(
        sep="\t", header=False, index=False, lineterminator="\n"
    )
    with open("out_rev.waypoints", "w") as file:
        file.write(out)
        file.close()


if __name__ == "__main__":
    main()
