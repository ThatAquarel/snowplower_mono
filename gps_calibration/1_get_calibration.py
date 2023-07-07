import scipy
import numpy as np
import pandas as pd

FILENAMES = [
    "points/point_0.csv",
    "points/point_1.csv",
    "points/point_2.csv",
]

SOURCE = "points/point_n.waypoints"

SOURCE_COLS = [
    "INDEX",
    "CURRENT_WP",
    "COORD_FRAME",
    "COMMAND",
    "PARAM1",
    "PARAM2",
    "PARAM3",
    "PARAM4",
    "LATITUDE",
    "LONGITUDE",
    "ALTITUDE",
    "AUTOCONTINUE",
]


def main():
    destination, source = np.ones((2, len(FILENAMES), 3), dtype=np.float64)

    for i, filename in enumerate(FILENAMES):
        df = pd.read_csv(filename)

        lat = df["lat"].to_numpy().astype(np.float64)
        lon = df["lon"].to_numpy().astype(np.float64)

        destination[i, :] = [np.mean(lat), np.mean(lon), 1]
        print(f"{filename} {destination[i, 0:2].tolist()}")
    print(destination)
    print()

    df = pd.read_csv(SOURCE, sep="\t", names=SOURCE_COLS)
    df = df.drop(np.arange(2))
    df.LATITUDE *= 10**7
    df.LONGITUDE *= 10**7

    source[:, 0:2] = np.transpose([df.LATITUDE, df.LONGITUDE])
    source_points = source[:, 0:2]
    destination_points = destination[:, 0:2]

    print(source_points.tolist())
    print(destination_points.tolist())
    print()

    dist_map = scipy.spatial.distance.cdist(source_points, destination_points)
    index = np.argmin(dist_map, axis=1)
    print(dist_map)
    print(index)
    print()

    destination = destination[index]

    print(source)
    print(destination)
    print()

    matrix = np.linalg.solve(source, destination)
    matrix_inv = np.linalg.inv(matrix)
    print(matrix.tolist())
    print(matrix_inv.tolist())


if __name__ == "__main__":
    main()
