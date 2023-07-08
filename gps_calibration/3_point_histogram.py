import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


FILENAME = "points/point_0.csv"


def main():
    df = pd.read_csv(FILENAME)

    x = df["lat"].to_numpy().astype(np.float64)
    y = df["lon"].to_numpy().astype(np.float64)

    fig = plt.figure()
    ax = fig.add_subplot(projection="3d")
    hist, xedges, yedges = np.histogram2d(
        x, y, bins=4, range=[[x.min(), x.max()], [y.min(), y.max()]]
    )

    xpos, ypos = np.meshgrid(xedges[:-1] + 0.25, yedges[:-1] + 0.25, indexing="ij")
    xpos = xpos.ravel()
    ypos = ypos.ravel()
    zpos = 0

    # Construct arrays with the dimensions for the 16 bars.
    dx = dy = 0.5 * np.ones_like(zpos)
    dz = hist.ravel()

    ax.bar3d(xpos, ypos, zpos, dx, dy, dz, zsort="average")

    plt.show()


if __name__ == "__main__":
    main()
