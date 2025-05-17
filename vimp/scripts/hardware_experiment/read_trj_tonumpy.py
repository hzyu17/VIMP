import csv
from pathlib import Path
import numpy as np


if __name__ == '__main__':
    csv_file = Path(__file__).parent / "Data" / "Trajectories" / "zk_sdf.csv"
    trj_np = np.loadtxt(csv_file, delimiter=',').transpose()
    print(trj_np.shape)
