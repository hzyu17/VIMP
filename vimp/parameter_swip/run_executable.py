import subprocess
import argparse

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("dof", help="dof, choices: 2, 3, 7", type=int, default=2)
    parser.add_argument("map", help="map, choices: 1, 2", type=int, default=1)
    
    args = parser.parse_args()
    if args.dof == 2:
        if args.map == 1:
            subprocess.run("/home/hzyu/git/VIMP/vimp/build/PlanarPRModel", shell=True, check=True)
    