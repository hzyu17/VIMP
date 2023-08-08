## run_gvimp.py
# Author: Hongzhe Yu
# Date: 08/07/2023
# Brief: run gvi motion planning algorithm for a given configuration file.

import os
pwd = os.getcwd()
built_dir = pwd + "/build"
exp = "map2"

if exp == "map1":
    script = built_dir + "/gvi_PointRobot"
    os.system(script)
elif exp == "map2":
    config_file = "configs/vimp/planar_pR_map2_new.xml"
    script = built_dir + "/gvi_PointRobot" + " " + config_file
    print("===== script =====")
    print()
    print(script)
    os.system(script)