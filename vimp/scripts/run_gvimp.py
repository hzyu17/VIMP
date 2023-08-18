## run_gvimp.py
# Author: Hongzhe Yu
# Date: 08/07/2023
# Brief: run gvi motion planning algorithm for a given configuration file.

import os
full_path = os.path.realpath(__file__)
path, filename = os.path.split(full_path)
vimp_root = os.path.dirname(os.path.dirname(full_path))

built_dir = vimp_root + "/build/src/gvimp"
exp = "pr_map2"

if exp == "pr_map1":
    config_file = "configs/vimp/planar_pR_map1_new.xml"
    script = built_dir + "/gvi_PointRobot" + " " + config_file
    os.system(script)
elif exp == "pr_map2":
    config_file = "configs/vimp/planar_pR_map2_new.xml"
    script = built_dir + "/gvi_PointRobot" + " " + config_file
    print("===== script =====")
    print()
    print(script)
    os.system(script)
elif exp == "pr_map3":
    config_file = "configs/vimp/planar_pR_map3_new.xml"
    script = built_dir + "/gvi_PointRobot" + " " + config_file
    print("===== script =====")
    print()
    print(script)
    os.system(script)