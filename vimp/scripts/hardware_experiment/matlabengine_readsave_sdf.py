# Test python code to run matlab script
# Install matlab engine in python:

# cd /usr/local/MATLAB/R2020b/extern/engines/python
# python setup.py \
#         build  --build-base=/tmp/engine_build \
#         install --user


import matlab.engine
import os
# vimp root directory
this_dir = os.path.dirname(os.path.abspath(__file__))
    

def read_and_save_sdf(in_map_file, out_bin_file):
    # ------------------------------------------------------------- 
    #    Matlab function directories and gtsam toolbox directory
    # -------------------------------------------------------------
    # eng = matlab.engine.start_matlab("-nojvm -nodisplay")  # No displays, no figures showing up
    eng = matlab.engine.start_matlab("-nodesktop")  # Show the figures
    eng.eval("disp(version)", nargout=0)

    print("MATLAB version:", eng.version())

    matlab_func_dir = str(this_dir + "/../../maps/WAM")
    gtsam_toolbox_dir = str(this_dir + "/../../../matlab_helpers/tools/gtsam_toolbox")
    eng.addpath(matlab_func_dir)
    eng.addpath(gtsam_toolbox_dir, nargout=0)

    # -------------------------------- 
    #   Call the saving SDF function
    # --------------------------------
    eng.SaveSDFFromOccMapFile(in_map_file, out_bin_file, nargout=0)

    # input("MATLAB figure is open.  Press <Enter> when you’re done…")

    eng.quit()


if __name__ == '__main__':
    # Data directories
    map_file = str(this_dir + "/occupancy_map.mat")
    out_bin = str(this_dir + "/sdf.bin")
    
    read_and_save_sdf(map_file, out_bin)

