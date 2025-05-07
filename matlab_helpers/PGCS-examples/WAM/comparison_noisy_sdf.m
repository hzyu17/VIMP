% Comparison of performance on a map with noisy obstacles

close all
clear

addpath("/usr/local/gtsam_toolbox")

import gtsam.*
import gpmp2.*


% 2 configurations
start_confs = [-0.8,   -1.70,   1.64,  1.29,   1.1, -0.106,    2.2;
               -0.9,  -1.70,   1.34,  1.19,   0.8,  -0.126,    2.5];
end_confs = [-0.0,    0.94,     0,     1.6,     0,   -0.919,   1.55;
              -0.7,   1.35,    1.2,    1.0,   -0.7,  -0.1,   1.2];
          
