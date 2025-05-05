addpath('../../tools')
addpath('../../tools/gtsam_toolbox')
addpath ('../../tools/WAM/utils')

import gtsam.*
import gpmp2.*

arm = generateArm('WAMArm');
start_conf = [-0.8,   -1.70,   1.64,  1.29,   1.1, -0.106,    2.2];
h = plotWAMModel(arm, start_conf)