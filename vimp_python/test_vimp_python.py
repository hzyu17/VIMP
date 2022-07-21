import numpy as np
from gtsam import *
from gpmp2 import *
from vimp import *
import matplotlib.pyplot as plt

# GP
Qc = np.identity(2)
Qc_model = noiseModel_Gaussian.Covariance(Qc)

a = np.asarray([0.5, 0.5])
d = np.asarray([0, 0])
alpha = np.asarray([0, 0])
arm = Arm(2, a, alpha, d)