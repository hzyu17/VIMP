import numpy as np
from gtsam import *
from gpmp2 import *
from vimp import *
import matplotlib.pyplot as plt

from gpmp2_python.datasets.generate2Ddataset import generate2Ddataset
from gpmp2_python.robots.generateArm import generateArm
from gpmp2_python.utils.plot_utils import *
from gpmp2_python.utils.signedDistanceField2D import signedDistanceField2D


# gtsam::GP
Qc = np.identity(2)
Qc_model = noiseModel_Gaussian.Covariance(Qc)

a = np.asarray([0.5, 0.5])
d = np.asarray([0, 0])
alpha = np.asarray([0, 0])

#gpmp2::Arm
arm = Arm(2, a, alpha, d)

# point robot
pR = PointRobot(2, 1)
spheres_data = np.asarray([0.0, 0.0, 0.0, 0.0, 1.5])
nr_body = spheres_data.shape[0]
sphere_vec = BodySphereVector()
sphere_vec.push_back(
    BodySphere(spheres_data[0], spheres_data[4], Point3(spheres_data[1:4]))
)
pR_model = PointRobotModel(pR, sphere_vec)

# sdf
dataset = generate2Ddataset("MultiObstacleDataset")
rows = dataset.rows
cols = dataset.cols
cell_size = dataset.cell_size
origin_point2 = Point2(dataset.origin_x, dataset.origin_y)

# Signed Distance field
field = signedDistanceField2D(dataset.map, cell_size)
sdf = PlanarSDF(origin_point2, cell_size, field)

# Obstacle avoid settings
cost_sigma = 0.5
epsilon_dist = 4.0
obs = ObstaclePlanarSDFFactorPointRobot(ord('x'), pR_model, sdf, cost_sigma, epsilon_dist)
x = np.asarray([0.5, 0.5])
print "evaluateErr", obs.evaluateError(x)

# ------------------------------ namespace vimp
# UnaryFactorTranslation2D
factor = UnaryFactorTranslation2D(ord('x'), a, Qc_model)
Qc_get = factor.get_Qc()
print "Qc_get"
print Qc_get

# CythonTest
cython_test = CythonTest(a, factor)
print "cython test", cython_test.vec() 
x = np.asarray([[0.5, 0], [0, 0.5]])
print "cython test f function", cython_test.f(x)

# CythonTest2
cytest_2 = CyTest2(pR_model) 

