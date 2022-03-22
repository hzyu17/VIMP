//
// Created by hongzhe on 3/20/22.
//

#include "../include/Optimizer.h"
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>

#include <gpmp2/obstacle/ObstaclePlanarSDFFactorGPPointRobot.h>
#include <gpmp2/gp/GaussianProcessPriorLinear.h>

using namespace gtsam;
using namespace std;
using namespace GaussianSampler;
using namespace gpmp2;
using namespace Eigen;

inline gtsam::Vector errorWrapper(const ObstaclePlanarSDFFactorGPPointRobot& factor,
                                  const gtsam::Vector& conf1, const gtsam::Vector& vel1,
                                  const gtsam::Vector& conf2, const gtsam::Vector& vel2) {
    return factor.evaluateError(conf1, vel1, conf2, vel2);
}

// convert sdf vector to hinge loss err vector
inline gtsam::Vector convertSDFtoErr(const gtsam::Vector& sdf, double eps) {
    gtsam::Vector err_ori = 0.0 - sdf.array() + eps;
    return (err_ori.array() > 0.0).select(err_ori, gtsam::Vector::Zero(err_ori.rows()));  // (R < s ? P : Q)
}

/* ************************************************************************** */
// signed distance field data
gtsam::Matrix map_ground_truth = (gtsam::Matrix(7, 7) <<
                                        0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0,
        0,     0,     1,     1,     1,     0,     0,
        0,     0,     1,     1,     1,     0,     0,
        0,     0,     1,     1,     1,     0,     0,
        0,     0,     0,     0,     0,     0,     0,
        0,     0,     0,     0,     0,     0,     0).finished();
gtsam::Matrix field = (gtsam::Matrix(7, 7) <<
                             2.8284,    2.2361,    2.0000,    2.0000,    2.0000,    2.2361,    2.8284,
        2.2361,    1.4142,    1.0000,    1.0000,    1.0000,    1.4142,    2.2361,
        2.0000,    1.0000,   -1.0000,   -1.0000,   -1.0000,    1.0000,    2.0000,
        2.0000,    1.0000,   -1.0000,   -2.0000,   -1.0000,    1.0000,    2.0000,
        2.0000,    1.0000,   -1.0000,   -1.0000,   -1.0000,    1.0000,    2.0000,
        2.2361,    1.4142,    1.0000,    1.0000,    1.0000,    1.4142,    2.2361,
        2.8284,    2.2361,    2.0000,    2.0000,    2.0000,    2.2361,    2.8284).finished();
// bottom-left is (0,0), length is +/- 1 per point.
Point2 origin(0, 0);
double cell_size = 1.0;

PlanarSDF sdf = PlanarSDF(origin, cell_size, field);


// 2D point robot
const int ndof = 2, nlinks = 1, nspheres = 1, nsupptd_states = 2, N = 1;
const int ndim = 2 * ndof * nlinks * nsupptd_states;
PointRobot pR(ndof, nlinks);

double r = 1.5;
BodySphereVector body_spheres;
body_spheres.push_back(BodySphere(0, r, Point3(0.0, 0.0, 0.0)));

SharedNoiseModel Qc_model = noiseModel::Isotropic::Sigma(2, 1.0);

double delta_t = 0.1, tau = 0.025;
double obs_eps = 0.2, obs_sigma = 1.0;

PointRobotModel pRModel(pR, body_spheres);
ObstaclePlanarSDFFactorGPPointRobot factor(0, 1, 2, 3, pRModel, sdf, obs_sigma, obs_eps,
                                           Qc_model, delta_t, tau);

auto NoiseModel = factor.noiseModel();
//    Matrix modelInfomationMatrix = NoiseModel->R() * NoiseModel->R();
//    cout << modelInfomationMatrix << endl;

// just check cost of two link joint
gtsam::Matrix H1_act, H2_act, H3_act, H4_act;

// origin zero  and stationary case
VectorXd q1{gtsam::Vector2(0, 0)};
VectorXd q2{gtsam::Vector2(6, 4)};
VectorXd qdot1{gtsam::Vector2(0, 0)};
VectorXd qdot2{gtsam::Vector2(0, 0)};

gtsam::Vector err_act = factor.evaluateError(q1, qdot1, q2, qdot2, H1_act, H2_act, H3_act, H4_act);

