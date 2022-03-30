//
// Created by Hongzhe Yu on 3/20/22.
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

/**
 * The definition of error funciton h(x) using the cost class 'factor'.
 * The definition of any cost function should be
 * in the form: f(Args ..., cost class).
 * **/
inline double errorWrapper(const gtsam::Vector& theta, const ObstaclePlanarSDFFactorGPPointRobot& factor) {
    VectorXd conf1{theta.segment<2>(0)};
    VectorXd vel1{theta.segment<2>(2)};
    VectorXd conf2{theta.segment<2>(4)};
    VectorXd vel2{theta.segment<2>(6)};
    VectorXd vec_err = factor.evaluateError(conf1, vel1, conf2, vel2);
    MatrixXd precision_obs{MatrixXd::Identity(vec_err.rows(), vec_err.rows())};
    double sig_obs = 1.0;
    precision_obs = precision_obs / sig_obs;
    return vec_err.transpose() * precision_obs * vec_err;
}

// convert sdf vector to hinge loss err vector
inline gtsam::Vector convertSDFtoErr(const gtsam::Vector& sdf, double eps) {
    gtsam::Vector err_ori = 0.0 - sdf.array() + eps;
    return (err_ori.array() > 0.0).select(err_ori, gtsam::Vector::Zero(err_ori.rows()));  // (R < s ? P : Q)
}

void test_point_robot(){
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
    double total_time = 5.0;
    int num_support_states = 20, num_interp = 5;

    const int ndof = 2, nlinks = 1, nspheres = 1;
    const int dim_theta = 2 * ndof * nlinks;
    const int ndim = dim_theta * num_support_states;

    double delta_t = total_time / num_support_states, tau = delta_t / num_interp;
    double obs_eps = 0.2, obs_sigma = 1.0;

    PointRobot pR(ndof, nlinks);

    double r = 1.5;

    BodySphereVector body_spheres;
    body_spheres.push_back(BodySphere(0, r, Point3(0.0, 0.0, 0.0)));

    SharedNoiseModel Qc_model = noiseModel::Isotropic::Sigma(2, 1.0);

    PointRobotModel pRModel(pR, body_spheres);

    // just check cost of two link joint
    gtsam::Matrix H1_act, H2_act, H3_act, H4_act;

    // origin zero  and stationary case
    double start_x = 0.0, start_y = 0.0, goal_x = 5.5, goal_y = 4;
    VectorXd q1{gtsam::Vector2(start_x, start_y)};
    VectorXd q2{gtsam::Vector2(goal_x, goal_y)};
    VectorXd qdot1{gtsam::Vector2(0, 0)};
    VectorXd qdot2{gtsam::Vector2(0, 0)};

    vector<std::function<double(const gtsam::Vector&, const ObstaclePlanarSDFFactorGPPointRobot&)>> vec_cost_functions;
    vector<ObstaclePlanarSDFFactorGPPointRobot> vec_cost_classes;
    vector<gtsam::Matrix> vec_Pks;

    for (int i=0; i<num_support_states-1; i++){
        // cost classes
        vec_cost_classes.emplace_back(ObstaclePlanarSDFFactorGPPointRobot{symbol((unsigned char) 'x', i),
                                                                          symbol((unsigned char) 'v', i),
                                                                          symbol((unsigned char) 'x', i+1),
                                                                          symbol((unsigned char) 'v', i+1),
                                                                          pRModel,
                                                                          sdf,
                                                                          obs_sigma,
                                                                          obs_eps,
                                                                          Qc_model,
                                                                          delta_t,
                                                                          tau});
        // cost functions
        vec_cost_functions.emplace_back(errorWrapper);
        // Pk matrices
        MatrixXd Pk{MatrixXd::Zero(2*dim_theta, ndim)};
        Pk.block<dim_theta, dim_theta>(0, i*dim_theta) = MatrixXd::Identity(dim_theta, dim_theta);
        Pk.block<dim_theta, dim_theta>(dim_theta, (i+1)*dim_theta) = MatrixXd::Identity(dim_theta, dim_theta);
//        cout << "Pk " << endl << Pk << endl;
        vec_Pks.emplace_back(Pk);
    }

    // declare the optimizer
    // template <typename Function, typename costClass, typename... Args>
    VariationalInferenceMPOptimizer<std::function<double(const gtsam::Vector&, const ObstaclePlanarSDFFactorGPPointRobot&)>,
            ObstaclePlanarSDFFactorGPPointRobot, gtsam::Vector> optimizer(ndim, dim_theta, vec_cost_functions, vec_cost_classes, vec_Pks);

    const int num_iter = 10;
    double step_size = 0.9;

    for (int i=0; i<num_iter; i++) {
        step_size = step_size / pow((i + 1), 1 / 3);
        optimizer.set_step_size(step_size, step_size);

        cout << "==== iteration " << i << " ====" << endl
             << "mean " << endl << optimizer.get_mean().format(CleanFmt) << endl;
//             << "precision matrix " << endl << optimizer.get_precision().format(CleanFmt) << endl;

        // get the derivatives from samples
        optimizer.step();

        // test for a known Gaussian posterior
//        optimizer.step_closed_form();
    }
}

int main(){
    test_point_robot();
    return 0;
}

