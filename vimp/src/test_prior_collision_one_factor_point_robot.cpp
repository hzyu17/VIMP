/**
 * @file test_prior_collision_one_factor_point_robot.cpp
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief 
 * @version 0.1
 * @date 2022-05-11
 * 
 * @copyright Copyright (c) 2022
 * 
 */

/// Description: Use a known top map to test the prior and collision factor on the support states, without interpolations.
#include <gtsam/inference/Symbol.h>
// #include <vimp/helpers/matplotlibcpp.h>
#include <vimp/optimizer/OptimizerFactorizedPriorCollisionPointRobot.h>
#include <vimp/optimizer/OptimizerFactorizedPriorGH.h>
#include <vimp/optimizer/OptimizerPlanarPointRobotGH.h>

using namespace gtsam;
using namespace std;
using namespace gpmp2;
using namespace Eigen;
// namespace plt = matplotlibcpp;
using namespace vimp;


/**
* Plot the iteration results
* @param results: data in shape (num_iteration, num_states)
* @param N : number of states
* @param dim_theta dimension of the configuration
**/
// void plot_result(const MatrixXd &results, int N, int dim_theta) {
    // plotting in the 2D case
    // plt::figure();
    // MatrixXd x(results.rows(), N), y(results.rows(), N);
    // vector<double> vec_x, vec_y;
    // vec_x.resize(x.cols());
    // vec_y.resize(x.cols());

    // for (int i = 0; i < N; i++) {
    //     x.col(i) = results.col(i * dim_theta);
    //     y.col(i) = results.col(i * dim_theta + 1);
    // }
    // for (int k = x.rows() - 1; k < x.rows(); k++) {
    //     VectorXd::Map(&vec_x[0], x.cols()) = VectorXd{x.row(k)};
    //     VectorXd::Map(&vec_y[0], y.cols()) = VectorXd{y.row(k)};
    //     plt::plot(vec_x, vec_y, "--");
    // }


//     cout << "x" << endl << x << endl;
//     cout << "y" << endl << y << endl;

//     plt::grid(true);
//     plt::show();
// }

/// Main testing function for a planar point robot with known SDF, start and goal states.
void test_point_robot() {
/// signed distance field data
    gtsam::Matrix map_ground_truth = (gtsam::Matrix(7, 7) <<
                                                          0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0,
            0, 0, 1, 1, 1, 0, 0,
            0, 0, 1, 1, 1, 0, 0,
            0, 0, 1, 1, 1, 0, 0,
            0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0).finished();
    gtsam::Matrix field = (gtsam::Matrix(7, 7) <<
            2.8284, 2.2361, 2.0000, 2.0000, 2.0000, 2.2361, 2.8284,
            2.2361, 1.4142, 1.0000, 1.0000, 1.0000, 1.4142, 2.2361,
            2.0000, 1.0000, -1.0000, -1.0000, -1.0000, 1.0000, 2.0000,
            2.0000, 1.0000, -1.0000, -2.0000, -1.0000, 1.0000, 2.0000,
            2.0000, 1.0000, -1.0000, -1.0000, -1.0000, 1.0000, 2.0000,
            2.2361, 1.4142, 1.0000, 1.0000, 1.0000, 1.4142, 2.2361,
            2.8284, 2.2361, 2.0000, 2.0000, 2.0000, 2.2361, 2.8284).finished();
    // layout of SDF: Bottom-left is (0,0), length is +/- 1 per point.
    Point2 origin(0, 0);
    double cell_size = 1.0;

    PlanarSDF sdf = PlanarSDF(origin, cell_size, field);

    /// 2D point robot
    // double total_time = 10.0;
    int num_support_states = 0, N = num_support_states + 1;
    // int num_interp = 2;

    /// parameters
    const int ndof = 2, nlinks = 1;
    const int dim_conf = ndof * nlinks;
    const int dim_theta = 2 * dim_conf;
    const int ndim = dim_theta * N;

    // double delta_t = total_time / num_support_states;
    // double obs_eps = 0.2, obs_sigma = 1.0;

    /// Robot model
    PointRobot pR(ndof, nlinks);
    double r = 0.5;
    BodySphereVector body_spheres;
    body_spheres.push_back(BodySphere(0, r, Point3(0.0, 0.0, 0.0)));
    PointRobotModel pRModel(pR, body_spheres);

    // initialize the mean by linear interpolation
    double start_x = 0.0, start_y = 0.0, goal_x = 5.5, goal_y = 0.0;
    VectorXd start_conf(dim_theta);
    start_conf << start_x, start_y, 0, 0;
    VectorXd goal_conf(dim_theta);
    goal_conf << goal_x, goal_y, 0, 0;


    /// factored optimizers: containing 2 types: 
    /// 1. the single factor of priors for start and goal states;
    /// 2. the prior + collision factors for supported states.
    
    /// Noise models
    SharedNoiseModel Qc_model = noiseModel::Isotropic::Sigma(dim_conf, 1.0);
    cout << "Qc" << endl << getQc(Qc_model) << endl;
    SharedNoiseModel K_0 = noiseModel::Isotropic::Sigma(dim_conf, 1.0);

    for (int i = 0; i < 1; i++) {
        // Pk matrices
        MatrixXd Pk{MatrixXd::Zero(2 * dim_theta, ndim)};
        Pk.block<dim_theta, dim_theta>(0, i * dim_theta) = MatrixXd::Identity(dim_theta, dim_theta);
        Pk.block<dim_theta, dim_theta>(dim_theta, (i + 1) * dim_theta) = MatrixXd::Identity(dim_theta, dim_theta);

        cout << "Pk" << endl << Pk << endl;

        // Pk matrices for conf
        MatrixXd Pk_conf1{MatrixXd::Zero(dim_conf, ndim)};
        Pk_conf1.block<dim_conf, dim_conf>(0, i * dim_theta) = MatrixXd::Identity(dim_conf, dim_conf);

        cout << "Pk_conf1" << endl << Pk_conf1 << endl;

        // Pk matrices for vel
        MatrixXd Pk_vel1{MatrixXd::Zero(dim_conf, ndim)};
        Pk_vel1.block<dim_conf, dim_conf>(0, i * dim_theta + dim_conf) = MatrixXd::Identity(dim_conf, dim_conf);

        cout << "Pk_vel1" << endl << Pk_vel1 << endl;

    //     if (i == 0) {
    //         VectorXd conf{start_conf + double(i) * (goal_conf - start_conf) / num_support_states};
    //         // prior at start and end pose
    //         vec_one_prior.emplace_back(
    //                 OptimizerFactorPriorColPointRobotGH{1 * dim_conf,
    //                                   errorWrapperSinglePrior,
    //                                   UnaryFactorTranslation2D{symbol( 'x', i), gtsam::Vector2{conf.segment<dim_conf>(0)}, K_0}, Pk_conf1});

    //         vec_one_prior.emplace_back(
    //                 OptimizerFactorPriorColPointRobotGH{1 * dim_conf,
    //                                   errorWrapperSinglePrior,
    //                                   UnaryFactorTranslation2D{symbol( 'v', i), gtsam::Vector2{conf.segment<dim_conf>(dim_conf)}, K_0}, Pk_vel1});
    //    }

    // N: goal conf
    // MatrixXd Pk_conf1{MatrixXd::Zero(dim_conf, ndim)};
    // Pk_conf1.block<dim_conf, dim_conf>(0, (N-1) * dim_theta) = MatrixXd::Identity(dim_conf, dim_conf);
    // vec_one_prior.emplace_back(
    //         OptimizerFactorizedPriorGH{1 * dim_conf, errorWrapperSinglePrior, UnaryFactorTranslation2D{symbol('x', N-1),
    //                                                                                           gtsam::Vector2{goal_conf.segment<dim_conf>(0)},
    //                                                                                           K_0},
    //                           Pk_conf1});
    // goal vel
    // MatrixXd Pk_vel1{MatrixXd::Zero(dim_conf, ndim)};
    // Pk_vel1.block<dim_conf, dim_conf>(0, (N-1) * dim_theta + dim_conf) = MatrixXd::Identity(dim_conf, dim_conf);
    // vec_one_prior.emplace_back(
    //         OptimizerFactorizedPriorGH{1 * dim_conf, 
    //                                     errorWrapperSinglePrior, 
    //                                     UnaryFactorTranslation2D{symbol('v', N-1), gtsam::Vector2{goal_conf.segment<dim_conf>(dim_conf)}, K_0},
    //                                     Pk_vel1});

    
    // optimizer.set_mu(init_mean);

    // const int num_iter = 80;
    // double step_size = 0.9;

    // MatrixXd results{MatrixXd::Zero(num_iter, ndim)};

    // for (int i = 0; i < num_iter; i++) {
    //     step_size = step_size / pow((i + 1), 1 / 3);
    //     optimizer.set_step_size(step_size, step_size);
    //     VectorXd mean_iter{optimizer.get_mean()};
    //     cout << "==== iteration " << i << " ====" << endl;
    //     for (int j = 0; j < N; j++) {
    //         cout << "position mean" << endl << mean_iter.segment<2>(j * dim_theta) << endl;
    //     }
    //     results.row(i) = optimizer.get_mean().transpose();
    //     optimizer.step();
    // }

    // plot_result(results, N, dim_theta);
}

}



int main(){
    test_point_robot();
    return 0;
}

