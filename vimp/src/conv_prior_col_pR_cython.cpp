/**
 * @file test_conv_prior_col_pR_cython.cpp
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Wrapping the functional input in C++ for cython connections with python code.
 * @version 0.1
 * @date 2022-07-23
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <vimp/instances/PriorColPlanarPointRobot.h>
#include <vimp/instances/PlanarPointRobotSDFExample.h>

using namespace std;
using namespace gpmp2;
using namespace Eigen;
using namespace vimp;


int main(){
    // An example pr and sdf
    vimp::PlanarPointRobotSDFExample planar_pr_sdf;
    gpmp2::PointRobotModel pRModel = std::move(planar_pr_sdf.pRmodel());
    gpmp2::PlanarSDF sdf = std::move(planar_pr_sdf.sdf());

    /// parameters
    int n_total_states = 20, N = n_total_states - 1;
    const int ndof = planar_pr_sdf.ndof(), nlinks = planar_pr_sdf.nlinks();
    const int dim_conf = ndof * nlinks;
    const int dim_theta = 2 * dim_conf; // theta = [conf, vel_conf]
    /// dimension of the joint optimization problem
    const int ndim = dim_theta * n_total_states;

    /// Obs factor
    double cost_sigma = 1.0, epsilon = 1.0;

    /// start and goal
    double start_x = 1.0, start_y = 1.5, goal_x = 5.5, goal_y = 4.5;
    VectorXd start_theta(dim_theta);
    start_theta << start_x, start_y, 0, 0;
    VectorXd goal_theta(dim_theta);
    goal_theta << goal_x, goal_y, 0, 0;

    /// factored optimizers: containing 2 types: 
    /// 1. the single factor of priors for start and goal states;
    /// 2. the prior + collision factors for supported states.
    
    /// Noise model
    SharedNoiseModel K_0 = noiseModel::Isotropic::Sigma(dim_conf, 0.5);

    /// Vector of factored optimizers
    vector<std::shared_ptr<OptFactPriColGHInstancePointRobot>> vec_factor_opts;

    /// initial values
    VectorXd joint_init_theta{VectorXd::Zero(ndim)};

    for (int i = 0; i < n_total_states; i++) {
        VectorXd theta{start_theta + double(i) * (goal_theta - start_theta) / N};
        joint_init_theta.segment(i*dim_theta, dim_theta) = std::move(theta);

        /// Pk matrices
        MatrixXd Pk{MatrixXd::Zero(dim_conf, ndim)};
        Pk.block(0, i * dim_theta, dim_conf, dim_conf) = std::move(MatrixXd::Identity(dim_conf, dim_conf));
        
        /// prior
        UnaryFactorTranslation2D prior_k{gtsam::symbol('x', i), Vector2d{theta.segment(0, dim_conf)}, K_0};

        // collision
        gpmp2::ObstaclePlanarSDFFactorPointRobot collision_k{gtsam::symbol('x', i), pRModel, sdf, cost_sigma, epsilon};

        /// Factored optimizer
        std::shared_ptr<OptFactPriColGHInstancePointRobot> pOptimizer{new OptFactPriColGHInstancePointRobot{dim_conf, prior_k, collision_k, Pk}};
        vec_factor_opts.emplace_back(pOptimizer);

    }

    /// The joint optimizer
    VIMPOptimizerGH<OptFactPriColGHInstancePointRobot> optimizer{vec_factor_opts};

    /// Set initial value to the linear interpolation
    optimizer.set_mu(joint_init_theta);

    /// Update n iterations and data file names
    int num_iter = 5;
    optimizer.set_niterations(num_iter);
    optimizer.update_file_names("data/2d_pR/mean.csv", "data/2d_pR/cov.csv");

    optimizer.optimize();

    return 0;
}