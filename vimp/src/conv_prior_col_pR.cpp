/**
 * @file test_conv_prior_col_pR.cpp
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Test the convergence of the algorithm with prior + collision cost only on supported states, 
 * for a planar robot.
 * @version 0.1
 * @date 2022-07-15
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "../instances/PriorColPlanarPointRobot.h"
// #include <gpmp2/obstacle/ObstaclePlanarSDFFactorPointRobot.h>
#include "../instances/PlanarPointRobotSDFExample.h"

using namespace std;
using namespace gpmp2;
using namespace Eigen;
using namespace vimp;

/**
 * @brief -log(p(x,z)) for prior and collision factors 
 * 
 * @param pose input pose
 * @param prior_factor the prior class
 * @param obstacle_factor the collision class
 * @return double 
 */
double errorWrapperPriorCol(const VectorXd& pose, 
                            const UnaryFactorTranslation2D& prior_factor,
                            const ObstaclePlanarSDFFactorPointRobot& collision_factor) {

    /**
     * Prior factor
     * */
    VectorXd vec_prior_err = prior_factor.evaluateError(pose);
    MatrixXd Qc = prior_factor.get_Qc();
    double prior_cost = vec_prior_err.transpose() * Qc.inverse() * vec_prior_err;

    /**
     * Obstacle factor
     * */
    VectorXd vec_err = collision_factor.evaluateError(pose);

    // MatrixXd precision_obs;
    MatrixXd precision_obs{MatrixXd::Identity(vec_err.rows(), vec_err.rows())};
    precision_obs = precision_obs / collision_factor.get_noiseModel()->sigmas()[0];

    double collision_cost = vec_err.transpose() * precision_obs * vec_err;

    // cout << "--- prior_cost ---" << endl << prior_cost << endl;
    // cout << "--- collision_cost ---" << endl << collision_cost << endl;

    return prior_cost + collision_cost;
}

int main(){
    // An example pr and sdf
    vimp::PlanarPointRobotSDFExample planar_pr_sdf;
    gpmp2::PointRobotModel pRModel = std::move(planar_pr_sdf.pRmodel());
    gpmp2::PlanarSDF sdf = std::move(planar_pr_sdf.sdf());

    /// parameters
    int n_total_states = 3, N = n_total_states - 1;
    const int ndof = planar_pr_sdf.ndof(), nlinks = planar_pr_sdf.nlinks();
    const int dim_conf = ndof * nlinks;
    const int dim_theta = 2 * dim_conf; // theta = [conf, vel_conf]
    /// dimension of the joint optimization problem
    const int ndim = dim_theta * n_total_states;

    /// Obs factor
    double cost_sigma = 1.0, epsilon = 1.5;

    /// start and goal
    double start_x = 1.0, start_y = 1.5, goal_x = 5.5, goal_y = 3.5;
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
    vector<std::shared_ptr<OptFactPriColPlanarPRGH>> vec_factor_opts;

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
        ObstaclePlanarSDFFactorPointRobot collision_k{gtsam::symbol('x', i), pRModel, sdf, cost_sigma, epsilon};

        /// Factored optimizer
        std::shared_ptr<OptFactPriColPlanarPRGH> pOptimizer{new OptFactPriColPlanarPRGH{dim_conf, errorWrapperPriorCol, prior_k, collision_k, Pk}};
        vec_factor_opts.emplace_back(pOptimizer);

    }

    /// The joint optimizer
    VIMPOptimizerGH<OptFactPriColPlanarPRGH> optimizer{vec_factor_opts};

    /// Set initial value to the linear interpolation
    int num_iter = 100;
    optimizer.set_mu(joint_init_theta);
    optimizer.set_niterations(num_iter);
    optimizer.set_step_size_base(0.75);
    optimizer.update_file_names("data/2d_pR/mean.csv", "data/2d_pR/cov.csv", "data/2d_pR/cost.csv");

    optimizer.optimize();

    return 0;
}