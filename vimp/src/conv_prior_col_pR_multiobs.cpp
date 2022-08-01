/**
 * @file conv_prior_col_pR_multiobs.cpp
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Test of algorithm in multi obstacle environment.
 * @version 0.1
 * @date 2022-07-27
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "../instances/PriorVelColPlanarPointRobot.h"
#include "../instances/PlanarPointRobotSDFMultiObsExample.h"

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
                            const UnaryFactorTranslation2D& prior_pos,
                            const UnaryFactorTranslation2D& prior_vel,
                            const ObstaclePlanarSDFFactorPointRobot& collision_factor) {
    
    Vector2d position = pose.segment(0, 2);
    Vector2d velocity = pose.segment(2, 2);

    /**
     * Prior factor
     * */
    VectorXd vec_prior_err = prior_pos.evaluateError(position);
    MatrixXd Qc = prior_pos.get_Qc();
    double prior_cost = vec_prior_err.transpose() * Qc.inverse() * vec_prior_err;

    /**
     * Prior factor for velocity
     * */
    VectorXd vec_vel_err = prior_vel.evaluateError(velocity);
    MatrixXd Qc_v = prior_vel.get_Qc();
    double prior_cost_vel = vec_vel_err.transpose() * Qc_v.inverse() * vec_vel_err;

    /**
     * Obstacle factor
     * */
    VectorXd vec_err = collision_factor.evaluateError(position);

    // MatrixXd precision_obs;
    MatrixXd precision_obs{MatrixXd::Identity(vec_err.rows(), vec_err.rows())};
    precision_obs = precision_obs / collision_factor.get_noiseModel()->sigmas()[0];

    double collision_cost = vec_err.transpose() * precision_obs * vec_err;

    return prior_cost + prior_cost_vel + collision_cost;
}


int main(){
    // An example pr and sdf
    vimp::PlanarPointRobotSDFMultiObsExample planar_pr_sdf;
    gpmp2::PointRobotModel pRModel = std::move(planar_pr_sdf.pRmodel());
    gpmp2::PlanarSDF sdf = std::move(planar_pr_sdf.sdf());

    /// parameters
    int n_total_states = 10, N = n_total_states - 1;
    const int ndof = planar_pr_sdf.ndof(), nlinks = planar_pr_sdf.nlinks();
    const int dim_conf = ndof * nlinks;
    const int dim_theta = 2 * dim_conf; // theta = [conf, vel_conf]
    /// dimension of the joint optimization problem
    const int ndim = dim_theta * n_total_states;

    /// Obs factor
    double cost_sigma = 0.5, epsilon = 4.0;

    /// start and goal
    double start_x = 0, start_y = 0, goal_x = 17, goal_y = 14;
    VectorXd start_theta(dim_theta);
    start_theta << start_x, start_y, 0, 0;
    VectorXd goal_theta(dim_theta);
    goal_theta << goal_x, goal_y, 0, 0;

    /// factored optimizers: containing 2 types: 
    /// 1. the single factor of priors for start and goal states;
    /// 2. the prior + collision factors for supported states.
    
    /// Noise model
    SharedNoiseModel K_0 = noiseModel::Isotropic::Sigma(dim_conf, 1.0);

    /// Vector of factored optimizers
    vector<std::shared_ptr<OptFactPriVelColPlanarPRGH>> vec_factor_opts;

    /// initial values
    VectorXd joint_init_theta{VectorXd::Zero(ndim)};

    for (int i = 0; i < n_total_states; i++) {
        VectorXd theta{start_theta + double(i) * (goal_theta - start_theta) / N};
        // initial vel: avg_vel
        if (i>0 && i<n_total_states - 1){
            theta.segment(2, 2) = VectorXd::Ones(2).cwiseProduct((goal_theta.segment(0, 2) - start_theta.segment(0, 2)) / n_total_states);
        }
        joint_init_theta.segment(i*dim_theta, dim_theta) = std::move(theta);

        /// Pk matrices
        MatrixXd Pk{MatrixXd::Zero(dim_theta, ndim)};
        Pk.block(0, i * dim_theta, dim_theta, dim_theta) = std::move(MatrixXd::Identity(dim_theta, dim_theta));
        
        /// prior
        UnaryFactorTranslation2D prior_k{gtsam::symbol('x', i), Vector2d{theta.segment(0, dim_conf)}, K_0};
        UnaryFactorTranslation2D prior_vk{gtsam::symbol('v', i), Vector2d{theta.segment(2, dim_conf)}, K_0};

        // collision
        ObstaclePlanarSDFFactorPointRobot collision_k{gtsam::symbol('x', i), pRModel, sdf, cost_sigma, epsilon};

        /// Factored optimizer
        std::shared_ptr<OptFactPriVelColPlanarPRGH> pOptimizer{new OptFactPriVelColPlanarPRGH{dim_theta, errorWrapperPriorCol, prior_k, prior_vk, collision_k, Pk}};
        vec_factor_opts.emplace_back(pOptimizer);

    }

    /// The joint optimizer
    VIMPOptimizerGH<OptFactPriVelColPlanarPRGH> optimizer{vec_factor_opts};

    /// Set initial value to the linear interpolation
    int num_iter = 30;
    optimizer.set_mu(joint_init_theta);
    optimizer.set_niterations(num_iter);
    optimizer.set_step_size_base(0.75);
    optimizer.update_file_names("data/2d_pR/mean_multiobs.csv", "data/2d_pR/cov_multiobs.csv", "data/2d_pR/cost_multiobs.csv");

    optimizer.optimize();

    return 0;
}