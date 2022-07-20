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

#include "instances/PriorColPlanarPR.h"
#include <gtsam/inference/Symbol.h>
#include "helpers/data_io.h"

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
                            const ObstaclePlanarSDFFactorPR& collision_factor) {

    /**
     * Prior factor
     * */
    VectorXd vec_prior_err = prior_factor.evaluateError(pose);
    MatrixXd Qc = prior_factor.get_Qc();
    double prior_err = vec_prior_err.transpose() * Qc.inverse() * vec_prior_err;

    /**
     * Obstacle factor
     * */
    VectorXd vec_err = collision_factor.evaluateError(pose);

    // MatrixXd precision_obs;
    MatrixXd precision_obs{MatrixXd::Identity(vec_err.rows(), vec_err.rows())};
    precision_obs = precision_obs / collision_factor.get_noiseModel()->sigmas()[0];

    double collision_cost = vec_err.transpose() * precision_obs * vec_err;

    return prior_err + collision_cost;
}

int main(){
    /// map and sdf
    MatrixXd map_ground_truth = (MatrixXd(7, 7) <<
            0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0,
            0, 0, 1, 1, 1, 0, 0,
            0, 0, 1, 1, 1, 0, 0,
            0, 0, 1, 1, 1, 0, 0,
            0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0).finished();
    MatrixXd field = (MatrixXd(7, 7) <<
            2.8284, 2.2361, 2.0000, 2.0000, 2.0000, 2.2361, 2.8284,
            2.2361, 1.4142, 1.0000, 1.0000, 1.0000, 1.4142, 2.2361,
            2.0000, 1.0000, -1.0000, -1.0000, -1.0000, 1.0000, 2.0000,
            2.0000, 1.0000, -1.0000, -2.0000, -1.0000, 1.0000, 2.0000,
            2.0000, 1.0000, -1.0000, -1.0000, -1.0000, 1.0000, 2.0000,
            2.2361, 1.4142, 1.0000, 1.0000, 1.0000, 1.4142, 2.2361,
            2.8284, 2.2361, 2.0000, 2.0000, 2.0000, 2.2361, 2.8284).finished();

    MatrixIO matrix_io{};
    string filename_map{"data/2d_pR/map_ground_truth.csv"};
    string filename_sdf{"data/2d_pR/map_sdf.csv"};

    matrix_io.saveData(filename_map, map_ground_truth);
    matrix_io.saveData(filename_sdf, field);
    // layout of SDF: Bottom-left is (0,0), length is +/- 1 per point.
    Point2 origin(0, 0);
    double cell_size = 1.0;

    PlanarSDF sdf = PlanarSDF(origin, cell_size, field);
    double cost_sigma = 1.0;
    double epsilon = 1.0;

    /// 2D point robot
    int n_total_states = 20, N = n_total_states - 1;

    /// parameters
    const int ndof = 2, nlinks = 1;
    const int dim_conf = ndof * nlinks;
    const int dim_theta = 2 * dim_conf; // theta = [conf, vel_conf]
    /// dimension of the joint optimization problem
    const int ndim = dim_theta * n_total_states;

    /// Robot model
    PointRobot pR(ndof, nlinks);
    double r = 1.0;
    BodySphereVector body_spheres;
    body_spheres.push_back(BodySphere(0, r, Point3(0.0, 0.0, 0.0)));
    PointRobotModel pRModel(pR, body_spheres);

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
    vector<std::shared_ptr<OptFactPriColPlanarPRGH>> vec_factor_opts;

    /// initial values
    VectorXd joint_init_theta{VectorXd::Zero(ndim)};

    for (int i = 0; i < n_total_states; i++) {
        VectorXd theta{start_theta + double(i) * (goal_theta - start_theta) / N};
        joint_init_theta.segment<dim_theta>(i*dim_theta) = std::move(theta);

        /// Pk matrices
        MatrixXd Pk{MatrixXd::Zero(dim_conf, ndim)};
        Pk.block<dim_conf, dim_conf>(0, i * dim_theta) = MatrixXd::Identity(dim_conf, dim_conf);
        
        /// prior
        UnaryFactorTranslation2D prior_k{gtsam::symbol('x', i), Vector2d{theta.segment<dim_conf>(0)}, K_0};

        // collision
        ObstaclePlanarSDFFactorPR collision_k{gtsam::symbol('x', i), pRModel, sdf, cost_sigma, epsilon};

        /// Factored optimizer
        std::shared_ptr<OptFactPriColPlanarPRGH> pOptimizer{new OptFactPriColPlanarPRGH{dim_conf, errorWrapperPriorCol, prior_k, collision_k, Pk}};
        vec_factor_opts.emplace_back(pOptimizer);

    }

    /// The joint optimizer
    VIMPOptimizerGH<OptFactPriColPlanarPRGH> optimizer{vec_factor_opts};

    /// Set initial value to the linear interpolation
    optimizer.set_mu(joint_init_theta);

    /// Update n iterations and data file names
    int num_iter = 5;
    optimizer.set_niterations(num_iter);
    optimizer.update_file_names("data/2d_pR/mean.csv", "data/2d_pR/cov.csv");

    optimizer.optimize();

    return 0;
}