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

#include "../include/OptimizerPriorColPlanarPR.h"
#include <gtsam/inference/Symbol.h>
#include "../include/helpers/data_io.h"


/**
 * @brief -log(p(x,z)) for prior and collision factors 
 * 
 * @param theta the input vector
 * @param prior_factor the prior class
 * @param obstacle_factor the collision class
 * @return double 
 */
double errorWrapperPriorCol(const VectorXd& theta, const UnaryFactorTranslation2D& prior_factor,
                                      const ObstaclePlanarSDFFactorPointRobot& collision_factor) {

    /**
     * Prior factor
     * */
    Vector2d position;
    position = theta.segment<2>(0);
    VectorXd vec_prior_err = prior_factor.evaluateError(position);
    MatrixXd K = prior_factor.get_Qc();
    double prior_err = vec_prior_err.transpose() * K.inverse() * vec_prior_err;

    /**
     * Obstacle factor
     * */
    VectorXd vec_err = collision_factor.evaluateError(theta);

    MatrixXd precision_obs{MatrixXd::Identity(vec_err.rows(), vec_err.rows())};
    double sig_obs = 1.0;
    precision_obs = precision_obs / sig_obs;

    double collision_cost = vec_err.transpose() * precision_obs * vec_err;

    return prior_err + collision_cost;
}


using namespace std;
using namespace gpmp2;
using namespace Eigen;
using namespace MPVI;

int main(){
    MatrixIO matrix_io{};
    string filename_map{"map_ground_truth.csv"};
    string filename_sdf{"map_sdf.csv"};
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

    matrix_io.saveData(filename_map, map_ground_truth);
    matrix_io.saveData(filename_sdf, field);
    // layout of SDF: Bottom-left is (0,0), length is +/- 1 per point.
    Point2 origin(0, 0);
    double cell_size = 1.0;

    PlanarSDF sdf = PlanarSDF(origin, cell_size, field);
    double cost_sigma = 1.0;
    double epsilon = 0.3;

    /// 2D point robot
    int n_total_states = 3, N = n_total_states - 1;

    /// parameters
    const int ndof = 2, nlinks = 1;
    const int dim_conf = ndof * nlinks;
    const int dim_theta = 2 * dim_conf; // theta = [conf, vel_conf]
    /// dimension of the joint optimization problem
    const int ndim = dim_theta * n_total_states;

    /// Robot model
    PointRobot pR(ndof, nlinks);
    double r = 0.5;
    BodySphereVector body_spheres;
    body_spheres.push_back(BodySphere(0, r, Point3(0.0, 0.0, 0.0)));
    PointRobotModel pRModel(pR, body_spheres);

    /// start and goal
    double start_x = 1.0, start_y = 2.0, goal_x = 5.5, goal_y = 2.0;
    VectorXd start_theta(dim_theta);
    start_theta << start_x, start_y, 0, 0;
    VectorXd goal_theta(dim_theta);
    goal_theta << goal_x, goal_y, 0, 0;

    /// factored optimizers: containing 2 types: 
    /// 1. the single factor of priors for start and goal states;
    /// 2. the prior + collision factors for supported states.
    
    /// Noise models
    SharedNoiseModel Qc_model = noiseModel::Isotropic::Sigma(dim_conf, 1.0);
    cout << "Qc" << endl << getQc(Qc_model) << endl;
    SharedNoiseModel K_0 = noiseModel::Isotropic::Sigma(dim_conf, 1.0);

    /// Vector of factored optimizers
    vector<std::shared_ptr<OptFactPriColPRGH>> vec_factor_opts;

    /// initial values
    VectorXd joint_init_theta{ndim} = VectorXd::Zero(ndim);

    for (int i = 0; i < n_total_states; i++) {
        VectorXd theta{start_theta + double(i) * (goal_theta - start_theta) / N};
        joint_init_theta.segment<dim_theta>(i*dim_theta) = std::move(theta);
        cout << "theta" << endl << theta << endl;
        cout << "joint_init_theta" << endl << joint_init_theta << endl;

        /// Pk matrices
        MatrixXd Pk{MatrixXd::Zero(dim_conf, ndim)};
        Pk.block<dim_conf, dim_conf>(0, i * dim_theta) = MatrixXd::Identity(dim_conf, dim_conf);
        /// Pk.block<dim_conf, dim_conf>(dim_conf, (i + 1) * dim_conf) = MatrixXd::Identity(dim_conf, dim_conf);
        
        cout << "Pk" << endl << Pk << endl;

        /// prior cost
        UnaryFactorTranslation2D prior_k{gtsam::symbol('x', i), Vector2d{theta.segment<dim_conf>(0)}, K_0};
        cout << prior_k.get_Qc() << endl;

        // collision cost
        ObstaclePlanarSDFFactorPointRobot collision_k{gtsam::symbol('x', i), pRModel, sdf, cost_sigma, epsilon};

        /// Factored optimizer
        std::shared_ptr<OptFactPriColPRGH> pOptimizer(new OptFactPriColPRGH{dim_conf, errorWrapperPriorCol, prior_k, collision_k, Pk});
        vec_factor_opts.emplace_back(pOptimizer);

    }

    /// The joint optimizer
    VIMPOptimizerGH<OptFactPriColPRGH> optimizer{vec_factor_opts};

    /// Set initial value to the linear interpolation
    optimizer.set_mu(joint_init_theta);

    /// Update steps
    int num_iter = 10;

    MatrixXd results{MatrixXd::Zero(num_iter, ndim)};

    optimizer.set_niterations(num_iter);
    optimizer.optimize();

    return 0;
}