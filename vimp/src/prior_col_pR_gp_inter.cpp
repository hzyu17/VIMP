/**
 * @file test_conv_prior_col_pR_gp_inter.cpp
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Test prior + collision cost only on supported states, 
 * for a planar robot and GP linear interpolations.
 * @version 0.1
 * @date 2022-07-18
 * 
 * @copyright Copyright (c) 2022
 * 
 */


#include <gtsam/inference/Symbol.h>
#include <gpmp2/gp/GPutils.h>
#include <vimp/helpers/data_io.h>
#include <vimp/instances/PriorColPlanarPRGPLinear.h>

using namespace std;
using namespace gpmp2;
using namespace Eigen;
using namespace vimp;


/**
 * @brief -log(p(x,z)) for prior and collision factors 
 * 
 * @param joint_theta the input joint confs and vels
 * @param prior_factor the prior class
 * @param obstacle_factor the collision class
 * @return double 
 */
double errorWrapperPriorColGPLinaer(const VectorXd& joint_theta, 
                                    const GPInterLinearExtend& prior_factor,
                                    const ObstacleFactorGPInterLinPR& collision_factor) {

    /**
     * @brief Robot dimensions
     * 
     */
    
    int tmp = prior_factor.dim();
    const int dim_conf = tmp;

    // int dim_conf = collision_factor.robot_.dof() * collision_factor.robot_.nr_links();
    /// TODO: try to write a class which can return the robot model in the obstacle class.

    VectorXd pose1 = joint_theta.segment(0, dim_conf);
    VectorXd vel1 = joint_theta.segment(dim_conf, dim_conf);
    VectorXd pose2 = joint_theta.segment(2*dim_conf, dim_conf);
    VectorXd vel2 = joint_theta.segment(3*dim_conf, dim_conf);
    
    /**
     * Prior factor
     * */
    VectorXd vec_prior_err = prior_factor.evaluateError(pose1, vel1, pose2, vel2);
    MatrixXd Qc = gpmp2::getQc(prior_factor.get_noiseModel());
    MatrixXd invQ = prior_factor.inv_Qc();
    double prior_err = vec_prior_err.transpose() * invQ * vec_prior_err;

    /**
     * Obstacle factor
     * */
    VectorXd vec_col_err = collision_factor.evaluateError(pose1, vel1, pose2, vel2);

    // MatrixXd precision_obs
    MatrixXd precision_obs{MatrixXd::Identity(vec_err.rows(), vec_err.rows())};
    precision_obs = precision_obs / collision_factor.get_noiseModel()->sigmas()[0];

    double collision_cost = vec_col_err.transpose() * precision_obs * vec_col_err;

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
    string filename_map{"data/2d_gpinter_pR/map_ground_truth.csv"};
    string filename_sdf{"data/2d_gpinter_pR/map_sdf.csv"};

    matrix_io.saveData<MatrixXd>(filename_map, map_ground_truth);
    matrix_io.saveData<MatrixXd>(filename_sdf, field);
    // layout of SDF: Bottom-left is (0,0), length is +/- 1 per point.
    Point2 origin(0, 0);
    double cell_size = 1.0;

    PlanarSDF sdf = PlanarSDF(origin, cell_size, field);
    double cost_sigma = 1.0;
    double epsilon = 1.0;

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
    double r = 1.0;
    BodySphereVector body_spheres;
    body_spheres.push_back(BodySphere(0, r, Point3(0.0, 0.0, 0.0)));
    PointRobotModel pRModel(pR, body_spheres);

    /// start and goal
    double start_x = 1.0, start_y = 1.5, goal_x = 5.5, goal_y = 5.5;
    VectorXd start_theta(dim_theta);
    start_theta << start_x, start_y, 0, 0;
    VectorXd goal_theta(dim_theta);
    goal_theta << goal_x, goal_y, 0, 0;
    
    /// Noise model
    SharedNoiseModel Qc_model = noiseModel::Isotropic::Sigma(dim_conf, 0.5);
    SharedNoiseModel K_0 = noiseModel::Isotropic::Sigma(dim_conf, 0.5);

    /// Vector of factored optimizers
    vector<std::shared_ptr<OptFactPriColGHGPInterLinPR>> vec_factor_opts;

    /// initial values
    VectorXd joint_init_theta{VectorXd::Zero(ndim)};

    double delta_t = 1.0;
    double ninterp = 5;

    for (int i = 0; i < n_total_states-1; i++) {
        VectorXd theta{start_theta + double(i) * (goal_theta - start_theta) / N};
        joint_init_theta.segment<dim_theta>(i*dim_theta) = std::move(theta);

        /// Pk matrices
        MatrixXd Pk{MatrixXd::Zero(2 * dim_theta, ndim)};
        Pk.block<2*dim_theta, 2*dim_theta>(0, 2*i * dim_theta) = MatrixXd::Identity(2*dim_theta, 2*dim_theta);
        
        cout << "Pk" << endl << Pk;

        /// prior
        GaussianProcessPriorLinear prior_k{gtsam::symbol('x', i), gtsam::symbol('v', i),
                                           gtsam::symbol('x', i+1), gtsam::symbol('v', i+1), 
                                           delta_t,
                                           Qc_model};

        for (int j=0; j<ninterp; j++){
            double tau = delta_t / ninterp * double(j);
            // collision
            ObstacleFactorGPInterLinPR collision_k{gtsam::symbol('x', i), gtsam::symbol('v', i),
                                                gtsam::symbol('x', i+1), gtsam::symbol('v', i+1), 
                                                pRModel, 
                                                sdf, 
                                                cost_sigma, 
                                                epsilon,
                                                Qc_model,
                                                delta_t,
                                                tau};
            /// Factored optimizer
            std::shared_ptr<OptFactPriColGHGPInterLinPR> pOptimizer(new OptFactPriColGHGPInterLinPR{dim_conf, errorWrapperPriorColGPLinaer, prior_k, collision_k, Pk});
            vec_factor_opts.emplace_back(pOptimizer);
        }

    }

    /// The joint optimizer
    VIMPOptimizerGH<OptFactPriColGHGPInterLinPR> optimizer{vec_factor_opts};

    /// Set initial value to the linear interpolation
    optimizer.set_mu(joint_init_theta);

    /// Update n iterations and data file names
    int num_iter = 20;
    optimizer.set_niterations(num_iter);
    optimizer.update_file_names("data/2d_gpinter_pR/mean.csv", "data/2d_gpinter_pR/cov.csv");

    optimizer.optimize();

    return 0;
}