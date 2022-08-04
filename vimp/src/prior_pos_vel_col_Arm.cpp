/**
 * @file prior_pos_vel_col_Arm.cpp
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Experiment for arm robot model
 * @version 0.1
 * @date 2022-08-03
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "../instances/PriorColPlanarArm.h"
#include "../robots/PlanarArmSDFExample.h"
#include <gtsam/inference/Symbol.h>

using namespace std;
using namespace gpmp2;
using namespace Eigen;
using namespace vimp;


int main(){
    // An example pr and sdf
    vimp::PlanarArmSDFExample planar_arm_sdf;
    gpmp2::ArmModel arm_model = std::move(planar_arm_sdf.arm_model());
    gpmp2::PlanarSDF sdf = std::move(planar_arm_sdf.sdf());

    /// parameters1
    int n_total_states = 8, N = n_total_states - 1;
    const int ndof = planar_arm_sdf.ndof(), nlinks = planar_arm_sdf.nlinks();
    const int dim_conf = ndof * nlinks;
    const int dim_theta = 2 * dim_conf; // theta = [conf, vel_conf]
    /// dimension of the joint optimization problem
    const int ndim = dim_theta * n_total_states;

    /// start and goal
    const double PI = 3.1415926;
    double start_x = 0.0, start_y = 0.0, goal_x = PI / 2, goal_y = 0;
    VectorXd start_theta(dim_theta);
    start_theta << start_x, start_y, 0, 0;
    VectorXd goal_theta(dim_theta);
    goal_theta << goal_x, goal_y, 0, 0;

    /// prior 
    double total_time_sec = 1.0;
    double delta_t = total_time_sec / N;

    VectorXd avg_vel{(goal_theta.segment(0, dim_conf) - start_theta.segment(0, dim_conf)) / total_time_sec};

    // linear GP
    MatrixXd Qc = MatrixXd::Identity(dim_conf, dim_conf);

    /// Obs factor
    // double cost_sigma = 1.8, epsilon = 1.0;
    double cost_sigma = 0.1, epsilon = 0.1;

    /// Vector of base factored optimizers
    vector<std::shared_ptr<VIMPOptimizerFactorizedBase>> vec_factor_opts;

    /// initial values
    VectorXd joint_init_theta{VectorXd::Zero(ndim)};

    for (int i = 0; i < n_total_states; i++) {
        // initial state
        VectorXd theta{start_theta + double(i) * (goal_theta - start_theta) / N};

        // initial velocity: must have initial velocity for the fitst state??
        theta.segment(dim_conf, dim_conf) = avg_vel;
        joint_init_theta.segment(i*dim_theta, dim_theta) = std::move(theta);   

        // fixed start and goal priors
        if (i==0 || i==n_total_states-1){
            /// lin GP factor for the N th state
            if (i == n_total_states-1){
                MatrixXd Pk_lingp{MatrixXd::Zero(2*dim_theta, ndim)};
                Pk_lingp.block(0, (i-1) * dim_theta, 2*dim_theta, 2*dim_theta) = std::move(MatrixXd::Identity(2*dim_theta, 2*dim_theta));

                MinimumAccGP lin_gp{Qc, delta_t};

                std::shared_ptr<LinearGpPrior> p_lin_gp{new LinearGpPrior{2*dim_theta, cost_linear_gp, lin_gp, Pk_lingp}}; 
                vec_factor_opts.emplace_back(p_lin_gp);

            }

            /// Fixed GP
            FixedPriorGP fixed_gp{MatrixXd::Identity(dim_theta, dim_theta)*0.0001, MatrixXd{theta}};
            MatrixXd Pk{MatrixXd::Zero(dim_theta, ndim)};
            Pk.block(0, i * dim_theta, dim_theta, dim_theta) = std::move(MatrixXd::Identity(dim_theta, dim_theta));

            std::shared_ptr<FixedGpPrior> p_fix_gp{new FixedGpPrior{dim_theta, cost_fixed_gp, fixed_gp, Pk}};
            vec_factor_opts.emplace_back(p_fix_gp);

        }else{
            // support states: linear gp priors
            MatrixXd Pk{MatrixXd::Zero(2*dim_theta, ndim)};
            Pk.block(0, (i-1) * dim_theta, 2*dim_theta, 2*dim_theta) = std::move(MatrixXd::Identity(2*dim_theta, 2*dim_theta));

            MinimumAccGP lin_gp{Qc, delta_t};

            // linear gp factor
            std::shared_ptr<LinearGpPrior> p_lin_gp{new LinearGpPrior{2*dim_theta, cost_linear_gp, lin_gp, Pk}}; 
            vec_factor_opts.emplace_back(p_lin_gp);

            // collision factor
            gpmp2::ObstaclePlanarSDFFactorArm collision_k{gtsam::symbol('x', i), arm_model, sdf, cost_sigma, epsilon};

            MatrixXd Pk_col{MatrixXd::Zero(dim_conf, ndim)};
            Pk_col.block(0, i * dim_theta, dim_conf, dim_conf) = std::move(MatrixXd::Identity(dim_conf, dim_conf));

            /// Factored optimizer
            std::shared_ptr<OptPlanarSDFFactorArm> p_obs{new OptPlanarSDFFactorArm{dim_conf, cost_sdf_Arm, collision_k, Pk_col}};
            vec_factor_opts.emplace_back(p_obs);
        }
        
    }

    /// The joint optimizer
    VIMPOptimizerGH<VIMPOptimizerFactorizedBase> optimizer{vec_factor_opts};

    /// Set initial value to the linear interpolation
    int num_iter = 10;
    optimizer.set_mu(joint_init_theta);
    optimizer.set_GH_degree(3);
    optimizer.set_niterations(num_iter);
    // optimizer.set_step_size_base(0.35, 1e-4);
    optimizer.set_step_size_base(0.25, 1e-4);
    optimizer.update_file_names("/home/hongzhe/git/VIMP/vimp/data/2d_Arm/mean.csv", 
                                "/home/hongzhe/git/VIMP/vimp/data/2d_Arm/cov.csv", 
                                "/home/hongzhe/git/VIMP/vimp/data/2d_Arm/precisoin.csv", 
                                "/home/hongzhe/git/VIMP/vimp/data/2d_Arm/cost.csv",
                                "/home/hongzhe/git/VIMP/vimp/data/2d_Arm/factor_costs.csv");

    optimizer.optimize();

    return 0;
}