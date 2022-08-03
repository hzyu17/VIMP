/**
 * @file conv_prior_posvel_col_pR.cpp
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Test the convergence of the algorithm with prior (pos + vel) + collision cost only on supported states, 
 * for a planar robot.
 * @version 0.1
 * @date 2022-07-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "../instances/PriorColPlanarPointRobot.h"
#include "../instances/PlanarPointRobotSDFMultiObsExample.h"


using namespace std;
using namespace gpmp2;
using namespace Eigen;
using namespace vimp;

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

    /// start and goal
    double start_x = 0.0, start_y = 0.0, goal_x = 17.0, goal_y = 14.0;
    VectorXd start_theta(dim_theta);
    start_theta << start_x, start_y, 0, 0;
    VectorXd goal_theta(dim_theta);
    goal_theta << goal_x, goal_y, 0, 0;

    /// factored optimizers: containing 2 types: 
    /// 1. the single factor of priors for start and goal states;
    /// 2. the prior + collision factors for supported states.

    /// prior 
    double total_time_sec = 1.5;
    double delta_t = total_time_sec / N;

    VectorXd avg_vel{(goal_theta.segment(0, dim_conf) - start_theta.segment(0, dim_conf)) / total_time_sec};

    /// Obs factor
    double cost_sigma = 2.5, epsilon = 4.0;

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
            /// position
            FixedPriorGP fixed_gp{MatrixXd::Identity(dim_theta, dim_theta)*0.0001, MatrixXd{theta}};
            MatrixXd Pk{MatrixXd::Zero(dim_theta, ndim)};
            Pk.block(0, i * dim_theta, dim_theta, dim_theta) = std::move(MatrixXd::Identity(dim_theta, dim_theta));

            std::shared_ptr<FixedGpPrior> p_fix_gp{new FixedGpPrior{dim_theta, cost_fixed_gp, fixed_gp, Pk}};
            vec_factor_opts.emplace_back(p_fix_gp);

            /// lin GP factor
            if (i == n_total_states-1){
                MatrixXd Pk_lingp{MatrixXd::Zero(2*dim_theta, ndim)};
                Pk_lingp.block(0, (i-1) * dim_theta, 2*dim_theta, 2*dim_theta) = std::move(MatrixXd::Identity(2*dim_theta, 2*dim_theta));

                MinimumAccGP lin_gp{MatrixXd::Identity(dim_conf, dim_conf), delta_t};

                std::shared_ptr<LinearGpPrior> p_lin_gp{new LinearGpPrior{2*dim_theta, cost_linear_gp, lin_gp, Pk_lingp}}; 
                vec_factor_opts.emplace_back(p_lin_gp);

            }

        }else{
            // support states: linear gp priors
            MatrixXd Pk{MatrixXd::Zero(2*dim_theta, ndim)};
            Pk.block(0, (i-1) * dim_theta, 2*dim_theta, 2*dim_theta) = std::move(MatrixXd::Identity(2*dim_theta, 2*dim_theta));

            MinimumAccGP lin_gp{MatrixXd::Identity(dim_conf, dim_conf), delta_t};

            // linear gp factor
            std::shared_ptr<LinearGpPrior> p_lin_gp{new LinearGpPrior{2*dim_theta, cost_linear_gp, lin_gp, Pk}}; 
            vec_factor_opts.emplace_back(p_lin_gp);

            // collision factor
            gpmp2::ObstaclePlanarSDFFactorPointRobot collision_k{gtsam::symbol('x', i), pRModel, sdf, cost_sigma, epsilon};

            MatrixXd Pk_col{MatrixXd::Zero(dim_conf, ndim)};
            Pk_col.block(0, i * dim_theta, dim_conf, dim_conf) = std::move(MatrixXd::Identity(dim_conf, dim_conf));

            /// Factored optimizer
            std::shared_ptr<PlanarPRColFactor> p_obs{new PlanarPRColFactor{dim_conf, cost_obstacle, collision_k, Pk_col}};
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
    optimizer.set_step_size_base(0.75, 0.0001);
    optimizer.update_file_names("/home/hongzhe/git/VIMP/vimp/data/2d_pR/mean.csv", 
                                "/home/hongzhe/git/VIMP/vimp/data/2d_pR/cov.csv", 
                                "/home/hongzhe/git/VIMP/vimp/data/2d_pR/precisoin.csv", 
                                "/home/hongzhe/git/VIMP/vimp/data/2d_pR/cost.csv",
                                "/home/hongzhe/git/VIMP/vimp/data/2d_pR/factor_costs.csv");

    optimizer.optimize();

    return 0;
}