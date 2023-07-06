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

// #include "../../instances/PlanarArmFactor.h"
// #include "../../robots/PlanarArmSDFExample.h"
// #include <gtsam/inference/Symbol.h>

// using namespace std;
// using namespace gpmp2;
// using namespace Eigen;
// using namespace vimp;


int main(){
    // /// reading XML configurations
    // rapidxml::file<> xmlFile("configs/planar_2link_arm_map2.xml"); // Default template is char
    // rapidxml::xml_document<> doc;
    // doc.parse<0>(xmlFile.data());
    // rapidxml::xml_node<>* paramNode = doc.first_node("parameters");
    
    // string field_file = static_cast<std::string>(paramNode->first_node("field_file")->value());

    // double start_x = atof(paramNode->first_node("start_pos")->first_node("x")->value());
    // double start_y = atof(paramNode->first_node("start_pos")->first_node("y")->value());

    // double goal_x = atof(paramNode->first_node("goal_pos")->first_node("x")->value());
    // double goal_y = atof(paramNode->first_node("goal_pos")->first_node("y")->value());

    // int n_total_states = atoi(paramNode->first_node("n_total_states")->value());
    // double total_time_sec = atof(paramNode->first_node("total_time")->value());

    // double weight_Qc = atof(paramNode->first_node("coeff_Qc")->value());
    // double cost_sigma = atof(paramNode->first_node("cost_sigma")->value());
    // double epsilon = atof(paramNode->first_node("epsilon")->value());
    // double step_size = atof(paramNode->first_node("step_size")->value());

    // int num_iter = atoi(paramNode->first_node("num_iter")->value());

    // double init_precision_factor = atof(paramNode->first_node("init_precision_factor")->value());

    // int replanning = atoi(paramNode->first_node("replanning")->value());
    // string replan_mean_file = static_cast<std::string>(paramNode->first_node("mean_file")->value());
    // int replanning_starting = atoi(paramNode->first_node("replanning_starting")->value());

    // // An example pr and sdf
    // vimp::PlanarArmSDFExample planar_arm_sdf(field_file);

    // gpmp2::ArmModel arm_model = std::move(planar_arm_sdf.arm_model());
    // gpmp2::PlanarSDF sdf = std::move(planar_arm_sdf.sdf());

    // /// parameters1
    // // int n_total_states = 8;
    // int N = n_total_states - 1;
    // const int ndof = planar_arm_sdf.ndof(), nlinks = planar_arm_sdf.nlinks();
    // const int dim_conf = ndof * nlinks;
    // const int dim_theta = 2 * dim_conf; // theta = [conf, vel_conf]
    // /// dimension of the joint optimization problem
    // const int ndim = dim_theta * n_total_states;

    // /// start and goal
    // const double PI = 3.1415926;
    // // double start_x = 0.0, start_y = 0.0, goal_x = PI / 2, goal_y = 0;
    // VectorXd start_theta(dim_theta);
    // start_theta << start_x, start_y, 0, 0;
    // VectorXd goal_theta(dim_theta);
    // goal_theta << goal_x, goal_y, 0, 0;

    // /// prior 
    // // double total_time_sec = 1.0;
    // double delta_t = total_time_sec / N;

    // VectorXd avg_vel{(goal_theta.segment(0, dim_conf) - start_theta.segment(0, dim_conf)) / total_time_sec};

    // // linear GP
    // MatrixXd Qc = MatrixXd::Identity(dim_conf, dim_conf) * weight_Qc;

    // /// Obs factor
    // // double cost_sigma = 0.05, epsilon = 0.1;

    // /// Vector of base factored optimizers
    // vector<std::shared_ptr<GVIFactorizedBase>> vec_factor_opts;

    // /// initial values
    // VectorXd joint_init_theta{VectorXd::Zero(ndim)};

    // for (int i = 0; i < n_total_states; i++) {
    //     // initial state
    //     VectorXd theta{start_theta + double(i) * (goal_theta - start_theta) / N};

    //     // initial velocity: must have initial velocity for the fitst state??
    //     theta.segment(dim_conf, dim_conf) = avg_vel;
    //     joint_init_theta.segment(i*dim_theta, dim_theta) = std::move(theta);   

    //     // fixed start and goal priors
    //     if (i==0 || i==n_total_states-1){
    //         /// lin GP factor for the N th state
    //         if (i == n_total_states-1){
    //             MinimumAccGP lin_gp{Qc, delta_t};

    //             std::shared_ptr<LinearGpPrior> p_lin_gp{new LinearGpPrior{2*dim_theta, dim_theta, cost_linear_gp, lin_gp, n_total_states, i-1}}; 
    //             vec_factor_opts.emplace_back(p_lin_gp);

    //         }

    //         /// Fixed GP
    //         FixedPriorGP fixed_gp{MatrixXd::Identity(dim_theta, dim_theta)*0.0001, MatrixXd{theta}};

    //         std::shared_ptr<FixedGpPrior> p_fix_gp{new FixedGpPrior{dim_theta, dim_theta, cost_fixed_gp, fixed_gp, n_total_states, i}};
    //         vec_factor_opts.emplace_back(p_fix_gp);

    //     }else{
    //         // support states: linear gp priors
    //         MinimumAccGP lin_gp{Qc, delta_t};

    //         // linear gp factor
    //         std::shared_ptr<LinearGpPrior> p_lin_gp{new LinearGpPrior{2*dim_theta, dim_theta, cost_linear_gp, lin_gp, n_total_states, i-1}}; 
    //         vec_factor_opts.emplace_back(p_lin_gp);

    //         // collision factor
    //         gpmp2::ObstaclePlanarSDFFactorArm collision_k{gtsam::symbol('x', i), arm_model, sdf, cost_sigma, epsilon};

    //         /// Factored optimizer
    //         std::shared_ptr<OptPlanarSDFFactorArm> p_obs{new OptPlanarSDFFactorArm{dim_conf, dim_theta, cost_sdf_Arm, collision_k, n_total_states, i}};
    //         vec_factor_opts.emplace_back(p_obs);
    //     }
        
    // }

    // /// The joint optimizer
    // VIMPOptimizerGH<GVIFactorizedBase> optimizer{vec_factor_opts, dim_theta, n_total_states};

    // MatrixIO matrix_io;
    // /// Set initial value to the linear interpolation
    // // int num_iter = 20;
    // if (replanning == 1){
    //     MatrixXd means = matrix_io.load_csv(replan_mean_file);
    //     // VectorXd good_init_vec = means.row(means.rows()-1);
    //     VectorXd good_init_vec = means.row(replanning_starting);
    //     /// Set initial value to the linear interpolation
    //     optimizer.set_mu(good_init_vec);
    // }else{
    //     optimizer.set_mu(joint_init_theta);
    // }

    // // optimizer.set_mu(joint_init_theta);

    // MatrixXd init_precision{MatrixXd::Identity(ndim, ndim)*init_precision_factor};
    // init_precision.block(0, 0, dim_theta, dim_theta) = MatrixXd::Identity(dim_theta, dim_theta)*10000;
    // init_precision.block(N*dim_theta, N*dim_theta, dim_theta, dim_theta) = MatrixXd::Identity(dim_theta, dim_theta)*10000;

    // optimizer.set_precision(init_precision.sparseView());

    // optimizer.set_GH_degree(3);
    // optimizer.set_niterations(num_iter);
    // optimizer.set_step_size_base(step_size);

    // if (replanning==0){
    //     optimizer.update_file_names("/home/hyu419/git/VIMP/vimp/data/2d_Arm/mean_base.csv", 
    //                             "/home/hyu419/git/VIMP/vimp/data/2d_Arm/cov_base.csv", 
    //                             "/home/hyu419/git/VIMP/vimp/data/2d_Arm/precisoin_base.csv", 
    //                             "/home/hyu419/git/VIMP/vimp/data/2d_Arm/cost_base.csv",
    //                             "/home/hyu419/git/VIMP/vimp/data/2d_Arm/factor_costs_base.csv",
    //                             "/home/hyu419/git/VIMP/vimp/data/2d_Arm/perturbation_statistics_base.csv");
    // }else{
    //     optimizer.update_file_names("/home/hyu419/git/VIMP/vimp/data/2d_Arm/mean.csv", 
    //                             "/home/hyu419/git/VIMP/vimp/data/2d_Arm/cov.csv", 
    //                             "/home/hyu419/git/VIMP/vimp/data/2d_Arm/precisoin.csv", 
    //                             "/home/hyu419/git/VIMP/vimp/data/2d_Arm/cost.csv",
    //                             "/home/hyu419/git/VIMP/vimp/data/2d_Arm/factor_costs.csv",
    //                             "/home/hyu419/git/VIMP/vimp/data/2d_Arm/perturbation_statistics.csv");
    // }

    // optimizer.optimize();

    return 0;
}