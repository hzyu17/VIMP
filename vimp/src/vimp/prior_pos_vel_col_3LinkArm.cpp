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

#include "../../instances/PlanarArmFactor.h"
#include "../../robots/Planar3LinkArmSDFExample.h"
#include <gtsam/inference/Symbol.h>

using namespace std;
using namespace gpmp2;
using namespace Eigen;
using namespace vimp;


int main(){
    /// reading XML configurations
    rapidxml::file<> xmlFile("configs/planar_3link_arm_map1.xml"); // Default template is char
    rapidxml::xml_document<> doc;
    doc.parse<0>(xmlFile.data());
    rapidxml::xml_node<>* paramNode = doc.first_node("parameters");
    
    string field_file = static_cast<std::string>(paramNode->first_node("field_file")->value());

    double start_1 = atof(paramNode->first_node("start_pos")->first_node("1")->value());
    double start_2 = atof(paramNode->first_node("start_pos")->first_node("2")->value());
    double start_3 = atof(paramNode->first_node("start_pos")->first_node("3")->value());

    double goal_1 = atof(paramNode->first_node("goal_pos")->first_node("1")->value());
    double goal_2 = atof(paramNode->first_node("goal_pos")->first_node("2")->value());
    double goal_3 = atof(paramNode->first_node("goal_pos")->first_node("3")->value());

    int n_total_states = atoi(paramNode->first_node("n_total_states")->value());
    double total_time_sec = atof(paramNode->first_node("total_time")->value());

    double weight_Qc = atof(paramNode->first_node("coeff_Qc")->value());
    double cost_sigma = atof(paramNode->first_node("cost_sigma")->value());
    double epsilon = atof(paramNode->first_node("epsilon")->value());
    double step_size = atof(paramNode->first_node("step_size")->value());

    int num_iter = atoi(paramNode->first_node("num_iter")->value());

    double init_precision_factor = atof(paramNode->first_node("init_precision_factor")->value());

    int replanning = atoi(paramNode->first_node("replanning")->value());
    int replanning_starting = atoi(paramNode->first_node("replanning_starting")->value());

    // An example pr and sdf
    vimp::Planar3LinkArmSDFExample planar_arm_sdf(field_file);

    gpmp2::ArmModel arm_model = std::move(planar_arm_sdf.arm_model());
    gpmp2::PlanarSDF sdf = std::move(planar_arm_sdf.sdf());

    /// parameters1
    // int n_total_states = 8;
    int N = n_total_states - 1;
    const int ndof = planar_arm_sdf.ndof(), nlinks = planar_arm_sdf.nlinks();
    const int dim_conf = ndof * nlinks;
    const int dim_theta = 2 * dim_conf; // theta = [conf, vel_conf]
    /// dimension of the joint optimization problem
    const int ndim = dim_theta * n_total_states;

    /// start and goal
    VectorXd start_theta(dim_theta);
    start_theta << start_1, start_2, start_3, 0, 0, 0;
    VectorXd goal_theta(dim_theta);
    goal_theta << goal_1, goal_2, goal_3, 0, 0, 0;

    double delta_t = total_time_sec / N;

    // average velocity
    VectorXd avg_vel{(goal_theta.segment(0, dim_conf) - start_theta.segment(0, dim_conf)) / total_time_sec};

    // linear GP
    MatrixXd Qc = MatrixXd::Identity(dim_conf, dim_conf) * weight_Qc;

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

        // linear gp_i
        MinimumAccGP lin_gp{Qc, delta_t}; 

        // fixed start and goal priors
        if (i==0 || i==n_total_states-1){
            /// lin GP factor for the N th state
            if (i == n_total_states-1){
                vec_factor_opts.emplace_back(new LinearGpPrior{2*dim_theta, dim_theta, cost_linear_gp, lin_gp, n_total_states, i-1});

            }

            /// Fixed GP
            FixedPriorGP fixed_gp{MatrixXd::Identity(dim_theta, dim_theta)*0.0001, MatrixXd{theta}};
            vec_factor_opts.emplace_back(new FixedGpPrior{dim_theta, dim_theta, cost_fixed_gp, fixed_gp, n_total_states, i});

        }else{
            // linear gp factor
            vec_factor_opts.emplace_back(new LinearGpPrior{2*dim_theta, dim_theta, cost_linear_gp, lin_gp, n_total_states, i-1});

            // collision factor
            gpmp2::ObstaclePlanarSDFFactorArm collision_k{gtsam::symbol('x', i), arm_model, sdf, cost_sigma, epsilon};

            /// Factored optimizer
            vec_factor_opts.emplace_back(new OptPlanarSDFFactorArm{dim_conf, dim_theta, cost_sdf_Arm, collision_k, n_total_states, i});
        }
        
    }

    /// The joint optimizer
    VIMPOptimizerGH<VIMPOptimizerFactorizedBase> optimizer{vec_factor_opts, dim_theta, n_total_states};

    MatrixIO matrix_io;
    /// Set initial value to the linear interpolation
    // int num_iter = 20;
    if (replanning == 1){
        MatrixXd means = matrix_io.load_csv("/home/hongzhe/git/VIMP/vimp/data/2d_3Arm/mean_base.csv");
        // VectorXd good_init_vec = means.row(means.rows()-1);
        VectorXd good_init_vec = means.row(replanning_starting);
        /// Set initial value to the linear interpolation
        optimizer.set_mu(good_init_vec);
    }else{
        optimizer.set_mu(joint_init_theta);
    }

    MatrixXd init_precision{MatrixXd::Identity(ndim, ndim)*init_precision_factor};
    init_precision.block(0, 0, dim_theta, dim_theta) = MatrixXd::Identity(dim_theta, dim_theta)*1000;
    init_precision.block(N*dim_theta, N*dim_theta, dim_theta, dim_theta) = MatrixXd::Identity(dim_theta, dim_theta)*1000;
    optimizer.set_precision(init_precision.sparseView());

    optimizer.set_GH_degree(3);
    optimizer.set_niterations(num_iter);
    optimizer.set_step_size_base(step_size);

    if (replanning==0){
        optimizer.update_file_names("/home/hongzhe/git/VIMP/vimp/data/2d_Arm3/mean_base.csv", 
                                "/home/hongzhe/git/VIMP/vimp/data/2d_Arm3/cov_base.csv", 
                                "/home/hongzhe/git/VIMP/vimp/data/2d_Arm3/precisoin_base.csv", 
                                "/home/hongzhe/git/VIMP/vimp/data/2d_Arm3/cost_base.csv",
                                "/home/hongzhe/git/VIMP/vimp/data/2d_Arm3/factor_costs_base.csv",
                                "/home/hongzhe/git/VIMP/vimp/data/2d_Arm3/perturbation_statistics_base.csv");
    }else{
        optimizer.update_file_names("/home/hongzhe/git/VIMP/vimp/data/2d_Arm3/mean.csv", 
                                "/home/hongzhe/git/VIMP/vimp/data/2d_Arm3/cov.csv", 
                                "/home/hongzhe/git/VIMP/vimp/data/2d_Arm3/precisoin.csv", 
                                "/home/hongzhe/git/VIMP/vimp/data/2d_Arm3/cost.csv",
                                "/home/hongzhe/git/VIMP/vimp/data/2d_Arm3/factor_costs.csv",
                                "/home/hongzhe/git/VIMP/vimp/data/2d_Arm3/perturbation_statistics.csv");
    }

    optimizer.optimize();

    return 0;
}