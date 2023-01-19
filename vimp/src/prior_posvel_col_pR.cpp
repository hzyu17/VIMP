/**
 * @file conv_prior_posvel_col_pR.cpp
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Test the convergence of the algorithm with prior (pos + vel) + collision cost 
 * only on supported states, for a planar robot.
 * @version 0.1
 * @date 2022-07-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "../instances/PriorColPlanarPointRobot.h"
#include "../robots/PlanarPointRobotSDFMultiObsExample.h"


using namespace std;
using namespace rapidxml;
using namespace gpmp2;
using namespace Eigen;
using namespace vimp;

int main(){
    
    /// reading XML configurations
    rapidxml::file<> xmlFile("/home/hongzhe/git/VIMP/vimp/configs/planar_pR_map2.xml"); // Default template is char
    // rapidxml::file<> xmlFile("/home/hongzhe/git/VIMP/vimp/configs/planar_pR_map2.xml");
    rapidxml::xml_document<> doc;
    doc.parse<0>(xmlFile.data());
    rapidxml::xml_node<>* paramNode = doc.first_node("parameters");

    string field_file = static_cast<std::string>(paramNode->first_node("field_file")->value());

    double start_x = atof(paramNode->first_node("start_pos")->first_node("x")->value());
    double start_y = atof(paramNode->first_node("start_pos")->first_node("y")->value());

    double goal_x = atof(paramNode->first_node("goal_pos")->first_node("x")->value());
    double goal_y = atof(paramNode->first_node("goal_pos")->first_node("y")->value());

    int n_total_states = atoi(paramNode->first_node("n_total_states")->value());
    double total_time_sec = atof(paramNode->first_node("total_time")->value());

    double Temperature = atof(paramNode->first_node("temperature")->value());
    
    double weight_Qc = atof(paramNode->first_node("coeff_Qc")->value());
    double cost_sigma = atof(paramNode->first_node("cost_sigma")->value());
    double epsilon = atof(paramNode->first_node("epsilon")->value());
    double step_size = atof(paramNode->first_node("step_size")->value());
    int num_iter = atoi(paramNode->first_node("num_iter")->value());
    
    double init_precision_factor = atof(paramNode->first_node("init_precision_factor")->value());
    
    int replanning = atoi(paramNode->first_node("replanning")->value());
    string replan_mean_file = static_cast<std::string>(paramNode->first_node("mean_file")->value());
    int replan_start = atoi(paramNode->first_node("replanning_starting")->value());
    
    MatrixIO matrix_io;
    // An example pr and sdf
    vimp::PlanarPointRobotSDFMultiObsExample planar_pr_sdf;
    gpmp2::PointRobotModel pRModel = std::move(planar_pr_sdf.pRmodel());

    MatrixXd field = matrix_io.load_csv(field_file);

    // layout of SDF: Bottom-left is (0,0), length is +/- cell_size per grid.
    Point2 origin(-20, -10);
    double cell_size = 0.1;

    gpmp2::PlanarSDF sdf = PlanarSDF(origin, cell_size, field);
    
    /// parameters
    int N = n_total_states - 1;
    const int ndof = planar_pr_sdf.ndof(), nlinks = planar_pr_sdf.nlinks();
    const int dim_conf = ndof * nlinks;
    const int dim_theta = 2 * dim_conf; // theta = [conf, vel_conf]
    /// dimension of the joint optimization problem
    const int ndim = dim_theta * n_total_states;

    /// start and goal
    VectorXd start_theta(dim_theta);
    start_theta << start_x, start_y, 0, 0;
    VectorXd goal_theta(dim_theta);
    goal_theta << goal_x, goal_y, 0, 0;

    /// prior 
    double delta_t = total_time_sec / N;

    VectorXd avg_vel{(goal_theta.segment(0, dim_conf) - start_theta.segment(0, dim_conf)) / total_time_sec};

    MatrixXd Qc = MatrixXd::Identity(dim_conf, dim_conf)*weight_Qc;
    MatrixXd K0_fixed = MatrixXd::Identity(dim_theta, dim_theta)*0.0001;

    /// Vector of base factored optimizers
    vector<std::shared_ptr<VIMPOptimizerFactorizedBase>> vec_factors;
    /// initial values
    VectorXd joint_init_theta{VectorXd::Zero(ndim)};

    for (int i = 0; i < n_total_states; i++) {
        // initial state
        VectorXd theta{start_theta + double(i) * (goal_theta - start_theta) / N};

        // initial velocity: must have initial velocity for the fitst state??
        theta.segment(dim_conf, dim_conf) = avg_vel;
        joint_init_theta.segment(i*dim_theta, dim_theta) = std::move(theta);   

        MinimumAccGP lin_gp{Qc, delta_t};

        // fixed start and goal priors
        // Factor Order: [fixed_gp_0, lin_gp_1, obs_1, ..., lin_gp_(N-1), obs_(N-1), lin_gp_(N), fixed_gp_(N)] 
        if (i==0 || i==n_total_states-1){

            // lin GP factor
            if (i == n_total_states-1){
                
                // std::shared_ptr<LinearGpPrior> p_lin_gp{}; 
                vec_factors.emplace_back(new LinearGpPrior{2*dim_theta, dim_theta, cost_linear_gp, lin_gp, n_total_states, i-1});
            }

            // Fixed gp factor
            FixedPriorGP fixed_gp{K0_fixed, MatrixXd{theta}};
            vec_factors.emplace_back(new FixedGpPrior{dim_theta, dim_theta, cost_fixed_gp, fixed_gp, n_total_states, i});

        }else{
            // linear gp factors
            vec_factors.emplace_back(new LinearGpPrior{2*dim_theta, dim_theta, cost_linear_gp, lin_gp, n_total_states, i-1});

            // collision factors
            gpmp2::ObstaclePlanarSDFFactorPointRobot collision_k{gtsam::symbol('x', i), pRModel, sdf, cost_sigma, epsilon};
            vec_factors.emplace_back(new OptPlanarSDFFactorPointRobot{dim_conf, dim_theta, cost_sdf_pR, collision_k, n_total_states, i});
        }
        
    }
    /// The joint optimizer
    VIMPOptimizerGH<VIMPOptimizerFactorizedBase> optimizer{vec_factors, dim_theta, n_total_states, Temperature};
    if (replanning == 1){
        MatrixXd means = matrix_io.load_csv(replan_mean_file);
        VectorXd good_init_vec = means.row(replan_start);
        optimizer.update_file_names("/home/hongzhe/git/VIMP/vimp/data/2d_pR/mean.csv", 
                                    "/home/hongzhe/git/VIMP/vimp/data/2d_pR/cov.csv", 
                                    "/home/hongzhe/git/VIMP/vimp/data/2d_pR/precisoin.csv", 
                                    "/home/hongzhe/git/VIMP/vimp/data/2d_pR/cost.csv",
                                    "/home/hongzhe/git/VIMP/vimp/data/2d_pR/factor_costs.csv",
                                    "/home/hongzhe/git/VIMP/vimp/data/2d_pR/perturbation_statistics.csv");
        /// Set initial value to the linear interpolation
        optimizer.set_mu(good_init_vec);
    }else{
        optimizer.update_file_names("/home/hongzhe/git/VIMP/vimp/data/2d_pR/mean_base.csv", 
                                    "/home/hongzhe/git/VIMP/vimp/data/2d_pR/cov_base.csv", 
                                    "/home/hongzhe/git/VIMP/vimp/data/2d_pR/precisoin_base.csv", 
                                    "/home/hongzhe/git/VIMP/vimp/data/2d_pR/cost_base.csv",
                                    "/home/hongzhe/git/VIMP/vimp/data/2d_pR/factor_costs_base.csv",
                                    "/home/hongzhe/git/VIMP/vimp/data/2d_pR/perturbation_statistics_base.csv");
        optimizer.set_mu(joint_init_theta);
        std::cout << "/home/hongzhe/git/VIMP/vimp/data/2d_pR/mean_base.csv" << std::endl;
    }


    MatrixXd init_precision{MatrixXd::Identity(ndim, ndim)*init_precision_factor};
    init_precision.block(0, 0, dim_theta, dim_theta) = MatrixXd::Identity(dim_theta, dim_theta)*10000;
    init_precision.block(N*dim_theta, N*dim_theta, dim_theta, dim_theta) = MatrixXd::Identity(dim_theta, dim_theta)*10000;
    optimizer.set_precision(init_precision.sparseView());

    optimizer.set_GH_degree(3);
    optimizer.set_niterations(num_iter);

    optimizer.set_step_size_base(step_size); // a local optima
    optimizer.optimize();
    return 0;
}