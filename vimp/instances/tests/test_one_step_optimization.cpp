/**
 * @file test_one_step_optimization.cpp
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Test one step of the GVI-MP algorithm for a known experiment result. 
 * Any modifications of the algorithm should pass this test.
 * @version 0.1
 * @date 2023-08-07
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "helpers/ExperimentRunner.h"
#include "instances/GVIMPPlanarPRSDF.h"
#include <gtest/gtest.h>

using namespace gpmp2;
using namespace Eigen;
using namespace vimp;
using namespace std;

#define STRING(x) #x
#define XSTRING(x) STRING(x)


std::string config_file{source_root+"/configs/vimp/planar_pR_map1_new.xml"};
int nx = 4, nu = 2, num_exp = 2;
GVIMPRunner<GVIMPPlanarPRSDF> runner(nx, nu, num_exp, config_file);
GVIMPParams params;

// **** ground truths ****
double coeff_Qc = 0.8, temperature = 10.0, high_temperature = 100.0, 
    step_size = 0.35, eps_sdf = 0.5, sig_obs = 0.007, robot_radius=1.5,
    boundary_penalties = 10000.0, backtrack_ratio = 1.0, total_time=1.5;

int nt = 10;

Eigen::Vector2d map_origin{(VectorXd(2) << -20, -10).finished()};
double cell_size = 0.1;

string map_name{"2dpr_map1"};

// robot and sdf 
GVIMPPlanarPRSDF opt_robot_sdf = runner.optimizer_robot_sdf();
PlanarPRSDFExample robot_sdf = opt_robot_sdf.robot_sdf();

// Eigen helpers
EigenWrapper ei;

// The test should be conducted using the same configuations as follows.
TEST(GVIOnestep, parameter_settings){
    // **** read configurations ****
    runner.read_config(params);

    ASSERT_LE(abs(params.coeff_Qc() - coeff_Qc), 1e-8);
    ASSERT_LE(abs(params.temperature() - temperature), 1e-8);
    ASSERT_LE(abs(params.high_temperature() - high_temperature), 1e-8);
    ASSERT_LE(abs(params.step_size() - step_size), 1e-8);
    ASSERT_LE(abs(params.eps_sdf() - eps_sdf), 1e-8);
    ASSERT_LE(abs(params.sig_obs() - sig_obs), 1e-8);
    ASSERT_LE(abs(params.radius() - robot_radius), 1e-8);
    ASSERT_LE(abs(params.boundary_penalties() - boundary_penalties), 1e-8);
    ASSERT_LE(abs(params.backtrack_ratio() - backtrack_ratio), 1e-8);
    ASSERT_LE(abs(params.total_time() - total_time), 1e-8);
    ASSERT_LE(abs(params.nt() - nt), 1e-8);
    ASSERT_LE((robot_sdf.map_origin() - map_origin).norm(), 1e-8);
    ASSERT_LE(abs(robot_sdf.cell_size() - cell_size), 1e-8);

}

// **** Test the costs and all the intermidiate values ****
TEST(GVIOnestep, initial_values){
    std::cout << "***** read common configurations *****" << std::endl;
    runner.read_config(params);

    std::cout << "***** read boundary conditions *****" << std::endl;
    // ***** read boundary conditions *****
    rapidxml::file<> xmlFile(runner._config_file.data()); // Default template is char
    rapidxml::xml_document<> doc;
    doc.parse<0>(xmlFile.data());
    
    std::string ExpNodeName = "Experiment" + std::to_string(1);

    char * c_expname = ExpNodeName.data();
    rapidxml::xml_node<>* paramNode = doc.first_node(c_expname);
    
    runner.read_boundary_conditions(paramNode, params);

    std::cout << "***** Initialize the optimizer *****" << std::endl;
    // ***** Initialize the optimizer *****
    /// parameters
        int n_states = params.nt();
        int N = n_states - 1;
        const int dim_conf = robot_sdf.ndof() * robot_sdf.nlinks();
        // state: theta = [conf, vel_conf]
        const int dim_state = 2 * dim_conf; 
        /// joint dimension
        const int ndim = dim_state * n_states;

        VectorXd start_theta{ params.m0() };
        VectorXd goal_theta{ params.mT() };

        MatrixXd Qc{MatrixXd::Identity(dim_conf, dim_conf)*params.coeff_Qc()};
        MatrixXd K0_fixed{MatrixXd::Identity(dim_state, dim_state)/params.boundary_penalties()};

        /// Vector of base factored optimizers
        vector<std::shared_ptr<GVIFactorizedBase>> vec_factors;

        /// initial values
        VectorXd joint_init_theta{VectorXd::Zero(ndim)};
        VectorXd avg_vel{(goal_theta.segment(0, dim_conf) - start_theta.segment(0, dim_conf)) / params.total_time()};
        
        /// prior 
        double delt_t = params.total_time() / N;

        auto robot_model = robot_sdf.RobotModel();
        auto sdf = robot_sdf.sdf();
        double sig_obs = params.sig_obs(), eps_sdf = params.eps_sdf();
        double temperature = params.temperature();

        std::cout << "***** Initialize the optimizer *****" << std::endl;

        for (int i = 0; i < n_states; i++) {

            // initial state
            VectorXd theta_i{start_theta + double(i) * (goal_theta - start_theta) / N};

            // initial velocity: must have initial velocity for the fitst state??
            theta_i.segment(dim_conf, dim_conf) = avg_vel;
            joint_init_theta.segment(i*dim_state, dim_state) = std::move(theta_i);   

            MinimumAccGP lin_gp{Qc, i, delt_t, start_theta};

            // fixed start and goal priors
            // Factor Order: [fixed_gp_0, lin_gp_1, obs_1, ..., lin_gp_(N-1), obs_(N-1), lin_gp_(N), fixed_gp_(N)] 
            if (i==0 || i==n_states-1){

                // lin GP factor for the first and the last support state
                if (i == n_states-1){
                    // std::shared_ptr<LinearGpPrior> p_lin_gp{}; 
                    vec_factors.emplace_back(new LinearGpPrior{2*dim_state, 
                                                                dim_state, 
                                                                cost_linear_gp, 
                                                                lin_gp, 
                                                                n_states, 
                                                                i-1, 
                                                                params.temperature(), 
                                                                params.high_temperature()});
                }

                // Fixed gp factor
                FixedPriorGP fixed_gp{K0_fixed, MatrixXd{theta_i}};
                vec_factors.emplace_back(new FixedGpPrior{dim_state, 
                                                          dim_state, 
                                                          cost_fixed_gp, 
                                                          fixed_gp, 
                                                          n_states, 
                                                          i, 
                                                          params.temperature(), 
                                                          params.high_temperature()});

            }else{
                // linear gp factors
                vec_factors.emplace_back(new LinearGpPrior{2*dim_state, 
                                                            dim_state, 
                                                            cost_linear_gp, 
                                                            lin_gp, 
                                                            n_states, 
                                                            i-1, 
                                                            params.temperature(), 
                                                            params.high_temperature()});

                // collision factor
                vec_factors.emplace_back(new PlanarSDFFactorPR{dim_conf, 
                                                                dim_state, 
                                                                cost_sdf_pR, 
                                                                SDFPR{gtsam::symbol('x', i), 
                                                                robot_model, 
                                                                sdf, 
                                                                sig_obs, 
                                                                eps_sdf}, 
                                                                n_states, 
                                                                i, 
                                                                params.temperature(), 
                                                                params.high_temperature()});    
            }
        }

        std::cout << "***** Declare joint optimizer *****" << std::endl;

        /// The joint optimizer
        GVIGH<GVIFactorizedBase> optimizer{vec_factors, 
                                           dim_state, 
                                           n_states, 
                                           params.max_iter(), 
                                           params.temperature(), 
                                           params.high_temperature()};

        std::cout << "***** Initialize joint optimizer *****" << std::endl;

        optimizer.set_max_iter_backtrack(params.max_n_backtrack());
        optimizer.set_niter_low_temperature(params.max_iter_lowtemp());
        optimizer.set_stop_err(params.stop_err());

        optimizer.update_file_names(params.saving_prefix());
        optimizer.set_mu(joint_init_theta);

        // initial precision matrix for the optimization
        MatrixXd init_precision(ndim, ndim);
        init_precision = MatrixXd::Identity(ndim, ndim)*params.initial_precision_factor();
        
        // boundaries
        init_precision.block(0, 0, dim_state, dim_state) = MatrixXd::Identity(dim_state, dim_state)*params.boundary_penalties();
        init_precision.block(N*dim_state, N*dim_state, dim_state, dim_state) = MatrixXd::Identity(dim_state, dim_state)*params.boundary_penalties();
        optimizer.set_precision(init_precision.sparseView());

        optimizer.set_GH_degree(3);
        optimizer.set_step_size_base(params.step_size()); // a local optima

        // ***** Test initial joint mean and covariance *****
        VectorXd joint_mean = optimizer.mean();
        SpMat joint_precision = optimizer.precision();

        ASSERT_LE((joint_mean - joint_init_theta).norm(), 1e-8);
        ASSERT_LE((joint_precision - init_precision).norm(), 1e-8);

        std::cout << "***** Test the values before the optimization starts *****" << std::endl;
        // ***** Test the factor cost values *****
        VectorXd factor_cost_vec{optimizer.factor_cost_vector()};
        VectorXd factor_cost_vec_gt(19);
        factor_cost_vec_gt.setZero();
        EigenWrapper ei;
        ei.print_matrix(factor_cost_vec, "factor_cost_vec");

        factor_cost_vec_gt <<  0.399999999999981, 32.7327000000002, 0, 65.3999999999997, 0, 
                               65.3999999999997, 0, 65.3999999999997, 0.739105727743422, 65.3999999999997,
                               58.3163759352287, 65.3999999999997, 222.108617990813, 65.3999999999997, 145.689597500301,
                               65.3999999999997, 21.6208111387552, 32.7327000000002, 0.399999999999962;
        ASSERT_LE((factor_cost_vec - factor_cost_vec_gt).norm(), 1e-8);

        // ***** Test One step optimization *****
        // one step gradients
        std::tuple<VectorXd, SpMat> dmudprecision = optimizer.compute_gradients();

        VectorXd dmu_one_step = std::get<0>(dmudprecision);
        SpMat dprecision_one_step = std::get<1>(dmudprecision);

        // MatrixIO mio;
        // mio.saveData("dmu_onestep.csv", dmu_one_step);
        // mio.saveData("dprecision_onestep.csv", MatrixXd{dprecision_one_step});

        VectorXd Vdmu_one_step{optimizer.Vdmu()};
        MatrixXd Vddmu_one_step{optimizer.Vddmu()};

        // MatrixIO mio;
        // mio.saveData("Vdmu_one_step.csv", Vdmu_one_step);
        // mio.saveData("Vddmu_one_step.csv", MatrixXd{Vddmu_one_step});

        VectorXd Vdmu_gt(ndim);
        Vdmu_gt.setZero();
        Vdmu_gt<< 0, 0, 0, 0, 0, 0, 0,
                  0, -2.87769807982841e-13, 0,
                  -2.39808173319034e-14, 0, 2.87769807982841e-13, 0,
                  -2.39808173319034e-14, 0, 3.97791188917929, 0.508761403327395,
                    0, 0, 62.1860445904482, 7.38298311375729e-14,
                    0, 0, 110.541566479321, 1.83939042141675, 4.79616346638068e-14,
                    0, -90.7936521658056, -3.5527136788005e-14, 4.79616346638068e-14,
                    0, -33.9682530137282, -4.44089209850063e-15, 0, 0,
                    0, 0, 0, 0;

        ASSERT_LE((Vdmu_one_step - Vdmu_gt).norm(), 1e-8);

        MatrixXd Vddmu_gt(ndim, ndim);
        Vddmu_gt.setZero();
        Vddmu_gt <<    2323.99999999995, 0, 27, 0, -324, 0, 27, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                        0, 2323.99999999995, 0, 27, 0, -324, 0, 27, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                        27, 0, 2002.99999999995, 0, -27, 0, 1.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                        0, 27, 0, 2002.99999999995, 0, -27, 0, 1.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                        -324, 0, -27, 0, 648.000000000001, 0, 3.5527136788005e-15, 0, -324, 0, 27, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                        0, -324, 0, -27, 0, 648.000000000001, 0, 3.5527136788005e-15, 0, -324, 0, 27, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                        27, 0, 1.5, 0, 3.5527136788005e-15, 0, 6.00000000000018, 0, -27, 0, 1.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                        0, 27, 0, 1.5, 0, 3.5527136788005e-15, 0, 6.00000000000018, 0, -27, 0, 1.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, -324, 0, -27, 0, 648.000000000001, 0, 3.5527136788005e-15, 0, -324, 0, 27, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, -324, 0, -27, 0, 648.000000000001, 0, 3.5527136788005e-15, 0, -324, 0, 27, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 27, 0, 1.5, 0, 3.5527136788005e-15, 0, 6.00000000000036, 0, -27, 0, 1.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 27, 0, 1.5, 0, 3.5527136788005e-15, 0, 6.00000000000036, 0, -27, 0, 1.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, -324, 0, -27, 0, 648.000000000001, 0, 3.5527136788005e-15, 0, -324, 0, 27, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, -324, 0, -27, 0, 648.000000000001, 0, 3.5527136788005e-15, 0, -324, 0, 27, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 27, 0, 1.5, 0, 3.5527136788005e-15, 0, 6.00000000000036, 0, -27, 0, 1.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 27, 0, 1.5, 0, 3.5527136788005e-15, 0, 6.00000000000036, 0, -27, 0, 1.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -324, 0, -27, 0, 662.396863457281, 2.63962853062749, 3.5527136788005e-15, 0, -324, 0, 27, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -324, 0, -27, 2.63962853062749, 646.781632140921, 0, 3.5527136788005e-15, 0, -324, 0, 27, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 27, 0, 1.5, 0, 3.5527136788005e-15, 0, 6.00000000000036, 0, -27, 0, 1.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 27, 0, 1.5, 0, 3.5527136788005e-15, 0, 6.00000000000036, 0, -27, 0, 1.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -324, 0, -27, 0, 684.133515365418, 4.9960036108132e-14, 3.5527136788005e-15, 0, -324, 0, 27, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -324, 0, -27, 4.9960036108132e-14, 648.000000000001, 0, 3.5527136788005e-15, 0, -324, 0, 27, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 27, 0, 1.5, 0, 3.5527136788005e-15, 0, 6.00000000000036, 0, -27, 0, 1.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 27, 0, 1.5, 0, 3.5527136788005e-15, 0, 6.00000000000036, 0, -27, 0, 1.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -324, 0, -27, 0, 669.854935223751, 10.0747562586889, 3.5527136788005e-15, 0, -324, 0, 27, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -324, 0, -27, 10.0747562586889, 641.28349582754, 0, 3.5527136788005e-15, 0, -324, 0, 27, 0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 27, 0, 1.5, 0, 3.5527136788005e-15, 0, 6.00000000000036, 0, -27, 0, 1.5, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 27, 0, 1.5, 0, 3.5527136788005e-15, 0, 6.00000000000036, 0, -27, 0, 1.5, 0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -324, 0, -27, 0, 676.571414485489, 0, 3.5527136788005e-15, 0, -324, 0, 27, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -324, 0, -27, 0, 648, 0, 3.5527136788005e-15, 0, -324, 0, 27, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 27, 0, 1.5, 0, 3.5527136788005e-15, 0, 6.00000000000036, 0, -27, 0, 1.5, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 27, 0, 1.5, 0, 3.5527136788005e-15, 0, 6.00000000000036, 0, -27, 0, 1.5, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -324, 0, -27, 0, 676.571430996084, 0, 3.5527136788005e-15, 0, -324, 0, 27, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -324, 0, -27, 0, 648, 0, 3.5527136788005e-15, 0, -324, 0, 27,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 27, 0, 1.5, 0, 3.5527136788005e-15, 0, 6.00000000000009, 0, -27, 0, 1.5, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 27, 0, 1.5, 0, 3.5527136788005e-15, 0, 6.00000000000009, 0, -27, 0, 1.5,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -324, 0, -27, 0, 2323.99999999995, 0, -27, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -324, 0, -27, 0, 2323.99999999995, 0, -27,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 27, 0, 1.5, 0, -27, 0, 2002.99999999995, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 27, 0, 1.5, 0, -27, 0, 2002.99999999995;
        ASSERT_LE((Vddmu_one_step - Vddmu_gt).norm(), 1e-7);

        VectorXd dmu_gt(ndim);
        dmu_gt.setZero();
        dmu_gt<< -0.00509588286804194, 0.0434789828171249, -0.00230979508695942,
                 0.0176400851720914, -0.455856511644597, 3.42966035850035, -5.02934494519157,
                 37.3958710291656, -1.55571992958528, 11.435214194586, -7.79154326505658,
                 55.4501096100018, -2.92721333164946, 20.8394750972159, -8.28890475467324,
                 54.180355827695, -4.1928639127966, 28.421777672533, -6.52142941405214,
                 33.5866096822485, -5.09024058327872, 31.2403887626056, -4.55986811844314,
                 -1.31034857968193, -5.06558379513711, 27.4693041532037, 9.05194400569646,
                 -45.4897387114474, -2.38147903514197, 17.0040001844276, 17.1097999621166,
                 -72.9856909817231, -0.397686483366718, 5.51014647000758, 6.53100775770025,
                 -57.8323356591729, 0.0204418732955952, 0.095960942083943, 0.000745358958228711,
                -0.0296727437669396;

        ASSERT_LE((dmu_one_step - dmu_gt).norm(), 1e-7);

        MatrixXd dprecision_gt(ndim, ndim);
        dprecision_gt.setZero();
        dprecision_gt << -7676.00000000005, 0, 27, 0, -324, 0, 27, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, -7676.00000000005, 0, 27, 0, -324, 0, 27, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            27, 0, -7997.00000000005, 0, -27, 0, 1.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 27, 0, -7997.00000000005, 0, -27, 0, 1.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            -324, 0, -27, 0, 638.000000000001, 0, 3.5527136788005e-15, 0, -324, 0, 27, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, -324, 0, -27, 0, 638.000000000001, 0, 3.5527136788005e-15, 0, -324, 0, 27, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            27, 0, 1.5, 0, 3.5527136788005e-15, 0, -3.99999999999982, 0, -27, 0, 1.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 27, 0, 1.5, 0, 3.5527136788005e-15, 0, -3.99999999999982, 0, -27, 0, 1.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, -324, 0, -27, 0, 638.000000000001, 0, 3.5527136788005e-15, 0, -324, 0, 27, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, -324, 0, -27, 0, 638.000000000001, 0, 3.5527136788005e-15, 0, -324, 0, 27, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 27, 0, 1.5, 0, 3.5527136788005e-15, 0, -3.99999999999964, 0, -27, 0, 1.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 27, 0, 1.5, 0, 3.5527136788005e-15, 0, -3.99999999999964, 0, -27, 0, 1.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, -324, 0, -27, 0, 638.000000000001, 0, 3.5527136788005e-15, 0, -324, 0, 27, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, -324, 0, -27, 0, 638.000000000001, 0, 3.5527136788005e-15, 0, -324, 0, 27, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 27, 0, 1.5, 0, 3.5527136788005e-15, 0, -3.99999999999964, 0, -27, 0, 1.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 27, 0, 1.5, 0, 3.5527136788005e-15, 0, -3.99999999999964, 0, -27, 0, 1.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -324, 0, -27, 0, 652.396863457281, 2.63962853062749, 3.5527136788005e-15, 0, -324, 0, 27, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -324, 0, -27, 2.63962853062749, 636.781632140921, 0, 3.5527136788005e-15, 0, -324, 0, 27, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 27, 0, 1.5, 0, 3.5527136788005e-15, 0, -3.99999999999964, 0, -27, 0, 1.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 27, 0, 1.5, 0, 3.5527136788005e-15, 0, -3.99999999999964, 0, -27, 0, 1.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -324, 0, -27, 0, 674.133515365418, 4.9960036108132e-14, 3.5527136788005e-15, 0, -324, 0, 27, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -324, 0, -27, 4.9960036108132e-14, 638.000000000001, 0, 3.5527136788005e-15, 0, -324, 0, 27, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 27, 0, 1.5, 0, 3.5527136788005e-15, 0, -3.99999999999964, 0, -27, 0, 1.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 27, 0, 1.5, 0, 3.5527136788005e-15, 0, -3.99999999999964, 0, -27, 0, 1.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -324, 0, -27, 0, 659.854935223751, 10.0747562586889, 3.5527136788005e-15, 0, -324, 0, 27, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -324, 0, -27, 10.0747562586889, 631.28349582754, 0, 3.5527136788005e-15, 0, -324, 0, 27, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 27, 0, 1.5, 0, 3.5527136788005e-15, 0, -3.99999999999964, 0, -27, 0, 1.5, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 27, 0, 1.5, 0, 3.5527136788005e-15, 0, -3.99999999999964, 0, -27, 0, 1.5, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -324, 0, -27, 0, 666.571414485489, 0, 3.5527136788005e-15, 0, -324, 0, 27, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -324, 0, -27, 0, 638, 0, 3.5527136788005e-15, 0, -324, 0, 27, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 27, 0, 1.5, 0, 3.5527136788005e-15, 0, -3.99999999999964, 0, -27, 0, 1.5, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 27, 0, 1.5, 0, 3.5527136788005e-15, 0, -3.99999999999964, 0, -27, 0, 1.5, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -324, 0, -27, 0, 666.571430996084, 0, 3.5527136788005e-15, 0, -324, 0, 27, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -324, 0, -27, 0, 638, 0, 3.5527136788005e-15, 0, -324, 0, 27,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 27, 0, 1.5, 0, 3.5527136788005e-15, 0, -3.99999999999991, 0, -27, 0, 1.5, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 27, 0, 1.5, 0, 3.5527136788005e-15, 0, -3.99999999999991, 0, -27, 0, 1.5,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -324, 0, -27, 0, -7676.00000000005, 0, -27, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -324, 0, -27, 0, -7676.00000000005, 0, -27,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 27, 0, 1.5, 0, -27, 0, -7997.00000000005, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 27, 0, 1.5, 0, -27, 0, -7997.00000000005;
        ASSERT_LE((MatrixXd{dprecision_one_step} - dprecision_gt).norm(), 1e-7);
}