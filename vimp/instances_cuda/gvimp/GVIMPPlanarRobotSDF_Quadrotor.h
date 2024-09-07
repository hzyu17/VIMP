/**
 * @file GVIMPPlanarRobotSDF.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief The optimizer for planar robots at the joint level.
 * @version 0.1
 * @date 2023-06-24
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "helpers/timer.h"
#include "helpers/ExperimentParams.h"
#include "GaussianVI/gp/factorized_opts_LTV.h"
#include "instances_cuda/CostFunctions_LTV.h"
#include "GaussianVI/ngd/NGDFactorizedBaseGH_Quadrotor.h"
#include "GaussianVI/ngd/NGD-GH.h"

#include "dynamics/PlanarQuad_linearization.h"

std::string GH_map_file{source_root+"/GaussianVI/quadrature/SparseGHQuadratureWeights.bin"};

namespace vimp{

class GVIMPPlanarRobotSDF_Quadrotor{

public:
    virtual ~GVIMPPlanarRobotSDF_Quadrotor(){}

    GVIMPPlanarRobotSDF_Quadrotor(){}

    GVIMPPlanarRobotSDF_Quadrotor(GVIMPParams& params){}

    double run_optimization_withtime(const GVIMPParams& params, bool verbose=true){
        Timer timer;
        timer.start();
        _last_iteration_mean_precision = run_optimization_return(params, verbose);

        std::cout << "========== Optimization time: " << std::endl;
        return timer.end_sec();
    }

    void run_optimization(const GVIMPParams& params, bool verbose=true){
        _last_iteration_mean_precision = run_optimization_return(params, verbose);
    }

    std::tuple<Eigen::VectorXd, gvi::SpMat> run_optimization_return(const GVIMPParams& params, bool verbose=true){
        
        // Read the sparse grid GH quadrature weights and nodes
        QuadratureWeightsMap nodes_weights_map;
        try {
            std::ifstream ifs(GH_map_file, std::ios::binary);
            if (!ifs.is_open()) {
                std::string error_msg = "Failed to open file for GH weights reading in file: " + GH_map_file;
                throw std::runtime_error(error_msg);
            }

            std::cout << "Opening file for GH weights reading in file: " << GH_map_file << std::endl;
            boost::archive::binary_iarchive ia(ifs);
            ia >> nodes_weights_map;

        } catch (const boost::archive::archive_exception& e) {
            std::cerr << "Boost archive exception: " << e.what() << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "Standard exception: " << e.what() << std::endl;
        }

        /// parameters
        int n_states = params.nt();
        int N = n_states - 1;
        const int dim_conf = 3;
        // state: theta = [conf, vel_conf]
        const int dim_state = 2 * dim_conf; 
        /// joint dimension
        const int ndim = dim_state * n_states;

        VectorXd start_theta{ params.m0() };
        VectorXd goal_theta{ params.mT() };

        MatrixXd Qc{MatrixXd::Identity(dim_conf, dim_conf)*params.coeff_Qc()};
        MatrixXd K0_fixed{MatrixXd::Identity(dim_state, dim_state)/params.boundary_penalties()};

        /// Vector of base factored optimizers
        vector<std::shared_ptr<gvi::GVIFactorizedBase_Cuda>> vec_factors;
        
        // Obtain the parameters from params and RobotSDF(PlanarPRSDFExample Here)
        double sig_obs = params.sig_obs(), eps_sdf = params.eps_sdf();
        double temperature = params.temperature();

        /// initial values
        VectorXd joint_init_theta{VectorXd::Zero(ndim)};
        VectorXd avg_vel{(goal_theta.segment(0, dim_conf) - start_theta.segment(0, dim_conf)) / params.total_time()};
        
        /// prior 
        double delt_t = params.total_time() / N;

        /// Initialize the trajectory
        std::vector<MatrixXd> hA(n_states);
        std::vector<MatrixXd> hB(n_states);
        std::vector<MatrixXd> Phi_vec(n_states+1);

        std::tuple<std::vector<MatrixXd>, std::vector<MatrixXd>, std::vector<VectorXd>> linearized_matrices;
        std::vector<MatrixXd> hB1;
        MatrixXd start_point (1, start_theta.size());
        start_point.row(0) = start_theta;
        start_point.block(0, dim_conf, 1, dim_conf) = ((goal_theta.segment(0, dim_conf) - start_theta.segment(0, dim_conf)).transpose() ) / params.total_time();

        linearized_matrices = planarquad_linearization_deterministic(start_point);
        hB1 = std::get<1>(linearized_matrices);

        std::cout << "x0 = " << std::endl << start_point << std::endl;
        std::cout << "hA1 = " << std::endl << std::get<0>(linearized_matrices)[0] << std::endl;
        std::cout << "hB1 = " << std::endl << std::get<1>(linearized_matrices)[0] << std::endl << std::endl;
         
        /// Obtain the Transition Matrices by integration
        MatrixXd Phi = MatrixXd::Identity(dim_state, dim_state);
        Phi_vec[0] = Phi;
        for (int i = 0; i < n_states; i++){
            hA[i] = std::get<0>(linearized_matrices)[0];
            hB[i] = hB1[0];
            Phi = Phi + hA[i] * Phi * delt_t;
            Phi_vec[i + 1] = Phi;
        }

        for (int i = 0; i < n_states; i++) {

            // initial state(Need to change for the quadrotor)
            VectorXd theta_i{start_theta + double(i) * (goal_theta - start_theta) / N};

            // // Checking the collision cost
            // VectorXd theta_i(6); 
            // theta_i << 10, 10, 0.78539816339, -2, -1, 0;

            // // initial velocity: must have initial velocity for the fitst state??
            // theta_i.segment(dim_conf, dim_conf) = avg_vel;
            // joint_init_theta.segment(i*dim_state, dim_state) = std::move(theta_i);   
            

            gvi::LTV_GP lin_gp{Qc, i, delt_t, start_theta, n_states, hA, hB, Phi_vec};
            // gvi::QuadGP lin_gp{Qc, i, delt_t, start_theta, n_states, hA, hb, Phi_vec};


            // fixed start and goal priors
            // Factor Order: [fixed_gp_0, lin_gp_1, obs_1, ..., lin_gp_(N-1), obs_(N-1), lin_gp_(N), fixed_gp_(N)] 
            if (i==0 || i==n_states-1){
                std::cout << "---------------- Building fixed start and goal priors ----------------" << std::endl;
                // lin GP factor for the first and the last support state
                if (i == n_states-1){
                    vec_factors.emplace_back(new gvi::LinearGpPrior{2*dim_state, 
                                                                dim_state, 
                                                                gvi::cost_linear_gp, 
                                                                lin_gp, 
                                                                n_states, 
                                                                i-1, 
                                                                params.temperature(), 
                                                                params.high_temperature()});
                }

                // Fixed gp factor
                gvi::FixedPriorGP fixed_gp{K0_fixed, MatrixXd{theta_i}};
                vec_factors.emplace_back(new gvi::FixedGpPrior{dim_state, 
                                                          dim_state, 
                                                          gvi::cost_fixed_gp, 
                                                          fixed_gp, 
                                                          n_states, 
                                                          i,
                                                          params.temperature(), 
                                                          params.high_temperature()});

            }else{
                // linear gp factors
                vec_factors.emplace_back(new gvi::LinearGpPrior{2*dim_state, 
                                                            dim_state, 
                                                            gvi::cost_linear_gp, 
                                                            lin_gp, 
                                                            n_states, 
                                                            i-1, 
                                                            params.temperature(), 
                                                            params.high_temperature()});

                // collision factor
                vec_factors.emplace_back(new NGDFactorizedBaseGH_Quadrotor{dim_conf, 
                                                                        dim_state, 
                                                                        params.GH_degree(),
                                                                        n_states, 
                                                                        i, 
                                                                        params.sig_obs(), 
                                                                        params.eps_sdf(), 
                                                                        params.radius(), 
                                                                        params.temperature(), 
                                                                        params.high_temperature(),
                                                                        nodes_weights_map});    
            }
        }

        /// The joint optimizer
        gvi::NGDGH<gvi::GVIFactorizedBase_Cuda> optimizer{vec_factors, 
                                           dim_state, 
                                           n_states, 
                                           params.max_iter(), 
                                           params.temperature(), 
                                           params.high_temperature()};

        optimizer.set_max_iter_backtrack(params.max_n_backtrack());
        optimizer.set_niter_low_temperature(params.max_iter_lowtemp());
        optimizer.set_stop_err(params.stop_err());

        optimizer.update_file_names(params.saving_prefix());
        optimizer.set_mu(joint_init_theta);

        optimizer.initilize_precision_matrix(params.initial_precision_factor());

        // optimizer.set_GH_degree(params.GH_degree());
        optimizer.set_step_size_base(params.step_size()); // a local optima

        std::cout << "---------------- Start the optimization ----------------" << std::endl;
        optimizer.optimize(verbose);

        _last_iteration_mean_precision = std::make_tuple(optimizer.mean(), optimizer.precision());

        return _last_iteration_mean_precision;

    }

    std::tuple<VectorXd, gvi::SpMat> get_mu_precision(){
        return _last_iteration_mean_precision;
    }

protected: 
    double _eps_sdf;
    double _sig_obs; // The inverse of Covariance matrix related to the obs penalty. 
    gvi::EigenWrapper _ei;
    std::shared_ptr<gvi::NGDGH<gvi::GVIFactorizedBase_Cuda>> _p_opt;

    std::tuple<Eigen::VectorXd, gvi::SpMat> _last_iteration_mean_precision;

};

} // namespace vimp