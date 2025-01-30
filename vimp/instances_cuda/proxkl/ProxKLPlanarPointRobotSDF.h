/**
 * @file ProxKLPlanarRobotSDF.h
 * @author Zinuo Chang (zchang40@gatech.edu)
 * @brief The optimizer for planar point robots at the joint level.
 * @version 0.1
 * @date 2024-12-20
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "helpers/timer.h"
#include "helpers/ExperimentParams.h"
#include "GaussianVI/proxKL/ProxKLFactorizedBaseGH_Cuda.h"
#include "GaussianVI/proxKL/ProxKL-GH-Cuda.h"

#include "ngd/NGDFactorizedLinear_Cuda.h"
#include "gp/fixed_prior_proxkl.h"
#include "gp/minimum_acc_prior_proxkl.h"

namespace gvi{
    using FixedGpPrior = NGDFactorizedLinear_Cuda<FixedPriorGP_proxkl>;
    using LinearGpPrior = NGDFactorizedLinear_Cuda<MinimumAccGP_proxkl>;

    double cost_fixed_gp(const VectorXd& x, const gvi::FixedPriorGP_proxkl& fixed_gp){
        return fixed_gp.fixed_factor_cost(x);
    }

    double cost_linear_gp(const VectorXd& pose_cmb, const gvi::MinimumAccGP_proxkl& gp_minacc){
        int dim = gp_minacc.dim_posvel();
        return gp_minacc.cost(pose_cmb.segment(0, dim), pose_cmb.segment(dim, dim));
    }
}

std::string GH_map_file{source_root+"/GaussianVI/quadrature/SparseGHQuadratureWeights_cereal.bin"};

namespace vimp{

using GHFunction = std::function<MatrixXd(const VectorXd&)>;
using GH = SparseGaussHermite_Cuda<GHFunction>;
using ProxKLFactorizedBaseGH = ProxKLFactorizedBaseGH_Cuda<CudaOperation_PlanarPR>;

class ProxKLPlanarPointRobotSDF{

public:
    virtual ~ProxKLPlanarPointRobotSDF(){}

    ProxKLPlanarPointRobotSDF(){}

    ProxKLPlanarPointRobotSDF(GVIMPParams& params){}

    double run_optimization_withtime(const GVIMPParams& params, bool verbose=true){
        Timer timer;
        timer.start();
        _last_iteration_mean_precision = run_optimization_return(params, verbose);

        std::cout << "========== Cereal Optimization time: " << std::endl;
        return timer.end_sec();
    }

    void run_optimization(const GVIMPParams& params, bool verbose=true){
        _last_iteration_mean_precision = run_optimization_return(params, verbose);
    }

    std::tuple<Eigen::VectorXd, gvi::SpMat> run_optimization_return(const GVIMPParams& params, bool verbose=true){
        
        QuadratureWeightsMap nodes_weights_map;
        try {
            std::ifstream ifs(GH_map_file, std::ios::binary);
            if (!ifs.is_open()) {
                std::string error_msg = "Failed to open file for GH weights reading in file: " + GH_map_file;
                throw std::runtime_error(error_msg);
            }

            std::cout << "Opening file for GH weights reading in file: " << GH_map_file << std::endl;
            
            // Use cereal for deserialization
            cereal::BinaryInputArchive archive(ifs);
            archive(nodes_weights_map); // Read and deserialize into nodes_weights_map

        } catch (const std::exception& e) {
            std::cerr << "Standard exception: " << e.what() << std::endl;
        }

        _nodes_weights_map_pointer = std::make_shared<QuadratureWeightsMap>(nodes_weights_map);

        /// parameters
        int n_states = params.nt();
        int N = n_states - 1;
        const int dim_conf = 2;
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

        _cuda_ptr = std::make_shared<CudaOperation_PlanarPR>(CudaOperation_PlanarPR{params.sig_obs(), params.eps_sdf(), params.radius()});
        
        // Obtain the parameters from params and RobotSDF(PlanarPRSDFExample Here)
        double sig_obs = params.sig_obs(), eps_sdf = params.eps_sdf();
        double temperature = params.temperature();

        /// initial values
        VectorXd joint_init_theta{VectorXd::Zero(ndim)};
        VectorXd avg_vel{(goal_theta.segment(0, dim_conf) - start_theta.segment(0, dim_conf)) / params.total_time()};
        
        /// prior 
        double delt_t = params.total_time() / N;

        // Since it is LTI, the transition matrix is constant -> Easy to compute the K inverse
        VectorXd mu_prior{VectorXd::Zero(ndim)}; // Use start_theta and transition matrix to compute the prior
        MatrixXd precision_prior{MatrixXd::Zero(ndim, ndim)}; // First create a dense matrix then use sparseView() to convert it to sparse

        MatrixXd B_matrix{MatrixXd::Zero(dim_state * (n_states+1), dim_state * n_states)};
        MatrixXd Q_inverse{MatrixXd::Zero(dim_state * (n_states+1), dim_state * (n_states+1))};

        VectorXd mu_start = start_theta;
        mu_start.segment(dim_conf, dim_conf) = avg_vel;

        // Create the prior mean vector
        for(int i = 0; i < n_states; i++)
        {   
            VectorXd theta_i{start_theta + double(i) * (goal_theta - start_theta) / N};
            theta_i.segment(dim_conf, dim_conf) = avg_vel;

            // Only works in linear system, in nonlinear systems, we need to compute the transition matrix and gramian by ode solver.
            MatrixXd Phi_i = MatrixXd::Zero(dim_state, dim_state);
            Phi_i << MatrixXd::Identity(dim_conf, dim_conf), i*delt_t*MatrixXd::Identity(dim_conf, dim_conf), 
                    MatrixXd::Zero(dim_conf, dim_conf), MatrixXd::Identity(dim_conf, dim_conf);

            if(i == 0){
                // mu_prior.segment(i*dim_state, dim_state) = Phi_i * mu_start;
                mu_prior.segment(i*dim_state, dim_state) = theta_i;
            }
            else if (i == n_states-1){
                // mu_prior.segment(i*dim_state, dim_state) = Phi_i * mu_start;
                mu_prior.segment(i*dim_state, dim_state) = theta_i; // set the last state to be the goal state
            }
            else{
                // mu_prior.segment(i*dim_state, dim_state) = Phi_i * start_theta;
                mu_prior.segment(i*dim_state, dim_state) = Phi_i * mu_start;
            }
        }

        for(int i = 0; i < n_states; i++)
        {
            // initial state
            VectorXd theta_i{start_theta + double(i) * (goal_theta - start_theta) / N};
            theta_i.segment(dim_conf, dim_conf) = avg_vel;
            joint_init_theta.segment(i*dim_state, dim_state) = theta_i;

            // some problems with index in lin_gp, but it is not used in the code
            gvi::MinimumAccGP_proxkl lin_gp{Qc, max(0, i-1), delt_t, mu_prior};
            
            if(i == 0){
                B_matrix.block(0, 0, dim_state, dim_state) = MatrixXd::Identity(dim_state, dim_state);
                Q_inverse.block(0, 0, dim_state, dim_state) = K0_fixed.inverse();
            }
            else if (i == n_states-1){
                B_matrix.block(i*dim_state, i*dim_state, dim_state, dim_state) = MatrixXd::Identity(dim_state, dim_state);
                B_matrix.block((i+1)*dim_state, i*dim_state, dim_state, dim_state) = MatrixXd::Identity(dim_state, dim_state);
                B_matrix.block(i*dim_state, (i-1)*dim_state, dim_state, dim_state) = -lin_gp.Phi();

                Q_inverse.block(i*dim_state, i*dim_state, dim_state, dim_state) = lin_gp.get_precision();
                Q_inverse.block((i+1)*dim_state, (i+1)*dim_state, dim_state, dim_state) = K0_fixed.inverse();
            }
            else{
                B_matrix.block(i*dim_state, i*dim_state, dim_state, dim_state) = MatrixXd::Identity(dim_state, dim_state);
                B_matrix.block(i*dim_state, (i-1)*dim_state, dim_state, dim_state) = -lin_gp.Phi();

                Q_inverse.block(i*dim_state, i*dim_state, dim_state, dim_state) = lin_gp.get_precision();
            }

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
                gvi::FixedPriorGP_proxkl fixed_gp{K0_fixed, MatrixXd{mu_prior.segment(i*dim_state, dim_state)}};
                // gvi::FixedPriorGP fixed_gp{K0_fixed, MatrixXd{Phi_i * start_theta}};

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

                // collision factor (Runs in GPU)  //Robot -> 
                vec_factors.emplace_back(new ProxKLFactorizedBaseGH{dim_conf, 
                                                                dim_state, 
                                                                params.GH_degree(),
                                                                n_states, 
                                                                i, 
                                                                params.temperature(), 
                                                                params.high_temperature(),
                                                                _nodes_weights_map_pointer, 
                                                                _cuda_ptr});    
            }
        }

        // Already make sure that the prior precision is good
        precision_prior = B_matrix.transpose() * Q_inverse * B_matrix;

        /// The joint optimizer
        gvi::ProxKLGH<gvi::GVIFactorizedBase_Cuda> optimizer{vec_factors, 
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
        optimizer.set_alpha(params.alpha());
        
        optimizer.classify_factors();
        optimizer.set_prior(mu_prior, precision_prior.sparseView());

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
    std::shared_ptr<gvi::ProxKLGH<gvi::GVIFactorizedBase_Cuda>> _p_opt;
    std::shared_ptr<CudaOperation_PlanarPR> _cuda_ptr;

    std::tuple<Eigen::VectorXd, gvi::SpMat> _last_iteration_mean_precision;

    std::shared_ptr<QuadratureWeightsMap> _nodes_weights_map_pointer;

};

} // namespace vimp