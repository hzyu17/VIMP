/**
 * @file ProxKL3DPointRobotSDF.h
 * @author Zinuo Chang (zchang40@gatech.edu)
 * @brief The optimizer for 3D point robots at the joint level.
 * @version 0.1
 * @date 2025-03-28
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
using ProxKLFactorizedBaseGH = ProxKLFactorizedBaseGH_Cuda<CudaOperation_3dpR>;

class ProxKL3DPointRobotSDF{

public:
    virtual ~ProxKL3DPointRobotSDF(){}

    ProxKL3DPointRobotSDF(){}

    ProxKL3DPointRobotSDF(GVIMPParams& params){}

    double run_optimization_withtime(const GVIMPParams& params, bool verbose=true){
        _last_iteration_mean_precision = run_optimization_return(params, verbose);
        return 0;
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
            cereal::BinaryInputArchive archive(ifs); // Use cereal for deserialization
            archive(nodes_weights_map); // Read and deserialize into nodes_weights_map

        } catch (const std::exception& e) {
            std::cerr << "Standard exception: " << e.what() << std::endl;
        }

        _nodes_weights_map_pointer = std::make_shared<QuadratureWeightsMap>(nodes_weights_map);


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

        std::shared_ptr<GH> gh_ptr = std::make_shared<GH>(GH{params.GH_degree(), dim_conf, _nodes_weights_map_pointer});
        std::shared_ptr<CudaOperation_3dpR> cuda_ptr = std::make_shared<CudaOperation_3dpR>(CudaOperation_3dpR{params.sig_obs(), params.eps_sdf(), params.radius()});
        
        // Obtain the parameters from params and RobotSDF(PlanarPRSDFExample Here)
        double sig_obs = params.sig_obs(), eps_sdf = params.eps_sdf();
        double temperature = params.temperature();

        /// initial values
        VectorXd joint_init_theta{VectorXd::Zero(ndim)};
        VectorXd avg_vel{(goal_theta.segment(0, dim_conf) - start_theta.segment(0, dim_conf)) / params.total_time()};
        
        /// prior
        double delt_t = params.total_time() / N;

        VectorXd mu_prior{VectorXd::Zero(ndim)}; // Use start_theta and transition matrix to compute the prior
        MatrixXd precision_prior{MatrixXd::Zero(ndim, ndim)}; // First create a dense matrix then use sparseView() to convert it to sparse

        VectorXd mu_start = start_theta;
        mu_start.segment(dim_conf, dim_conf) = avg_vel;

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

        std::vector<MatrixXd> PhiList(N);
        std::vector<MatrixXd> QInvList(N);
        MatrixXd K0Inv = K0_fixed.inverse();
        MatrixXd KNInv = K0_fixed.inverse();

        for (int i = 0; i < n_states; i++) {

            // initial state
            VectorXd theta_i{start_theta + double(i) * (goal_theta - start_theta) / N};

            theta_i.segment(dim_conf, dim_conf) = avg_vel;
            joint_init_theta.segment(i*dim_state, dim_state) = theta_i;

            gvi::MinimumAccGP_proxkl lin_gp{Qc, max(0, i-1), delt_t, mu_prior};

            if (i > 0){
                PhiList[i - 1] = lin_gp.Phi(); // Store Phi for the previous state
                QInvList[i - 1] = lin_gp.get_precision(); // Store precision for the previous state
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

                vec_factors.emplace_back(new ProxKLFactorizedBaseGH{dim_conf,
                                                                dim_state,
                                                                params.GH_degree(),
                                                                n_states,
                                                                i,
                                                                params.temperature(),
                                                                params.high_temperature(),
                                                                _nodes_weights_map_pointer,
                                                                cuda_ptr});
            }
        }

        precision_prior = computePrecisionPriorExplicit(PhiList, QInvList, K0Inv, KNInv, N, dim_state);

        /// The joint optimizer
        gvi::ProxKLGH<gvi::GVIFactorizedBase_Cuda, CudaOperation_3dpR> optimizer{vec_factors,
                                                                            dim_state,
                                                                            n_states,
                                                                            cuda_ptr,
                                                                            gh_ptr,
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
        optimizer.time_test();

        // std::cout << "---------------- Start the optimization ----------------" << std::endl;
        // optimizer.optimize(verbose);

        _last_iteration_mean_precision = std::make_tuple(optimizer.mean(), optimizer.precision());

        return _last_iteration_mean_precision;

    }

    std::tuple<VectorXd, gvi::SpMat> get_mu_precision(){
        return _last_iteration_mean_precision;
    }

    MatrixXd computePrecisionPriorExplicit(
        const std::vector<MatrixXd>& PhiList,  // PhiList[k] corresponds to Φ(k,k-1), k=1..N
        const std::vector<MatrixXd>& QInvList, // QInvList[k] corresponds to Q⁻¹(k,k), k=1..N
        const MatrixXd& K0Inv,            // Q⁻¹(0,0)
        const MatrixXd& KNInv,            // Q⁻¹(N+1,N+1)
        int N,                            // Number of Gramians, i.e., the number of intermediate blocks. B has N+2 rows and N+1 columns
        int dim_state)
    {
        // precision_prior has dimensions ((N+1)*dim_state) x ((N+1)*dim_state)
        MatrixXd precision_prior = MatrixXd::Zero((N+1) * dim_state, (N+1) * dim_state);
    
        // -------------------------------
        // Row k = 0, corresponds to B(0,0)=I, Q⁻¹(0,0)=K0Inv
        // Only affects block (0,0)
        precision_prior.block(0, 0, dim_state, dim_state) += K0Inv;
    
        // -------------------------------
        // For k = 1,...,N
        // Each row k has two non-zero blocks:
        //   B(k, k-1) = -Φ(k,k-1) and B(k,k) = I, Q⁻¹(k,k)= Qk (inverse of Gramian)
        for (int k = 0; k < N; ++k)
        {
            // Let i = k-1, j = k
            int i = k;
            int j = k+1;
    
            // Current Q⁻¹ block (inverse of Gramian)
            const MatrixXd& Qk = QInvList[k]; // k ranges from 1 to N
    
            // Current Φ, representing Φ(k,k-1)
            const MatrixXd& Phi = PhiList[k];  // k ranges from 1 to N
    
            // For block (i, i): from B(k, i) = -Φ, contribution is (-Φ)ᵀ Qk (-Φ) = Φᵀ Qk Φ
            precision_prior.block(i * dim_state, i * dim_state, dim_state, dim_state) += Phi.transpose() * Qk * Phi;
    
            // For block (i, j): from (-Φ)ᵀ Qk I = -Φᵀ Qk
            MatrixXd PhiTQk = Phi.transpose() * Qk;
            precision_prior.block(i * dim_state, j * dim_state, dim_state, dim_state) = -PhiTQk;
            precision_prior.block(j * dim_state, i * dim_state, dim_state, dim_state) = -PhiTQk.transpose();
    
            // For block (j, j): from Iᵀ Qk I = Qk
            precision_prior.block(j * dim_state, j * dim_state, dim_state, dim_state) += Qk;
        }
    
        // -------------------------------
        // Row k = N+1, corresponds to B(N+1,N)=I, Q⁻¹(N+1,N+1)=KNInv
        // Only affects block (N, N)
        precision_prior.block(N * dim_state, N * dim_state, dim_state, dim_state) += KNInv;
    
        return precision_prior;
    }

protected:
    double _eps_sdf;
    double _sig_obs; // The inverse of Covariance matrix related to the obs penalty.
    gvi::EigenWrapper _ei;

    std::tuple<Eigen::VectorXd, gvi::SpMat> _last_iteration_mean_precision;

    std::shared_ptr<QuadratureWeightsMap> _nodes_weights_map_pointer;

};

} // namespace vimp