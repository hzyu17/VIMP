/**
 * @file ProxKLPlanarQuadrotorSDF.h
 * @author Zinuo Chang (zchang40@gatech.edu)
 * @brief The optimizer for planar quadrotor at the joint level.
 * @version 0.1
 * @date 2025-01-27
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
#include "gp/LTV_prior.h"

#include "dynamics/PlanarQuad_linearization.h"
#include "dynamics/PlanarQuad_SLR.h"
#include <unsupported/Eigen/MatrixFunctions>

namespace gvi{
    using FixedGpPrior = NGDFactorizedLinear_Cuda<FixedPriorGP_proxkl>;
    using LinearGpPrior = NGDFactorizedLinear_Cuda<LTV_GP>;

    double cost_fixed_gp(const VectorXd& x, const gvi::FixedPriorGP_proxkl& fixed_gp){
        return fixed_gp.fixed_factor_cost(x);
    }

    double cost_linear_gp(const VectorXd& pose_cmb, const gvi::LTV_GP& gp_minacc){
        int dim = gp_minacc.dim_posvel();
        return gp_minacc.cost(pose_cmb.segment(0, dim), pose_cmb.segment(dim, dim));
    }
}

std::string GH_map_file{source_root+"/GaussianVI/quadrature/SparseGHQuadratureWeights.bin"};
std::string GH_map_file_cereal{source_root+"/GaussianVI/quadrature/SparseGHQuadratureWeights_cereal.bin"};

namespace vimp{

using GHFunction = std::function<MatrixXd(const VectorXd&)>;
using GH = SparseGaussHermite_Cuda<GHFunction>;
using ProxKLFactorizedBaseGH = ProxKLFactorizedBaseGH_Cuda<CudaOperation_Quad>;

class GVIMPPlanarQuadrotorSDF{

public:
    virtual ~GVIMPPlanarQuadrotorSDF(){}

    GVIMPPlanarQuadrotorSDF(){}

    GVIMPPlanarQuadrotorSDF(GVIMPParams& params){}

    double run_optimization_withtime(const GVIMPParams& params, bool verbose=true){
        Timer timer;
        timer.start();

        int n_iter = 1;
        int n_states = params.nt();
        const int dim_conf = 3;
        double delt_t = params.total_time() / (n_states - 1);

        VectorXd start_theta{ params.m0() };
        VectorXd goal_theta{ params.mT() };

        MatrixXd trajectory(n_states, 2*dim_conf);
        MatrixXd new_trajectory(n_states, 2*dim_conf);
        MatrixXd traj_diff(n_states, dim_conf);

        new_trajectory.setZero();
        traj_diff.setZero();

        VectorXd avg_vel{(goal_theta.segment(0, dim_conf) - start_theta.segment(0, dim_conf)) / params.total_time()};

        for (int i = 0; i < n_states; i++) {
            VectorXd theta{start_theta + double(i) * (goal_theta - start_theta) / (n_states - 1)};
            theta.segment(dim_conf, dim_conf) = avg_vel;
            trajectory.row(i) = theta.transpose();
        }

        MatrixXd dense_covariance = MatrixXd::Identity(2 * dim_conf * n_states, 2 * dim_conf * n_states) / params.initial_precision_factor();
        SpMat covariance = dense_covariance.sparseView();

        for (int i = 0; i < n_iter; i++){
            _last_iteration_mean_precision = run_optimization_return(params, trajectory, covariance, verbose);
            VectorXd mean = std::get<0>(_last_iteration_mean_precision);
            new_trajectory = Eigen::Map<Eigen::MatrixXd>(mean.data(), 2*dim_conf, n_states).transpose();
            
            // Compute the difference between the new trajectory and the old trajectory
            double difference = 0;
            for (int j = 0; j < n_states; j++)
                difference += (new_trajectory.row(j).segment(0,3) - trajectory.row(j).segment(0,3)).norm();

            std::cout << "Norm of difference between two trajectories = " << difference << std::endl;

            trajectory = new_trajectory;
            covariance = std::get<1>(_last_iteration_mean_precision); // Fix this, the matrix here is precision, not covariance
        }
        

        std::cout << "========== Optimization time: " << std::endl;
        return timer.end_sec();
    }

    // void run_optimization(const GVIMPParams& params, bool verbose=true){
    //     _last_iteration_mean_precision = run_optimization_return(params, verbose);
    // }

    std::tuple<Eigen::VectorXd, gvi::SpMat> run_optimization_return(const GVIMPParams& params, const MatrixXd& traj, const SpMat& cov, bool verbose=true){
        QuadratureWeightsMap nodes_weights_map;
        try {
            std::ifstream ifs(GH_map_file_cereal, std::ios::binary);
            if (!ifs.is_open()) {
                std::string error_msg = "Failed to open file for GH weights reading in file: " + GH_map_file_cereal;
                throw std::runtime_error(error_msg);
            }

            std::cout << "Opening file for GH weights reading in file: " << GH_map_file_cereal << std::endl;
            
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
        std::shared_ptr<CudaOperation_Quad> cuda_ptr = std::make_shared<CudaOperation_Quad>(CudaOperation_Quad{params.sig_obs(), params.eps_sdf(), params.radius()});

        std::shared_ptr<GH> gh_linearization = std::make_shared<GH>(GH{params.GH_degree(), dim_state, _nodes_weights_map_pointer});
        
        // Obtain the parameters from params
        double sig_obs = params.sig_obs(), eps_sdf = params.eps_sdf();
        double temperature = params.temperature();

        /// initial values
        VectorXd joint_init_theta{VectorXd::Zero(ndim)};
        
        /// prior 
        double delt_t = params.total_time() / N;

        // Initialize the trajectory
        std::vector<MatrixXd> hA(4 * N + 1);
        std::vector<MatrixXd> hB(4 * N + 1);
        std::vector<VectorXd> ha(4 * N + 1);
        std::vector<VectorXd> target_mean(n_states);

        MatrixXd trajectory = trajctory_interpolation(traj, target_mean); // the traj here is the prior mean
        std::vector<MatrixXd> covariances = covariance_interpolation(cov, dim_state, n_states);        

        auto linearized_matrices = planarquad_linearization_deterministic(trajectory);
        hA = std::get<0>(linearized_matrices);
        hB = std::get<1>(linearized_matrices);
        ha = std::get<2>(linearized_matrices);

        auto SLR_matrices = linearization_SLR(trajectory, covariances, gh_linearization);
        std::vector<MatrixXd> hA_SLR = std::get<0>(SLR_matrices);
        std::vector<VectorXd> ha_SLR = std::get<1>(SLR_matrices);
        hA = hA_SLR;
        ha = ha_SLR;

        MatrixXd trajectory_init(traj.rows(), traj.cols());
        trajectory_init.setZero();

        // Go Through
        trajectory_init = traj;

        VectorXd mu_prior{VectorXd::Zero(ndim)}; // Use start_theta and transition matrix to compute the prior
        MatrixXd precision_prior{MatrixXd::Zero(ndim, ndim)}; // First create a dense matrix then use sparseView() to convert it to sparse

        MatrixXd B_matrix{MatrixXd::Zero(dim_state * (n_states+1), dim_state * n_states)};
        MatrixXd Q_inverse{MatrixXd::Zero(dim_state * (n_states+1), dim_state * (n_states+1))};

        // create each factor
        // fill the B and Q_inverse matrix in each iteration
        for (int i = 0; i < n_states; i++) {

            // initial state                     
            joint_init_theta.segment(i*dim_state, dim_state) = trajectory_init.row(i);
            gvi::LTV_GP lin_gp{Qc, max(0, i-1), delt_t, start_theta, n_states, hA, hB, target_mean};

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
                gvi::FixedPriorGP_proxkl fixed_gp{K0_fixed, MatrixXd{target_mean[i]}};
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

        mu_prior = traj.transpose().reshaped();
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
        optimizer.set_prior(mu_prior, precision_prior.sparseView()); // Set the prior mean with trajectory

        std::cout << "---------------- Start the optimization ----------------" << std::endl;
        optimizer.optimize(verbose);

        _last_iteration_mean_precision = std::make_tuple(optimizer.mean(), optimizer.covariance());

        return _last_iteration_mean_precision;

    }

    std::tuple<VectorXd, gvi::SpMat> get_mu_precision(){
        return _last_iteration_mean_precision;
    }

    MatrixXd trajctory_interpolation(const MatrixXd& trajectory, std::vector<VectorXd>& target_mean){
        int N = trajectory.rows() - 1;
        int dim_state = trajectory.cols();
        MatrixXd traj(4*N+1, dim_state);

        for (int i = 0; i < N; i++) {
            traj.row(4*i) = trajectory.row(i);
            traj.row(4*i+1) = (3 * trajectory.row(i) + trajectory.row(i+1)) * 0.25;
            traj.row(4*i+2) = (trajectory.row(i) + trajectory.row(i+1)) * 0.5;
            traj.row(4*i+3) = (trajectory.row(i) + 3 * trajectory.row(i+1)) * 0.25;
            target_mean[i] = trajectory.row(i).transpose();
        }

        traj.row(4 * N) = trajectory.row(N);
        target_mean[N] = trajectory.row(N).transpose();

        return traj;
    }


    std::vector<MatrixXd> covariance_interpolation(const SpMat& covariance, int dim_state, int n_states) {
        int N = n_states - 1;
        std::vector<MatrixXd> interpolated_covariances(4 * N + 1);

        for (int i = 0; i < N; i++) {
            MatrixXd Sigma1 = covariance.block(i*dim_state, i*dim_state, dim_state, dim_state);
            MatrixXd Sigma2 = covariance.block((i+1)*dim_state, (i+1)*dim_state, dim_state, dim_state);

            // Compute the logarithm of the covariance matrices
            MatrixXd log_Sigma1 = Sigma1.log();
            MatrixXd log_Sigma2 = Sigma2.log();

            // Interpolate between the covariance matrices
            interpolated_covariances[4 * i] = Sigma1;
            interpolated_covariances[4 * i + 1] = (0.75*log_Sigma1 + 0.25*log_Sigma2).exp();
            interpolated_covariances[4 * i + 2] = (0.5*log_Sigma1 + 0.5*log_Sigma2).exp();
            interpolated_covariances[4 * i + 3] = (0.25*log_Sigma1 + 0.75*log_Sigma2).exp();

            // std::cout << "Norm of the differences: " << (Sigma1 - interpolated_covariances[4 * i]).norm() << ", " <<  (Sigma1 - interpolated_covariances[4 * i + 1]).norm() << ", " <<  (Sigma1 - interpolated_covariances[4 * i + 2]).norm()
            // << ", " <<  (Sigma1 - interpolated_covariances[4 * i + 3]).norm() << ", " <<  (Sigma1 - Sigma2).norm() << std::endl;
        }

        // Append the last covariance matrix
        interpolated_covariances[4 * N] = covariance.block(N * dim_state, N * dim_state, dim_state, dim_state);

        return interpolated_covariances;
    }


protected: 
    double _eps_sdf;
    double _sig_obs; // The inverse of Covariance matrix related to the obs penalty. 
    gvi::EigenWrapper _ei;
    std::shared_ptr<gvi::ProxKLGH<gvi::GVIFactorizedBase_Cuda>> _p_opt;

    std::tuple<Eigen::VectorXd, gvi::SpMat> _last_iteration_mean_precision;

    std::shared_ptr<QuadratureWeightsMap> _nodes_weights_map_pointer;

};

} // namespace vimp