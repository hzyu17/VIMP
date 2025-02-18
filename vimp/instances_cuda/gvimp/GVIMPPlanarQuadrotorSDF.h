/**
 * @file GVIMPPlanarQuadrotorSDF.h
 * @author Zinuo Chang (zchang40@gatech.edu)
 * @brief The optimizer for planar quadrotor at the joint level.
 * @version 0.1
 * @date 2024-09-04
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "helpers/timer.h"
#include "helpers/ExperimentParams.h"
#include "GaussianVI/gp/factorized_opts_LTV.h"
#include "GaussianVI/gp/cost_functions_LTV.h"
#include "GaussianVI/ngd/NGDFactorizedBaseGH_Cuda.h"
#include "GaussianVI/ngd/NGD-GH-Cuda.h"
#include "dynamics/PlanarQuad_linearization.h"
#include "dynamics/PlanarQuad_SLR.h"
#include <unsupported/Eigen/MatrixFunctions>

#include <vector>
#include <fstream>

std::string GH_map_file{source_root+"/GaussianVI/quadrature/SparseGHQuadratureWeights.bin"};
std::string GH_map_file_cereal{source_root+"/GaussianVI/quadrature/SparseGHQuadratureWeights_cereal.bin"};

namespace vimp{

using GHFunction = std::function<MatrixXd(const VectorXd&)>;
using GH = SparseGaussHermite_Cuda<GHFunction>;
using NGDFactorizedBaseGH = NGDFactorizedBaseGH_Cuda<CudaOperation_Quad>;

class GVIMPPlanarQuadrotorSDF{

public:
    virtual ~GVIMPPlanarQuadrotorSDF(){}

    GVIMPPlanarQuadrotorSDF(){}

    GVIMPPlanarQuadrotorSDF(GVIMPParams_nonlinear& params){}

    double run_optimization_withtime(const GVIMPParams_nonlinear& params, bool verbose=true){
        Timer timer;
        timer.start();

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


        int n_iter = params.max_linear_iter();
        int n_states = params.nt();
        const int dim_conf = 3;
        double delt_t = params.total_time() / (n_states - 1);

        VectorXd start_theta{ params.m0() };
        VectorXd goal_theta{ params.mT() };

        // VectorXd inter_theta(6);
        // inter_theta << 30, 5, -0.19634954084, 0.5, 1, 0.005;

        MatrixXd trajectory(n_states, 2*dim_conf);
        MatrixXd new_trajectory(n_states, 2*dim_conf);
        MatrixXd traj_diff(n_states, dim_conf);

        new_trajectory.setZero();
        traj_diff.setZero();

        VectorXd avg_vel{(goal_theta.segment(0, dim_conf) - start_theta.segment(0, dim_conf)) / params.total_time()};

        // for (int i = 0; i < n_states; i++) {
        //     VectorXd theta(6);
        //     theta.setZero();
        //     if (i < n_states / 2){
        //         theta = start_theta + double(i) * (inter_theta - start_theta) / (n_states / 2 - 1);
        //         // avg_vel = (inter_theta.segment(0, dim_conf) - start_theta.segment(0, dim_conf)) / ((n_states / 2 - 1) * delt_t);
        //     }
        //     else{
        //         theta = inter_theta + double(i - n_states/2 + 1) * (goal_theta - inter_theta) / (n_states / 2);
        //         // avg_vel = (goal_theta.segment(0, dim_conf) - inter_theta.segment(0, dim_conf)) / ((n_states / 2) * delt_t);
        //     }
        //     theta.segment(dim_conf, dim_conf) = avg_vel;
        //     trajectory.row(i) = theta.transpose();
        // }

        for (int i = 0; i < n_states; i++) {
            VectorXd theta{start_theta + double(i) * (goal_theta - start_theta) / (n_states - 1)};
            theta.segment(dim_conf, dim_conf) = avg_vel;
            trajectory.row(i) = theta.transpose();
        }

        MatrixXd dense_covariance = MatrixXd::Identity(2 * dim_conf * n_states, 2 * dim_conf * n_states) / params.initial_precision_factor();
        SpMat covariance = dense_covariance.sparseView();

        std::vector<double> norm_differences;

        for (int i = 0; i < n_iter; i++){
            bool is_final_iter = (i == n_iter - 1);
            _last_iteration_mean_precision = run_optimization_return(params, trajectory, covariance, is_final_iter, verbose);
            VectorXd mean = std::get<0>(_last_iteration_mean_precision);
            new_trajectory = Eigen::Map<Eigen::MatrixXd>(mean.data(), 2*dim_conf, n_states).transpose();
            
            // Compute the difference between the new trajectory and the old trajectory
            double difference = 0;
            for (int j = 0; j < n_states; j++)
                difference += (new_trajectory.row(j).segment(0,3) - trajectory.row(j).segment(0,3)).norm();

            std::cout << "Norm of difference between two trajectories = " << difference << std::endl;

            norm_differences.push_back(difference);

            trajectory = new_trajectory;
            covariance = std::get<1>(_last_iteration_mean_precision);
        }
        
        // // Save the norm differences to a CSV file
        // std::ofstream file("norm_differences.csv");
        // if (file.is_open()) {
        //     for (size_t i = 0; i < norm_differences.size(); i++){
        //         file << norm_differences[i];
        //         if (i != norm_differences.size() - 1)
        //             file << "\n";  // Newline for each entry
        //     }
        //     file.close();
        //     std::cout << "Norm differences saved to norm_differences.csv" << std::endl;
        // }
        // else {
        //     std::cerr << "Could not open file to write norm differences." << std::endl;
        // }

        std::cout << "========== Optimization time: " << std::endl;
        return timer.end_sec();
    }

    // void run_optimization(const GVIMPParams_nonlinear& params, bool verbose=true){
    //     _last_iteration_mean_precision = run_optimization_return(params, verbose);
    // }

    std::tuple<Eigen::VectorXd, gvi::SpMat> run_optimization_return(const GVIMPParams_nonlinear& params, const MatrixXd& traj, const SpMat& cov, const bool final_iter, bool verbose=true){
        
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

        MatrixXd trajectory = trajctory_interpolation(traj, target_mean);
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

        // for (int i = 0; i < 4*N+1; i++){
        //     std::cout << "i: " << i << std::endl;
        //     // std::cout << "hA: " << std::endl << hA[i] << std::endl;
        //     // std::cout << "hA_SLR: " << std::endl << hA_SLR[i] << std::endl;
        //     // std::cout << "hA_error: " << (hA[i] - hA_SLR[i]) << std::endl;
        //     std::cout << "norm of hA and ha: " << hA[i].norm() << ", " << ha[i].norm() << std::endl;
        //     std::cout << "hA_error_norm: " << (hA[i] - hA_SLR[i]).norm() << std::endl;
        //     std::cout << "ha_error_norm: " << (ha[i] - ha_SLR[i]).norm() << std::endl;
        // }

        MatrixXd trajectory_init(traj.rows(), traj.cols());
        trajectory_init.setZero();

        // // Go Around
        // VectorXd inter_theta(6);
        // inter_theta << 20, 10, -0.19634954084, 0.5, 1, 0.005;
        // VectorXd avg_vel{(goal_theta.segment(0, dim_conf) - start_theta.segment(0, dim_conf)) / params.total_time()};

        // for (int i = 0; i < n_states; i++) {
        //     VectorXd theta(6);
        //     theta.setZero();
        //     if (i < n_states / 2)
        //         theta = start_theta + double(i) * (inter_theta - start_theta) / (n_states / 2 - 1);
        //     else
        //         theta = inter_theta + double(i - n_states/2 + 1) * (goal_theta - inter_theta) / (n_states / 2);

        //     theta.segment(dim_conf, dim_conf) = avg_vel;
        //     trajectory_init.row(i) = theta.transpose();
        // }

        // Go Through
        trajectory_init = traj;

        // create each factor
        for (int i = 0; i < n_states; i++) {

            // initial state                     
            joint_init_theta.segment(i*dim_state, dim_state) = trajectory_init.row(i);
            gvi::LTV_GP lin_gp{Qc, max(0, i-1), delt_t, start_theta, n_states, hA, hB, target_mean}; // Check if it affects the result

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
                gvi::FixedPriorGP fixed_gp{K0_fixed, MatrixXd{target_mean[i]}};
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
                vec_factors.emplace_back(new NGDFactorizedBaseGH{dim_conf, 
                                                                dim_state, 
                                                                params.GH_degree(),
                                                                n_states, 
                                                                i, 
                                                                params.sig_obs(), 
                                                                params.eps_sdf(), 
                                                                params.radius(), 
                                                                params.temperature(), 
                                                                params.high_temperature(),
                                                                _nodes_weights_map_pointer, 
                                                                cuda_ptr});    
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
        optimizer.set_alpha(params.alpha());
        optimizer.set_data_save(final_iter);
        
        optimizer.classify_factors();

        std::cout << "---------------- Start the optimization ----------------" << std::endl;
        optimizer.optimize(verbose);

        _last_iteration_mean_precision = std::make_tuple(optimizer.mean(), optimizer.covariance());

        // MatrixXd covariances = optimizer.precision().toDense();
        // std::vector<MatrixXd> control_gain(n_states-1);
        // std::vector<MatrixXd> states(n_states);
        // std::vector<MatrixXd> control(n_states - 1);

        // // const int dim_conf = 3;
        // // const int dim_state = 2 * dim_conf; 

        // for (int i = 0; i < n_states-1; i++){
        //     MatrixXd cov_i = covariances.block(i*dim_state, i*dim_state, dim_state, dim_state);
        //     MatrixXd cov_j_i = covariances.block((i+1)*dim_state, i*dim_state, dim_state, dim_state);
        //     MatrixXd control_gain_i = (cov_j_i * cov_i.inverse() - MatrixXd::Identity(dim_state, dim_state)) / delt_t - hA[4*i];
        //     control_gain_i = hB[4*i].transpose() * control_gain_i;
        //     control_gain[i] = control_gain_i;

        //     MatrixXd cov_i_j = cov_i * ((hA[4*i] + hB[4*i] * control_gain_i) * delt_t + MatrixXd::Identity(dim_state, dim_state)).transpose();

        //     std::cout << "cov_i_j: " << std::endl << cov_i_j << std::endl;
        //     std::cout << "cov_i_j: " << std::endl << cov_j_i.transpose() << std::endl;

        //     // std::cout << "Error: " << error.norm() << std::endl;
        //     // std::cout << "B*B^T: " << hB[4*i].transpose() * hB[4*i] << std::endl;
        // }

        // VectorXd initial_mean = optimizer.mean().segment(0, dim_state);
        // MatrixXd initial_cov = covariances.block(0, 0, dim_state, dim_state);

        // // Cholesky decomposition to get lower triangular matrix L
        // Eigen::MatrixXd L = initial_cov.llt().matrixL();

        // // Create random number generator for standard normal distribution
        // std::random_device rd;
        // std::mt19937 gen(rd());
        // std::normal_distribution<> dist(0.0, 1.0);

        // // Generate a standard normal vector z
        // Eigen::VectorXd z(6);
        // for (int i = 0; i < z.size(); ++i) {
        //     z(i) = dist(gen);
        // }

        // // Transform to get multivariate Gaussian sample
        // Eigen::VectorXd x = initial_mean + L * z;
        // states[0] = initial_mean;

        // for (int i = 0; i < n_states-1; i++){
        //     states[i+1] = states[i] + ((hA[4*i]+ hB[4*i] * control_gain[i]) * states[i] + ha[4*i]) * delt_t;
        //     // Eigen::VectorXd noise(2);
        //     // for (int i = 0; i < noise.size(); ++i) {
        //     //     noise(i) = dist(gen);
        //     // }
        //     // states[i+1] = states[i+1] + hB[4*i] * sqrt(delt_t) * noise;
        //     control[i] = control_gain[i] * states[i];
        // }
        // MatrixXd states_matrix(states[0].rows(), states.size() * states[0].cols());
        // MatrixXd control_matrix(control[0].rows(), control.size() * control[0].cols());

        // for (int i = 0; i < n_states; i++){
        //     states_matrix.block(0, i*states[0].cols(), states[0].rows(), states[0].cols()) = states[i];
        // }

        // for (int i = 0; i < n_states - 1; i++){
        //     control_matrix.block(0, i*control[0].cols(), control[0].rows(), control[0].cols()) = control[i];

        // }


        // MatrixIO m_io;
        // std::string file_samples{"../matlab_helpers/GVIMP-examples/2d_Quad/case1/samples.csv"};
        // std::string file_controls{"../matlab_helpers/GVIMP-examples/2d_Quad/case1/controls.csv"};

        // m_io.saveData(file_samples, states_matrix);
        // m_io.saveData(file_controls, control_matrix);

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
    std::shared_ptr<gvi::NGDGH<gvi::GVIFactorizedBase_Cuda>> _p_opt;

    std::tuple<Eigen::VectorXd, gvi::SpMat> _last_iteration_mean_precision;

    std::shared_ptr<QuadratureWeightsMap> _nodes_weights_map_pointer;

};

} // namespace vimp