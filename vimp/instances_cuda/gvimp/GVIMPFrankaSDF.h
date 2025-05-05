/**
 * @file GVIMPFrankaSDF.h
 * @author Zinuo Chang (zchang40@gatech.edu)
 * @brief The optimizer for a PR2 Robot Arm at the joint level.
 * @version 0.1
 * @date 2025-04-30
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "helpers/timer.h"
#include "helpers/ExperimentParams.h"
#include "GaussianVI/gp/factorized_opts_linear_Cuda.h"
#include "GaussianVI/gp/cost_functions.h"
#include "GaussianVI/ngd/NGDFactorizedBaseGH_Cuda.h"
#include "GaussianVI/ngd/NGD-GH-Cuda.h"

std::string GH_map_file{source_root+"/GaussianVI/quadrature/SparseGHQuadratureWeights_cereal.bin"};

namespace vimp{

using GHFunction = std::function<MatrixXd(const VectorXd&)>;
using GH = SparseGaussHermite_Cuda<GHFunction>;

class GVIMPFrankaSDF{

public:
    virtual ~GVIMPFrankaSDF(){}

    GVIMPFrankaSDF(){}

    GVIMPFrankaSDF(GVIMPParams& params){}

    double run_optimization_withtime(const GVIMPParams& params, bool verbose=true){
        Timer timer;
        timer.start();
        _last_iteration_mean_precision = run_optimization_return(params, verbose);

        VectorXd trajectory;
        collision_checking_and_resampling(std::get<0>(_last_iteration_mean_precision), std::get<1>(_last_iteration_mean_precision), _sdf_file);

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

            // std::cout << "Opening file for GH weights reading in file: " << GH_map_file << std::endl;
            cereal::BinaryInputArchive archive(ifs);
            archive(nodes_weights_map);

        } catch (const std::exception& e) {
            std::cerr << "Standard exception: " << e.what() << std::endl;
        }

        _nodes_weights_map_pointer = std::make_shared<QuadratureWeightsMap>(nodes_weights_map);

        /// parameters
        int n_states = params.nt();
        int N = n_states - 1;
        const int dim_conf = 7;
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

        DHType dh_type = DHType::Modified;
        VectorXd a(8);
        a << 0.0, 0.0, 0.0, 0.0825, -0.0825, 0.0, 0.088, 0.0;
        VectorXd alpha(8);
        alpha << 0, -M_PI/2.0, M_PI/2.0, M_PI/2.0, -M_PI/2.0, M_PI/2.0, M_PI/2.0, 0.0;
        VectorXd d(8);
        d << 0.333, 0, 0.316, 0.0, 0.384, 0.0, 0.0, 0.107;
        VectorXd theta_bias(8);
        theta_bias << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -M_PI/4;

        VectorXd radii(20);
        radii << 0.1, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05,
                0.05, 0.03, 0.05, 0.05, 0.05, 0.03, 0.03, 0.03, 0.03, 0.03, 0.03;
        VectorXi frames(20);
        frames << 1, 1, 2, 2, 3, 3, 3, 4, 4, 5, 5, 5, 5, 7, 8, 8, 8, 8, 8, 8;
        MatrixXd centers(20, 3); // Each row is a center; transpose when inputting to CUDA
        centers <<  0.0, 0.0,  -0.3,
                    0.0, 0.0,  -0.1,
                    0.0, 0.0,  -0.07,
                    0.0, 0.0,   0.07,
                    0.0, 0.0,  -0.10,
                    0.0, 0.0,  -0.175,
                    0.0, 0.0,  -0.25,
                    0.0, 0.0,  -0.07,
                    0.0, 0.0,   0.07,
                    0.0, 0.0,  -0.30,
                    0.0, 0.08, -0.15,
                    0.0, 0.10,  0.00,
                    0.0, 0.00,  0.00,
                    0.0, 0.0,  -0.05,
                    0.0, 0.0,   0.00,
                    0.0, 0.06,  0.00,
                    0.0, -0.06, 0.00,
                    0.0, 0.0,   0.04,
                    0.0, 0.07,  0.04,
                    0.0, -0.07, 0.04;

        std::shared_ptr<GH> gh_ptr = std::make_shared<GH>(GH{params.GH_degree(), dim_conf, _nodes_weights_map_pointer});
        std::shared_ptr<CudaOperation_3dArm> cuda_ptr = std::make_shared<CudaOperation_3dArm>(CudaOperation_3dArm{a, alpha, d, theta_bias, radii, frames, centers.transpose(), params.sdf_file(), params.sig_obs(), params.eps_sdf(), dh_type});
        
        _sig_obs = params.sig_obs();
        _eps_sdf = params.eps_sdf();
        _dim_conf = dim_conf;
        _num_states = n_states;
        _sdf_file = params.sdf_file();

        double temperature = params.temperature();

        /// initial values
        VectorXd joint_init_theta{VectorXd::Zero(ndim)};
        VectorXd avg_vel{(goal_theta.segment(0, dim_conf) - start_theta.segment(0, dim_conf)) / params.total_time()};

        // std::vector<Point3> joints;
        // compute_joint_positions_cpu(start_theta.data(), 8, a.data(), alpha.data(), d.data(), theta_bias.data(), joints, frames, centers.transpose(), dh_type);
        // std::cout << "Start Theta: " << start_theta.transpose() << std::endl;
        // for (int i = 0; i < joints.size(); i++) {
        //     std::cout << "Joint " << i << ": " << joints[i].x << ", " << joints[i].y << ", " << joints[i].z << std::endl;
        // }
        
        /// prior
        double delt_t = params.total_time() / N;

        for (int i = 0; i < n_states; i++) {

            // initial state
            VectorXd theta_i{start_theta + double(i) * (goal_theta - start_theta) / N};

            // initial velocity: must have initial velocity for the fitst state??
            theta_i.segment(dim_conf, dim_conf) = avg_vel;
            joint_init_theta.segment(i*dim_state, dim_state) = std::move(theta_i);

            gvi::MinimumAccGP lin_gp{Qc, i, delt_t, start_theta};

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

                // collision factor (Runs in GPU)  //Robot ->
                vec_factors.emplace_back(new NGDFactorizedBaseGH_Cuda{dim_conf,
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
        gvi::NGDGH<gvi::GVIFactorizedBase_Cuda, CudaOperation_3dArm> optimizer{vec_factors,
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

        optimizer.set_step_size_base(params.step_size()); // a local optima
        optimizer.set_alpha(params.alpha());
        // optimizer.set_save_covariance(false);
        
        optimizer.classify_factors();

        std::cout << "---------------- Start the optimization ----------------" << std::endl;
        optimizer.optimize(verbose);

        _last_iteration_mean_precision = std::make_tuple(optimizer.mean(), optimizer.precision());

        return _last_iteration_mean_precision;

    }

    void collision_checking_and_resampling(const VectorXd& joint_mean, const SpMat& joint_covariance, const std::string& sdf_name){
        // Define the parameters for the robot arm
        DHType dh_type = DHType::Modified;
        VectorXd a(8);
        a << 0.0, 0.0, 0.0, 0.0825, -0.0825, 0.0, 0.088, 0.0;
        VectorXd alpha(8);
        alpha << 0, -M_PI/2.0, M_PI/2.0, M_PI/2.0, -M_PI/2.0, M_PI/2.0, M_PI/2.0, 0.0;
        VectorXd d(8);
        d << 0.333, 0, 0.316, 0.0, 0.384, 0.0, 0.0, 0.107;
        VectorXd theta_bias(8);
        theta_bias << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -M_PI/4;

        VectorXd radii(20);
        radii << 0.1, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05,
                0.05, 0.03, 0.05, 0.05, 0.05, 0.03, 0.03, 0.03, 0.03, 0.03, 0.03;
        VectorXi frames(20);
        frames << 1, 1, 2, 2, 3, 3, 3, 4, 4, 5, 5, 5, 5, 7, 8, 8, 8, 8, 8, 8;
        MatrixXd centers(20, 3); // Each row is a center; transpose when inputting to CUDA
        centers <<  0.0, 0.0,  -0.3,
                    0.0, 0.0,  -0.1,
                    0.0, 0.0,  -0.07,
                    0.0, 0.0,   0.07,
                    0.0, 0.0,  -0.10,
                    0.0, 0.0,  -0.175,
                    0.0, 0.0,  -0.25,
                    0.0, 0.0,  -0.07,
                    0.0, 0.0,   0.07,
                    0.0, 0.0,  -0.30,
                    0.0, 0.08, -0.15,
                    0.0, 0.10,  0.00,
                    0.0, 0.00,  0.00,
                    0.0, 0.0,  -0.05,
                    0.0, 0.0,   0.00,
                    0.0, 0.06,  0.00,
                    0.0, -0.06, 0.00,
                    0.0, 0.0,   0.04,
                    0.0, 0.07,  0.04,
                    0.0, -0.07, 0.04;

        std::shared_ptr<CudaOperation_3dArm> cuda_ptr = std::make_shared<CudaOperation_3dArm>(CudaOperation_3dArm{a, alpha, d, theta_bias, radii, frames, centers.transpose(), sdf_name, _sig_obs, _eps_sdf, dh_type});
        gvi::NGDGH<gvi::GVIFactorizedBase_Cuda, CudaOperation_3dArm> optimizer(_dim_conf, _num_states, cuda_ptr);

        optimizer.collision_checking_and_resampling(joint_mean, joint_covariance);

    }

    std::tuple<VectorXd, gvi::SpMat> get_mu_precision(){
        return _last_iteration_mean_precision;
    }

    // void identity(double* M) {
    //     M[0]  = 1; M[4]  = 0; M[8]  = 0; M[12] = 0;
    //     M[1]  = 0; M[5]  = 1; M[9]  = 0; M[13] = 0;
    //     M[2]  = 0; M[6]  = 0; M[10] = 1; M[14] = 0;
    //     M[3]  = 0; M[7]  = 0; M[11] = 0; M[15] = 1;
    // }

    // void mat_mul(const double* A, const double* B, double* C, const int dim) {
    //     for (int col = 0; col < dim; col++) {
    //         for (int row = 0; row < dim; row++) {
    //             double sum = 0.0;
    //             for (int k = 0; k < dim; k++) {
    //                 sum += A[row + k*dim] * B[k + col*dim];
    //             }
    //             C[row + col*dim] = sum;
    //         }
    //     }
    // }

    // void dh_matrix(int i, double theta, const double* a, const double* alpha, const double* d, double* mat) {
    //     double ct = cos(theta);
    //     double st = sin(theta);
    //     double ca = cos(alpha[i]);
    //     double sa = sin(alpha[i]);
        
    //     // DH Matrix
    //     mat[0]  = ct;        mat[4] = -st * ca;  mat[8]  = st * sa;   mat[12] = a[i] * ct;
    //     mat[1]  = st;        mat[5] = ct * ca;   mat[9]  = -ct * sa;  mat[13] = a[i] * st;
    //     mat[2]  = 0;         mat[6] = sa;        mat[10] = ca;        mat[14] = d[i];
    //     mat[3]  = 0;         mat[7] = 0;         mat[11] = 0;         mat[15] = 1;
    // }

    // inline void dh_matrix_modified(int i, double theta, const double* a, const double* alpha, const double* d, double* mat) const {
    //     // std::cout << "Using modified DH matrix" << std::endl;
    //     double ct = cos(theta);
    //     double st = sin(theta);
    //     double di = d[i];
    //     double ca = cos(alpha[i]);
    //     double sa = sin(alpha[i]);
    //     double ai = a[i];
    
    //     // Fill in column-major order for the modified DH convention:
    //     mat[0]  = ct;        mat[4]  = -st;       mat[8]  = 0;         mat[12] = ai;
    //     mat[1]  = st * ca;   mat[5]  = ct * ca;   mat[9]  = -sa;       mat[13] = -di * sa;
    //     mat[2]  = st * sa;   mat[6]  = ct * sa;   mat[10] = ca;        mat[14] = di * ca;
    //     mat[3]  = 0;         mat[7]  = 0;         mat[11] = 0;         mat[15] = 1;
    // }

    // void compute_joint_positions_cpu(const double* theta, int num_joints,
    //                                 const double* a, const double* alpha, const double* d, const double* theta_bias,
    //                                 std::vector<Point3>& joints, VectorXi& frames, MatrixXd centers, DHType dh_type = DHType::Classical) {
    //     int MATRIX_ELEMENTS = 16;
    //     double T[MATRIX_ELEMENTS];
    //     identity(T);
    //     std::vector<Matrix4d> dh_matrices;
    //     dh_matrices.clear();

    //     joints.clear();
    //     joints.push_back({ T[12], T[13], T[14] });

    //     // Sequentially compute the cumulative transformation for each joint and extract the translation part as the joint position
    //     for (int i = 0; i < num_joints; ++i) {
    //         double th = theta[i] + theta_bias[i];  // Consider joint bias
    //         double dh_mat[MATRIX_ELEMENTS];
    //         if (dh_type == DHType::Classical) {
    //             dh_matrix(i, th, a, alpha, d, dh_mat);
    //         } else {
    //             dh_matrix_modified(i, th, a, alpha, d, dh_mat);
    //         }

    //         double T_new[MATRIX_ELEMENTS];
    //         mat_mul(T, dh_mat, T_new, 4);

    //         // Update the cumulative transformation matrix T
    //         std::memcpy(T, T_new, sizeof(double)*MATRIX_ELEMENTS);

    //         // Use the translation part of the current cumulative transformation matrix as the joint position
    //         joints.push_back({ T[12], T[13], T[14] });

    //         dh_matrices.push_back(Matrix4d::Map(T_new));
    //     }

    //     // for (int i = 0; i < dh_matrices.size(); i++)
    //     // {
    //     //     Matrix4d T = dh_matrices[i];
    //     //     std::cout << "DH Matrix " << i << ": " << T << std::endl;
    //     // }        

    //     for (int i = 0; i < frames.size(); i++)
    //     {
    //         VectorXd center = centers.col(i);
    //         Matrix4d T = dh_matrices[frames[i]-1];
    //         Vector4d center_homogeneous(center(0), center(1), center(2), 1.0);
    //         Vector4d transformed_center = T * center_homogeneous;
    //         std::cout << "Transformed center" << i << ": " << transformed_center.transpose().segment(0,3) << std::endl;
    //     }
    // }

protected:
    double _eps_sdf;
    double _sig_obs; // The inverse of the covariance matrix related to the observation penalty.
    int _num_states;
    int _dim_conf;
    std::string _sdf_file;
    gvi::EigenWrapper _ei;

    std::tuple<Eigen::VectorXd, gvi::SpMat> _last_iteration_mean_precision;

    std::shared_ptr<QuadratureWeightsMap> _nodes_weights_map_pointer;

};

} // namespace vimp