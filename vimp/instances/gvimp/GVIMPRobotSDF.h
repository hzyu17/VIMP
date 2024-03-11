/**
 * @file GVIMPRobotSDF.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief The optimizer for robots and 3D workspace SDF at the joint level.
 * @version 0.1
 * @date 2023-08-14
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "helpers/ExperimentParams.h"
#include "instances/FactorizedGVI.h"
#include <gpmp2/obstacle/ObstacleSDFFactor.h>
#include <gtsam/inference/Symbol.h>

namespace vimp{

template <typename Robot, typename RobotSDF>
class GVIMPRobotSDF{
    using SDFPR = gpmp2::ObstacleSDFFactor<Robot>;
    using GVIFactorizedSDFRobot = GVIFactorizedSDF<Robot>;

public:
    virtual ~GVIMPRobotSDF(){}

    GVIMPRobotSDF(){}

    GVIMPRobotSDF(GVIMPParams& params):
    _robot_sdf(params.eps_sdf(), params.radius(), params.map_name(), params.sdf_file())
    {}

    RobotSDF robot_sdf(){
        return _robot_sdf;
    }

    void run_optimization(const GVIMPParams& params, bool verbose=true){
        _last_iteration_mean_precision = run_optimization_return(params, verbose);
    }

    std::tuple<Eigen::VectorXd, SpMat> run_optimization_return(const GVIMPParams& params, bool verbose=true){
        /// parameters
        int n_states = params.nt();
        int N = n_states - 1;
        const int dim_conf = _robot_sdf.ndof() * _robot_sdf.nlinks();
        // state: theta = [conf, vel_conf]
        const int dim_state = 2 * dim_conf; 
        /// joint dimension
        const int ndim = dim_state * n_states;

        VectorXd start_theta{ params.m0() };
        VectorXd goal_theta{ params.mT() };

        MatrixXd Qc{MatrixXd::Identity(dim_conf, dim_conf)*params.coeff_Qc()};
        MatrixXd K0_fixed{MatrixXd::Identity(dim_state, dim_state)/params.boundary_penalties()};

        /// Vector of base factored optimizers
        vector<std::shared_ptr<gvi::GVIFactorizedBase>> vec_factors;
        
        auto robot_model = _robot_sdf.RobotModel();
        auto sdf = _robot_sdf.sdf();
        double sig_obs = params.sig_obs(), eps_sdf = params.eps_sdf();
        double temperature = params.temperature(), high_temperature = params.high_temperature();

        /// initial values
        VectorXd joint_init_theta{VectorXd::Zero(ndim)};
        VectorXd avg_vel{(goal_theta.segment(0, dim_conf) - start_theta.segment(0, dim_conf)) / params.total_time()};
        
        /// prior 
        double delt_t = params.total_time() / N;

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
                    // std::shared_ptr<gvi::LinearGpPrior> p_lin_gp{}; 
                    vec_factors.emplace_back(new gvi::LinearGpPrior{2*dim_state, 
                                                                dim_state, 
                                                                // params.GH_degree(),
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
                                                        //   params.GH_degree(),
                                                          cost_fixed_gp, 
                                                          fixed_gp, 
                                                          n_states, 
                                                          i,
                                                          params.temperature(), 
                                                          params.high_temperature()});

            }else{
                // linear gp factors
                vec_factors.emplace_back(new gvi::LinearGpPrior{2*dim_state, 
                                                            dim_state, 
                                                            // params.GH_degree(),
                                                            cost_linear_gp, 
                                                            lin_gp, 
                                                            n_states, 
                                                            i-1, 
                                                            params.temperature(), 
                                                            params.high_temperature()});

                // collision factor
                // auto cost_sdf_Robot = cost_obstacle<Robot>;
                vec_factors.emplace_back(new GVIFactorizedSDFRobot{dim_conf, 
                                                                    dim_state, 
                                                                    params.GH_degree(),
                                                                    cost_obstacle<Robot>, 
                                                                    SDFPR{gtsam::symbol('x', i), 
                                                                    robot_model, 
                                                                    sdf, 
                                                                    1.0/sig_obs, 
                                                                    eps_sdf}, 
                                                                    n_states, 
                                                                    i, 
                                                                    temperature, 
                                                                    high_temperature});    
            }
        }

        /// The joint optimizer
        gvi::NGDGH<gvi::GVIFactorizedBase> optimizer{vec_factors, 
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

        #include "helpers/timer.h"
        Timer timer;
        timer.start();
        optimizer.optimize(verbose);
        std::cout << "========== Optimization time: " << std::endl;
        timer.end_mus();
        _last_iteration_mean_precision = std::make_tuple(optimizer.mean(), optimizer.precision());

        return _last_iteration_mean_precision;

    }

    std::tuple<VectorXd, SpMat> get_mu_precision(){
        return _last_iteration_mean_precision;
    }

protected: 
    RobotSDF _robot_sdf;
    double _eps_sdf;
    double _sig_obs; // The inverse of Covariance matrix related to the obs penalty. 
    EigenWrapper _ei;
    std::shared_ptr<gvi::NGDGH<gvi::GVIFactorizedBase>> _p_opt;

    std::tuple<Eigen::VectorXd, SpMat> _last_iteration_mean_precision;

};

} // namespace vimp