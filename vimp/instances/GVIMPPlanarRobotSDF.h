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

#include "helpers/ExperimentParams.h"
#include "instances/PlanarFactorPR.h"
#include "robots/PlanarPointRobotSDF_pgcs.h"

namespace vimp{

using SDFPR = gpmp2::ObstaclePlanarSDFFactorPointRobot;

template <typename Robot, typename RobotSDF>
class GVIMPPlanarRobotSDF{
public:
    virtual ~GVIMPPlanarRobotSDF(){}
    GVIMPPlanarRobotSDF(){}
    GVIMPPlanarRobotSDF(GVIMPParams& params):
    _robot_sdf(params.eps_sdf(), params.radius(), params.map_name(), params.sdf_file())
    {}

    void run_optimization(const GVIMPParams& params){
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
        vector<std::shared_ptr<GVIFactorizedBase>> vec_factors;
        
        auto robot_model = _robot_sdf.RobotModel();
        auto sdf = _robot_sdf.sdf();
        double sig_obs = params.sig_obs(), eps_sdf = params.eps_sdf();
        double temperature = params.temperature();

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

        /// The joint optimizer
        GVIGH<GVIFactorizedBase> optimizer{vec_factors, 
                                           dim_state, 
                                           n_states, 
                                           params.max_iter(), 
                                           temperature, 
                                           params.high_temperature()};

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
        optimizer.optimize();

    }

protected: 
    RobotSDF _robot_sdf;
    double _eps_sdf;
    double _sig_obs; // The inverse of Covariance matrix related to the obs penalty. 
    EigenWrapper _ei;

};

}