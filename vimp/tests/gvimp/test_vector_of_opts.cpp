/**
 * @file test_vector_of_opts.cpp
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Test the vector of different types of cost evaluators (factorized optimizers).
 * @version 0.1
 * @date 2022-07-31
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "GaussianVI/gp/fixed_prior.h"
#include "GaussianVI/gp/minimum_acc_prior.h"
#include "GaussianVI/ngd/NGDFactorizedBase.h"
#include "robots/PlanarPointRobotSDFExample.h"
#include "GaussianVI/ngd/NGD-GH.h"
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorPointRobot.h>

#include <gtest/gtest.h>

using namespace vimp;
using namespace Eigen;

namespace vimp{
    template <typename Cl1>
    class VIMPFactorizedOneCost : public gvi::NGDFactorizedBase{
        using Base = gvi::NGDFactorizedBase;
        using GHFunction = std::function<MatrixXd(const VectorXd&)>;
        using Function = std::function<double(const VectorXd&, const Cl1&)>;
        // using GH = gvi::SparseGaussHermite<GHFunction>;
        using GH = gvi::GaussHermite<GHFunction>;
        public:
            VIMPFactorizedOneCost(const int& dimension, 
                                    int state_dim, 
                                    int num_states,
                                    int start_indx,
                                    double temperature,
                                    double high_temperature,
                                    const Function& function, 
                                    const Cl1& cost_class,
                                    const MatrixXd& Pk_):
                Base(dimension, state_dim, num_states, start_indx, temperature, high_temperature){
                Base::_func_phi = [this, function, cost_class](const VectorXd& x){return MatrixXd::Constant(1, 1, function(x, cost_class));};
                Base::_func_Vmu = [this, function, cost_class](const VectorXd& x){return (x-Base::_mu) * function(x, cost_class);};
                Base::_func_Vmumu = [this, function, cost_class](const VectorXd& x){return MatrixXd{(x-Base::_mu) * (x-Base::_mu).transpose().eval() * function(x, cost_class)};};
                Base::_gh = std::make_shared<GH>(GH{6, dimension, Base::_mu, Base::_covariance});
            }

    };
}


double cost_fixed_gp(const VectorXd& x, const gvi::FixedPriorGP& fixed_gp){
    return fixed_gp.fixed_factor_cost(x);
}

double cost_linear_gp(const VectorXd& pose, const gvi::MinimumAccGP& gp_minacc){
    int dim = gp_minacc.dim_posvel();
    VectorXd x1 = pose.segment(0, dim);
    VectorXd x2 = pose.segment(dim, dim);

    return gp_minacc.cost(x1, x2);
}

double cost_obstacle(const VectorXd& pose, const gpmp2::ObstaclePlanarSDFFactorPointRobot& obs_factor){
    /**
     * Obstacle factor
     * */
    VectorXd vec_err = obs_factor.evaluateError(pose);

    // MatrixXd precision_obs;
    MatrixXd precision_obs{MatrixXd::Identity(vec_err.rows(), vec_err.rows())};
    precision_obs = precision_obs / obs_factor.get_noiseModel()->sigmas()[0];

    return vec_err.transpose() * precision_obs * vec_err;

}

/// test the creation
TEST(TESTVectorOpt, creation){
    int dim = 2; // dimension of x only. The whole theta is in dimension 2*dim
    int ndim = 3*2*dim;
    MatrixXd K{MatrixXd::Identity(2*dim, 2*dim)};
    VectorXd mu{VectorXd::Ones(2*dim)};

    std::vector<std::shared_ptr<gvi::GVIFactorizedBase>> vec_opt;

    MatrixXd P0{MatrixXd::Zero(2*dim, ndim)};
    P0.block(0, 0, 2*dim, 2*dim) = std::move(MatrixXd::Identity(2*dim, 2*dim));

    gvi::FixedPriorGP p_fixed_gp{K, mu};
    std::shared_ptr<VIMPFactorizedOneCost<gvi::FixedPriorGP>> p_opt_fixedgp{new VIMPFactorizedOneCost<gvi::FixedPriorGP>{
                                                                                                        2*dim,
                                                                                                        2*dim, 
                                                                                                        3,
                                                                                                        0,
                                                                                                        1.0,
                                                                                                        10.0,
                                                                                                        cost_fixed_gp, 
                                                                                                        p_fixed_gp, 
                                                                                                        P0}}; 

    vec_opt.emplace_back(std::move(p_opt_fixedgp));

    MatrixXd P1{MatrixXd::Zero(4*dim, ndim)};
    P1.block(0, 0, 4*dim, 4*dim) = std::move(MatrixXd::Identity(4*dim, 4*dim));

    double delt_t = 0.01;

    MatrixXd Qc{MatrixXd::Identity(dim, dim)};
    gvi::MinimumAccGP lin_gp{Qc, 0, delt_t, mu};

    std::shared_ptr<VIMPFactorizedOneCost<gvi::MinimumAccGP>> p_opt_lingp{new VIMPFactorizedOneCost<gvi::MinimumAccGP>{
                                                                                            4*dim, 
                                                                                            2*dim,
                                                                                            3,
                                                                                            0,
                                                                                            1.0,
                                                                                            10.0,
                                                                                            cost_linear_gp, 
                                                                                            lin_gp, 
                                                                                            P1}};
    vec_opt.emplace_back(std::move(p_opt_lingp));

    gvi::GVIGH<gvi::GVIFactorizedBase> opt{vec_opt, 2*dim, 1};

    double thres = 1e-10;  

    ASSERT_EQ(0,0);  
    
}


/// test the integration function
TEST(TESTVectorOpt, integration){
    int dim = 2;
    int dim_theta = 2*dim;
    int ndim = 3*dim_theta;
    MatrixXd K{MatrixXd::Identity(dim_theta, dim_theta)};
    VectorXd mu{VectorXd::Ones(dim_theta)};

    std::vector<std::shared_ptr<gvi::GVIFactorizedBase>> vec_opt;

    MatrixXd P0{MatrixXd::Zero(dim_theta, ndim)};
    P0.block(0, 0, dim_theta, dim_theta) = std::move(MatrixXd::Identity(dim_theta, dim_theta));

    gvi::FixedPriorGP fixed_gp{K, mu};
    std::shared_ptr<VIMPFactorizedOneCost<gvi::FixedPriorGP>> p_opt_fixedgp{new VIMPFactorizedOneCost<gvi::FixedPriorGP>{
                                                                                                dim_theta,
                                                                                                dim_theta, 
                                                                                                3,
                                                                                                0,
                                                                                                1.0,
                                                                                                10.0,
                                                                                                cost_fixed_gp, 
                                                                                                fixed_gp, 
                                                                                                P0}}; 
                                                                                               
    vec_opt.emplace_back(p_opt_fixedgp);

    MatrixXd P1{MatrixXd::Zero(2*dim_theta, ndim)};
    P1.block(0, 0, 2*dim_theta, 2*dim_theta) = std::move(MatrixXd::Identity(2*dim_theta, 2*dim_theta));

    double delt_t = 0.01;

    MatrixXd Qc{MatrixXd::Identity(dim, dim)};
    gvi::MinimumAccGP lin_gp{Qc, 0, delt_t, mu};

    std::shared_ptr<VIMPFactorizedOneCost<gvi::MinimumAccGP>> p_opt_lingp{new VIMPFactorizedOneCost<gvi::MinimumAccGP>{
                                                                                            2*dim_theta, 
                                                                                            dim_theta,
                                                                                            3,
                                                                                            0,
                                                                                            1.0,
                                                                                            10.0,
                                                                                            cost_linear_gp, 
                                                                                            lin_gp, 
                                                                                            P1}};
    vec_opt.emplace_back(p_opt_lingp);
    
    gvi::GVIGH<gvi::GVIFactorizedBase> opt{vec_opt, dim_theta, 3};
    opt.set_GH_degree(5);
    const double thres = 1e-10;    

    cout << "p_opt_fixedgp->EPhi " << endl << p_opt_fixedgp->E_Phi() << endl;
    cout << "----------------" << endl;
    cout << "p_opt_lingp->EPhi " << endl << p_opt_lingp->E_Phi() << endl;
    cout << "----------------" << endl;

    ASSERT_LE(abs(opt.E_Phis()[0] - p_opt_fixedgp->E_Phi()), thres);
    ASSERT_LE(abs(opt.E_Phis()[1] - p_opt_lingp->E_Phi()), thres);

    ASSERT_LE((opt.E_xMuPhis()[0] - p_opt_fixedgp->E_xMuPhi()).norm(), thres);
    ASSERT_LE((opt.E_xMuxMuTPhis()[0] - p_opt_fixedgp->E_xMuxMuTPhi()).norm(), thres);

    ASSERT_LE((opt.E_xMuPhis()[1] - p_opt_lingp->E_xMuPhi()).norm(), thres);
    ASSERT_LE((opt.E_xMuxMuTPhis()[1] - p_opt_lingp->E_xMuxMuTPhi()).norm(), thres);

    
}

TEST(TESTVectorOpt, collision_factor){
    PlanarPointRobotSDFExample planar_pr_sdf;
    PointRobotModel pRModel = std::move(planar_pr_sdf.pRmodel());
    PlanarSDF sdf = std::move(planar_pr_sdf.sdf());

    /// Obs factor
    double cost_sigma = 1.0, epsilon = 1.5;

    int dim = 4;
    int ndim = 3*dim;

    std::vector<std::shared_ptr<gvi::GVIFactorizedBase>> vec_opt;

    MatrixXd P0{MatrixXd::Zero(dim, ndim)};
    P0.block(0, 0, dim, dim) = std::move(MatrixXd::Identity(dim, dim));

    // collision
    ObstaclePlanarSDFFactorPointRobot collision_k{gtsam::symbol('x', 0), pRModel, sdf, cost_sigma, epsilon};

    std::shared_ptr<VIMPFactorizedOneCost<ObstaclePlanarSDFFactorPointRobot>> p_opt_obs{new VIMPFactorizedOneCost<ObstaclePlanarSDFFactorPointRobot>{
                                                                                                dim, 
                                                                                                dim,
                                                                                                3,
                                                                                                0,
                                                                                                1.0,
                                                                                                10.0,
                                                                                                cost_obstacle, 
                                                                                                collision_k, 
                                                                                                P0}}; 

    vec_opt.emplace_back(p_opt_obs);

    gvi::GVIGH<gvi::GVIFactorizedBase> opt{vec_opt, 4, 1};
    opt.set_GH_degree(10);
    double thres = 1e-10;    

    /// Check the equivalence of E_Phix
    ASSERT_LE(abs(opt.E_Phis()[0] - vec_opt[0]->E_Phi()), thres);

    ASSERT_LE((opt.E_xMuPhis()[0] - p_opt_obs->E_xMuPhi()).norm(), thres);
    ASSERT_LE((opt.E_xMuxMuTPhis()[0] - p_opt_obs->E_xMuxMuTPhi()).norm(), thres);

}
