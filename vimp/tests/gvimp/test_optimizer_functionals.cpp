/**
 * @file test_optimizer_functionals.cpp
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Test functions defined in the optimizers using the 1d example data.
 * @version 0.1
 * @date 2022-07-26
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "GaussianVI/ngd/NGDFactorizedSimpleGH.h"
#include "GaussianVI/gvibase/GVI-GH.h"
#include <gtest/gtest.h>
#include "GaussianVI/quadrature/GaussHermite.h"


double cost_function(const VectorXd& vec_x){
    double x = vec_x(0);
    double mu_p = 20, f = 400, b = 0.1, sig_r_sq = 0.09;
    double sig_p_sq = 9;

    // y should be sampled. for single trial just give it a value.
    double y = mu_p + 0.05;

    return (x - mu_p)*(x -  mu_p) / sig_p_sq / 2 + (y - f*b/x)*(y - f*b/x) / sig_r_sq / 2; 

}


MatrixXd Phix(const VectorXd& x){
    return MatrixXd{MatrixXd::Constant(1,1,cost_function(x))};
}

MatrixXd x_minus_mu_phi(const VectorXd& x){
    VectorXd mu{VectorXd::Constant(1, 20)};
    return MatrixXd{(x-mu)*cost_function(x)};
}

MatrixXd x_minus_mu_x_minus_muT_phi(const VectorXd& x){
    VectorXd mu{VectorXd::Constant(1, 20)};
    return MatrixXd{(x-mu)*(x-mu).transpose().eval()*cost_function(x)};
}

using namespace std;

typedef std::function<double(const VectorXd&)> Function;
typedef std::function<MatrixXd(const VectorXd&)> GHFunction;
typedef gvi::NGDFactorizedSimpleGH<Function> OptFact;


/**
 * @brief Test function phi, (x-mu)*phi, and (x-mu)(x-mu)^T*phi
 * 
 */
TEST(TestFunctional, function_values){
    MatrixXd Pk{MatrixXd::Constant(1, 1, 1)}; 
    std::shared_ptr<OptFact> p_opt_fac{new OptFact(1, 1, 1, 0, cost_function, 1.0, 10.0)};
    
    VectorXd mean{VectorXd::Constant(1, 20)};
    MatrixXd cov{MatrixXd::Constant(1,1,9)};

    p_opt_fac->update_mu(mean);
    p_opt_fac->update_covariance(cov);

    double Phi_mean_expected = 1810.013888888889;
    ASSERT_EQ(abs(p_opt_fac->negative_log_probability(mean)(0, 0)-Phi_mean_expected), 0);
    
    /// All below functions evaluated at mu should be 0;
    ASSERT_EQ(p_opt_fac->xMu_negative_log_probability(mean).norm(), 0);
    ASSERT_EQ(p_opt_fac->xMuxMuT_negative_log_probability(mean).norm(), 0);

    /// The following should not be 0.
    VectorXd mean_rd = mean + VectorXd::Random(1);
    ASSERT_NE(p_opt_fac->xMu_negative_log_probability(mean_rd).norm(), 0);
    ASSERT_NE(p_opt_fac->xMuxMuT_negative_log_probability(mean_rd).norm(), 0);

    /// update of the covariance should not change the results.
    MatrixXd cov_rd = cov + MatrixXd::Random(1,1);
    p_opt_fac->update_covariance(cov_rd);

    ASSERT_EQ((p_opt_fac->covariance()-cov_rd).norm(), 0);

    ASSERT_EQ(abs(p_opt_fac->negative_log_probability(mean)(0, 0)-Phi_mean_expected), 0);

    /// All below functions evaluated at mu should be 0;
    ASSERT_EQ(p_opt_fac->xMu_negative_log_probability(mean).norm(), 0);
    ASSERT_EQ(p_opt_fac->xMuxMuT_negative_log_probability(mean).norm(), 0);

    /// The following should not be 0.
    ASSERT_NE(p_opt_fac->xMu_negative_log_probability(mean_rd).norm(), 0);
    ASSERT_NE(p_opt_fac->xMuxMuT_negative_log_probability(mean_rd).norm(), 0);

}

/**
 * @brief Test the integration process
 * 
 */
TEST(TestFunctional, GH_integrations){

    VectorXd mean{VectorXd::Constant(1, 20.0)};
    MatrixXd cov{MatrixXd::Constant(1, 1, 9.0)};

    int deg = 6;
    int dim = 1;
    typedef gvi::GaussHermite<GHFunction> GH;
    
    GH gh_inst{deg, dim, mean, cov};

    double IntPhix_expected = 1.801423462172827e+03;
    ASSERT_LE(abs(gh_inst.Integrate(Phix)(0,0) - IntPhix_expected), 1e-10);

    // gh_inst.update_integrand(x_minus_mu_phi);
    double IntxMuPhix_expected = 1.925745912968216e+02;
    ASSERT_LE(abs(gh_inst.Integrate(x_minus_mu_phi)(0,0) - IntxMuPhix_expected), 1e-10);

    // gh_inst.update_integrand(x_minus_mu_x_minus_muT_phi);
    double IntxMuxMuTPhix_expected = 1.604681203455295e+04;
    ASSERT_LE(abs(gh_inst.Integrate(x_minus_mu_x_minus_muT_phi)(0,0) - IntxMuxMuTPhix_expected), 1e-10);

}


/**
 * @brief Test if the function integrated inside 
 * the optimizer is the true one we want.
 */
TEST(TestFunctional, integrations){
    MatrixXd Pk{MatrixXd::Constant(1, 1, 1)}; 

    std::shared_ptr<OptFact> p_opt_fac{new OptFact(1, 1, 1, 0, cost_function, 1.0, 10.0)};
    
    VectorXd mean = VectorXd::Constant(1, 20.0);
    MatrixXd cov = MatrixXd::Constant(1,1,9.0);
    p_opt_fac->update_mu(mean);
    p_opt_fac->update_covariance(cov);

    p_opt_fac->set_GH_points(6);
    p_opt_fac->updateGH(mean, cov);

    /// E_Phi
    double E_Phi = p_opt_fac->E_Phi();
    double E_PHi_expected = 1.801423462172827e+03;

    ASSERT_LE(abs(E_Phi - E_PHi_expected), 1e-10);

    /// E_xMuPhi
    MatrixXd E_xMuPhi{p_opt_fac->E_xMuPhi()};
    double E_xMuPhi_expected = 1.925745912968216e+02;

    ASSERT_LE(abs(E_xMuPhi(0,0) - E_xMuPhi_expected), 1e-10);
    
    /// E_xMuxMuTPhi
    MatrixXd E_xMuxMuTPhi{p_opt_fac->E_xMuxMuTPhi()};
    double E_xMuxMuTPhi_expected = 1.604681203455295e+04;
    
    ASSERT_LE(abs(E_xMuxMuTPhi(0,0) - E_xMuxMuTPhi_expected), 1e-10);

}

/**
 * @brief To see if the cost value match with the ground truth.
 */
TEST(TestFunctional, cost_value){

    MatrixXd Pk{MatrixXd::Constant(1, 1, 1)}; 
    cout << "Pk" << endl << Pk << endl;
    std::shared_ptr<OptFact> p_opt_fac{new OptFact(1, 1, 1, 0, cost_function, 1.0, 10.0)};
    
    VectorXd mean = VectorXd::Constant(1, 20.0);
    MatrixXd cov = MatrixXd::Constant(1,1,9.0);
    p_opt_fac->update_mu(mean);
    p_opt_fac->update_covariance(cov);

    p_opt_fac->set_GH_points(6);
    p_opt_fac->updateGH(mean, cov);

    ASSERT_LE(abs(p_opt_fac->precision().determinant() - 1/9.0), 1e-10);

    /// cost value
    double cost_value = p_opt_fac->E_Phi() + log(p_opt_fac->precision().determinant()) / 2;
    double cost_value_expected = 1.800324849884159e+03;

    ASSERT_LE(abs(cost_value-cost_value_expected), 1e-10);

}

/**
 * @brief Test the cost value function
 */
TEST(TestFunctional, joint_cost_value){
    vector<std::shared_ptr<OptFact>> vec_opts;
    MatrixXd Pk{MatrixXd::Constant(1, 1, 1)}; 
    std::shared_ptr<OptFact> p_opt_fac{new OptFact(1, 1, 1, 0, cost_function, 1.0, 10.0)};

    vec_opts.emplace_back(p_opt_fac);

    gvi::GVIGH<OptFact> opt{vec_opts, 1, 1};

    VectorXd mu_rd{VectorXd::Random(1)};
    MatrixXd prec_rd{MatrixXd::Random(1,1)};

    gvi::SpMat prec_rd_sp = prec_rd.sparseView();

    opt.set_mu(mu_rd);
    opt.set_precision(prec_rd_sp);

    /// should be equal
    gvi::SpMat cov_sp = prec_rd.inverse().sparseView();
    ASSERT_LE(abs(opt.cost_value() - opt.cost_value(mu_rd, cov_sp)), 1e-10);

}