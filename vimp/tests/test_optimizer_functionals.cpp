/**
 * @file test_optimizer_functionals.cpp
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Test functions defined in the optimizers.
 * @version 0.1
 * @date 2022-07-26
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "../optimizer/OptimizerFactorizedSimpleGH.h"
#include <gtest/gtest.h>


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

MatrixXd x_minus_mu_phi(const VectorXd& x, const VectorXd& mu){
    return MatrixXd{(x-mu)*cost_function(x)};
}

MatrixXd x_minus_mu_x_minus_muT_phi(const VectorXd& x, const VectorXd& mu){
    return MatrixXd{(x-mu)*(x-mu).transpose().eval()*cost_function(x)};
}

using namespace std;
using namespace vimp;
typedef std::function<double(const VectorXd&)> Function;
typedef VIMPOptimizerFactorizedSimpleGH<Function> OptFact;


/**
 * @brief Test function phi, (x-mu)*phi, and (x-mu)(x-mu)^T*phi
 * 
 */
TEST(TestFunctional, function_values){
    MatrixXd Pk{MatrixXd::Constant(1, 1, 1)}; 
    std::shared_ptr<OptFact> p_opt_fac{new OptFact(1, cost_function, Pk)};
    
    VectorXd mean{VectorXd::Constant(1, 20)};
    MatrixXd cov{MatrixXd::Constant(1,1,9)};

    p_opt_fac->update_mu(mean);
    p_opt_fac->update_covariance(cov);

    double Phi_mean_expected = 1810.013888888889;
    ASSERT_EQ(abs(p_opt_fac->Phi(mean)(0, 0)-Phi_mean_expected), 0);
    
    /// All below functions evaluated at mu should be 0;
    ASSERT_EQ(p_opt_fac->xMuPhi(mean).norm(), 0);
    ASSERT_EQ(p_opt_fac->xMuxMuTPhi(mean).norm(), 0);

    /// The following should not be 0.
    VectorXd mean_rd = mean + VectorXd::Random(1);
    ASSERT_NE(p_opt_fac->xMuPhi(mean_rd).norm(), 0);
    ASSERT_NE(p_opt_fac->xMuxMuTPhi(mean_rd).norm(), 0);


    /// update of the covariance should not change the results.
    MatrixXd cov_rd = cov + MatrixXd::Random(1,1);
    p_opt_fac->update_covariance(cov_rd);

    ASSERT_EQ((p_opt_fac->covariance()-cov_rd).norm(), 0);

    ASSERT_EQ(abs(p_opt_fac->Phi(mean)(0, 0)-Phi_mean_expected), 0);

    /// All below functions evaluated at mu should be 0;
    ASSERT_EQ(p_opt_fac->xMuPhi(mean).norm(), 0);
    ASSERT_EQ(p_opt_fac->xMuxMuTPhi(mean).norm(), 0);

    /// The following should not be 0.
    ASSERT_NE(p_opt_fac->xMuPhi(mean_rd).norm(), 0);
    ASSERT_NE(p_opt_fac->xMuxMuTPhi(mean_rd).norm(), 0);

}


/**
 * @brief Test if the function integrated inside 
 * the optimizer is the true one we want.
 */
TEST(TestFunctional, integrations){
    MatrixXd Pk{MatrixXd::Constant(1, 1, 1)}; 

    cout << "Pk" << endl << Pk << endl;

    std::shared_ptr<OptFact> p_opt_fac{new OptFact(1, cost_function, Pk)};
    
    p_opt_fac->update_mu(VectorXd::Constant(1, 20));
    p_opt_fac->update_covariance(MatrixXd::Constant(1,1,9));

    /// E_Phi
    double E_Phi = p_opt_fac->E_Phi();
    double E_PHi_expected = 1.801423462172827e+03;

    cout << "p_opt_fac->E_Phi() " << endl << E_Phi << endl;

    // ASSERT_LE(abs(E_Phi - E_PHi_expected), 1e-6);

    /// E_xMuPhi
    MatrixXd E_xMuPhi{p_opt_fac->E_xMuPhi()};
    MatrixXd E_xMuPhi_expected{MatrixXd::Constant(1,1,1.925745912968216e+02)};

    cout << "p_opt_fac->E_xMuPhi() " << endl << E_xMuPhi << endl;

    // ASSERT_LE(abs(E_xMuPhi(0,0) - E_xMuPhi_expected(0,0)), 1e-6);
    
    /// E_xMuxMuTPhi
    MatrixXd E_xMuxMuTPhi{p_opt_fac->E_xMuxMuTPhi()};
    MatrixXd E_xMuxMuTPhi_expected{MatrixXd::Constant(1,1,1.604681203455295e+04)};
    
    cout << "E_xMuxMuTPhi " << endl << E_xMuxMuTPhi << endl;
    
    // ASSERT_LE(abs(E_xMuxMuTPhi(0,0) - E_xMuxMuTPhi_expected(0,0)), 1e-6);

}