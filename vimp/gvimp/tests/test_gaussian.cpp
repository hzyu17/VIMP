/**
 * @file test_gaussian.cpp
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Test the algorithm for a gaussian p(x,z), KL divergence, the cost function, etc.
 * @version 0.1
 * @date 2022-07-26
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <gtest/gtest.h>
#include "gvimp/OptimizerGH.h"
#include "gvimp/OptimizerFactorizedGH.h"
#include "helpers/MVGsampler.h"
#include <iostream>
#include <random>

using namespace std;
using namespace vimp;
using FactorizedOptimizer = VIMPOptimizerFactorizedGaussHermite<std::function<double(const VectorXd&, const Gaussian_distribution&)>, Gaussian_distribution>;

inline double cost_function(const VectorXd& sample, const vimp::Gaussian_distribution& target_distr){
    return -target_distr.log_prob(sample);
}


/**
 * @brief The KL divergence of two same gaussian should be 0.
 */
TEST(TestGaussian, KLdiver){

    int ndim = 2;
    
    // Pks
    MatrixXd Pk{MatrixXd::Zero(2, ndim)};
    Pk(0, 0) = 1;
    Pk(1, 1) = 1;

    // known target posteria Gaussian
    VectorXd mean_t(2);
    MatrixXd precision_t(2, 2);
    precision_t = MatrixXd::Identity(2, 2);
    precision_t(0, 1) = -0.74;
    precision_t(1, 0) = -0.74;
    mean_t = VectorXd::Ones(2);
    MatrixXd cov_t{precision_t.inverse()};

    Gaussian_distribution target_distr(mean_t, precision_t.inverse());

    /// try with unique pointer
    FactorizedOptimizer::shared_ptr pFactOptimizer(new FactorizedOptimizer{2, cost_function, target_distr, Pk});
    
    pFactOptimizer->update_mu(mean_t);
    pFactOptimizer->update_covariance(cov_t);

    pFactOptimizer->updateGH();
    
    // E_{q}[-log(p(x,z))]
    double EPhi = pFactOptimizer->E_Phi();

    // True entropy
    double PI = 3.1415926;
    double Entropy_q = ndim/2*(1+log(2*PI)) + log(cov_t.determinant()) / 2;

    cout << "EPhi - Entropy_q" << endl << EPhi - Entropy_q << endl;

    ASSERT_LE(abs(EPhi - Entropy_q), 1e-6);
    
}

/**
 * @brief The KL divergence test for all marginals in a joint optimizer.
 */
TEST(TestGaussian, KL_diver_joint_decoupled){

    int ndim = 4;

    vector<FactorizedOptimizer::shared_ptr> vec_factorized_opt;

    VectorXd mean_j(ndim);
    MatrixXd precision_j(ndim, ndim);
    precision_j << 1, 0.25, 0, 0, 
                   0.25, 1, 0, 0, 
                   0, 0, 1, 0.25,
                   0, 0, 0.25, 1;
    MatrixXd cov_j{precision_j.inverse()};

    cout << "precision_j " << endl << precision_j << endl;
    cout << "cov_j " << endl << cov_j << endl;

    // vector<MatrixXd> vec_Pks;
    for (int i=0; i<2; i++){
        // Pks
        MatrixXd Pk{MatrixXd::Zero(2, ndim)};
        Pk(0, i*2) = 1;
        Pk(1, i*2+1) = 1;

        cout << "Pk " << endl << Pk << endl;

        Gaussian_distribution target_distr(VectorXd::Ones(2), MatrixXd::Identity(2, 2));

        /// try with unique pointer
        FactorizedOptimizer::shared_ptr pFactOptimizer(new FactorizedOptimizer{2, cost_function, target_distr, Pk});
        vec_factorized_opt.emplace_back(pFactOptimizer);
    }
    VIMPOptimizerGH<FactorizedOptimizer> optimizer(vec_factorized_opt);

    optimizer.set_mu(mean_j);
    optimizer.set_precision(precision_j);

    MatrixXd precision_t(2, 2);
    precision_t = MatrixXd::Identity(2, 2);
    precision_t(0, 1) = 0.25;
    precision_t(1, 0) = 0.25;
    cout << "precision_t " << endl << precision_t << endl;
    MatrixXd cov_t{precision_t.inverse()};

    vector<double> v_EPhi = optimizer.E_Phis();

    for (double & E_Phi: v_EPhi){

        double PI = 3.1415926;
        double Entropy_q = ndim/2*(1+log(2*PI)) + log(cov_t.determinant()) / 2;

        cout << "E_Phi - Entropy_q" << endl << E_Phi - Entropy_q << endl;

        ASSERT_LE(abs(E_Phi - Entropy_q), 1e-6);   
    }


}


/**
 * @brief The KL divergence test for all marginals in a joint optimizer.
 */
TEST(TestGaussian, KL_diver_joint){

    int ndim = 4;

    vector<FactorizedOptimizer::shared_ptr> vec_factorized_opt;

    VectorXd mean_j(ndim);
    MatrixXd precision_j(ndim, ndim);
    precision_j << 1, 0.25, 0, 0, 
                   0.25, 2, 0.25, 0, 
                   0, 0.25, 2, 0.25,
                   0, 0, 0.25, 1;
    MatrixXd cov_j{precision_j.inverse()};

    cout << "precision_j " << endl << precision_j << endl;

    // vector<MatrixXd> vec_Pks;
    for (int i=0; i<ndim-1; i++){
        // Pks
        MatrixXd Pk{MatrixXd::Zero(2, ndim)};
        Pk(0, i) = 1;
        Pk(1, i+1) = 1;

        Gaussian_distribution target_distr(VectorXd::Ones(2), MatrixXd::Identity(2, 2));

        /// try with unique pointer
        FactorizedOptimizer::shared_ptr pFactOptimizer(new FactorizedOptimizer{2, cost_function, target_distr, Pk});
        vec_factorized_opt.emplace_back(pFactOptimizer);
    }
    VIMPOptimizerGH<FactorizedOptimizer> optimizer(vec_factorized_opt);

    optimizer.set_mu(mean_j);
    optimizer.set_precision(precision_j);

    MatrixXd precision_t(2, 2);
    precision_t = MatrixXd::Identity(2, 2);
    precision_t(0, 1) = 0.25;
    precision_t(1, 0) = 0.25;
    cout << "precision_t " << endl << precision_t << endl;
    MatrixXd cov_t{precision_t.inverse()};

    vector<double> v_E_Phi = optimizer.E_Phis();

    for (double & E_Phi: v_E_Phi){
        
        double PI = 3.1415926;
        double Entropy_q = ndim/2*(1+log(2*PI)) + log(cov_t.determinant()) / 2;

        cout << "E_Phi - Entropy_q" << endl << E_Phi - Entropy_q << endl;

        ASSERT_LE(abs(E_Phi - Entropy_q), 1e-6);   
    }
    
    
}

