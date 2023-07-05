/**
 * @file test_optimizer.cpp
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Test the optimizer (joint and marginal) classes
 * @version 0.1
 * @date 2022-07-19
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <gtest/gtest.h>
#include "gvimp/GVI-GH.h"
#include "gvimp/GVIFactorizedGHBase.h"

using namespace std;
using namespace vimp;

TEST(TestFactorizedOptBase, basicFunctions){
    int dim = 3;
    MatrixXd Pk{MatrixXd::Identity(dim, dim)};
    std::shared_ptr<VIMPOptimizerFactorizedBase> p_opt{new VIMPOptimizerFactorizedBase{dim, Pk}};

    VectorXd mean{VectorXd::Random(dim)};
    MatrixXd precision{MatrixXd::Random(dim, dim)};
    MatrixXd cov{precision.inverse()};

    p_opt->update_mu(mean);
    p_opt->update_precision(precision);

    const double tol = 1e-15;
    
    ASSERT_LE((p_opt->mean() - mean).norm(), tol);
    ASSERT_LE((p_opt->precision() - precision).norm(), tol);
    ASSERT_LE((p_opt->covariance() - cov).norm(), tol);
}


TEST(TestFactorizedOptBase, fromJoint){
    const int dim = 3;
    const double tol = 1e-15;

    MatrixXd P1{MatrixXd::Zero(dim, 2*dim)};
    MatrixXd P2{MatrixXd::Zero(dim, 2*dim)};
    P1.block(0, 0, dim, dim) = MatrixXd::Identity(dim, dim);
    P2.block(0, dim, dim, dim) = MatrixXd::Identity(dim, dim);

    ASSERT_LE((P1 * P1.transpose() - MatrixXd::Identity(dim, dim)).norm(), tol);
    ASSERT_LE((P2 * P2.transpose() - MatrixXd::Identity(dim, dim)).norm(), tol);

    ASSERT_LE((P2.transpose() * P2 + P1.transpose() * P1 - MatrixXd::Identity(2*dim, 2*dim)).norm(), tol);

    std::shared_ptr<VIMPOptimizerFactorizedBase> p_opt1{new VIMPOptimizerFactorizedBase{dim, P1}};
    std::shared_ptr<VIMPOptimizerFactorizedBase> p_opt2{new VIMPOptimizerFactorizedBase{dim, P2}};

    ASSERT_LE((p_opt1->Pk()-P1).norm(), tol);
    ASSERT_LE((p_opt2->Pk()-P2).norm(), tol);

    VectorXd j_mean{VectorXd::Random(2*dim)};
    MatrixXd j_precision{MatrixXd::Random(2*dim, 2*dim)};
    MatrixXd j_cov{j_precision.inverse()};

    p_opt1->update_mu_from_joint_mean(j_mean);
    p_opt1->update_precision_from_joint_covariance(j_cov);

    p_opt2->update_mu_from_joint_mean(j_mean);
    p_opt2->update_precision_from_joint_covariance(j_cov);
    
    ASSERT_LE((p_opt1->mean() - P1*j_mean).norm(), tol);
    ASSERT_LE((p_opt1->covariance() - P1 * j_cov * P1.transpose()).norm(), tol);

    ASSERT_LE((p_opt2->mean() - P2*j_mean).norm(), tol);
    ASSERT_LE((p_opt2->covariance() - P2 * j_cov * P2.transpose()).norm(), tol);

    ASSERT_NE((p_opt1->precision() - P1 * j_precision * P1.transpose()).norm(), 0);
    ASSERT_NE((p_opt2->precision() - P2 * j_precision * P2.transpose()).norm(), 0);

    ASSERT_LE((p_opt1->joint_mean() + p_opt2->joint_mean() - j_mean).norm(), tol);
    ASSERT_LE((P1 * p_opt1->joint_covariance() * P1.transpose() - P1 * j_cov * P1.transpose()).norm(), tol);
    ASSERT_LE((P2 * p_opt2->joint_covariance() * P2.transpose() - P2 * j_cov * P2.transpose()).norm(), tol);

    // ASSERT_LE((p_opt1->joint_precision() + p_opt2->joint_precision() - j_precision).norm(), tol);
    cout << "p_opt1->joint_covariance() " << endl << p_opt1->joint_covariance() << endl;
    cout << "p_opt2->joint_covariance() " << endl << p_opt2->joint_covariance() << endl;
    cout << "j_cov " << endl << j_cov << endl;
    // ASSERT_LE((p_opt1->joint_covariance() + p_opt2->joint_covariance() - j_cov).norm(), tol);
    cout << "p_opt1->joint_precision() " << endl << p_opt1->joint_precision() << endl;
    cout << "p_opt2->joint_precision() " << endl << p_opt2->joint_precision() << endl;
    cout << "j_precision " << endl << j_precision << endl;
    
}

TEST(TestFactorizedOptBase, DECOUPLED_precision){
    const int dim = 2;
    const double tol = 1e-10;

    MatrixXd P1{MatrixXd::Zero(dim, 2*dim)};
    MatrixXd P2{MatrixXd::Zero(dim, 2*dim)};

    P1.block(0, 0, dim, dim) = MatrixXd::Identity(dim, dim);
    P2.block(0, 2, dim, dim) = MatrixXd::Identity(dim, dim);

    std::shared_ptr<VIMPOptimizerFactorizedBase> p_opt1{new VIMPOptimizerFactorizedBase{dim, P1}};
    std::shared_ptr<VIMPOptimizerFactorizedBase> p_opt2{new VIMPOptimizerFactorizedBase{dim, P2}};

    VectorXd j_mean{VectorXd::Random(2*dim)};
    MatrixXd j_precision{MatrixXd::Random(2*dim, 2*dim)};
    j_precision.block(0,2,2,2) = MatrixXd::Zero(2,2);
    j_precision.block(2,0,2,2) = MatrixXd::Zero(2,2);
    MatrixXd j_cov{j_precision.inverse()};

    p_opt1->update_mu_from_joint_mean(j_mean);
    p_opt1->update_precision_from_joint_covariance(j_cov);

    ASSERT_LE((p_opt1->mean() - p_opt1->Pk()*j_mean).norm(), tol);
    ASSERT_LE((p_opt1->covariance() - p_opt1->Pk() * j_cov * p_opt1->Pk().transpose()).norm(), tol);

    p_opt2->update_mu_from_joint_mean(j_mean);
    p_opt2->update_precision_from_joint_covariance(j_cov);

    VectorXd sum_j_mean = p_opt1->joint_mean() + p_opt2->joint_mean();
    ASSERT_LE((sum_j_mean- j_mean).norm(), tol);
    
    MatrixXd sum_j_precision = p_opt1->joint_precision() + p_opt2->joint_precision();
    ASSERT_LE((sum_j_precision - j_precision).norm(), tol);

}


TEST(TestFactorizedOptBase, sparse_precision){
    const int dim = 2;
    const double tol = 1e-10;

    MatrixXd P1{MatrixXd::Zero(dim, 2*dim)};
    MatrixXd P2{MatrixXd::Zero(dim, 2*dim)};
    MatrixXd P3{MatrixXd::Zero(dim, 2*dim)};

    P1.block(0, 0, dim, dim) = MatrixXd::Identity(dim, dim);
    P2.block(0, 1, dim, dim) = MatrixXd::Identity(dim, dim);
    P3.block(0, 2, dim, dim) = MatrixXd::Identity(dim, dim);

    RowVectorXd row_sum{RowVectorXd::Zero((2*dim))};
    row_sum = row_sum + P1.colwise().sum();
    row_sum = row_sum + P2.colwise().sum();
    row_sum = row_sum + P3.colwise().sum();

    std::shared_ptr<VIMPOptimizerFactorizedBase> p_opt1{new VIMPOptimizerFactorizedBase{dim, P1}};
    std::shared_ptr<VIMPOptimizerFactorizedBase> p_opt2{new VIMPOptimizerFactorizedBase{dim, P2}};
    std::shared_ptr<VIMPOptimizerFactorizedBase> p_opt3{new VIMPOptimizerFactorizedBase{dim, P3}};

    VectorXd j_mean{VectorXd::Random(2*dim)};
    MatrixXd j_precision{MatrixXd::Random(2*dim, 2*dim)};
    j_precision(0, 2) = 0;
    j_precision(0, 3) = 0;
    j_precision(1, 3) = 0;
    j_precision(2, 0) = 0;
    j_precision(3, 0) = 0;
    j_precision(3, 1) = 0;
    MatrixXd j_cov{j_precision.inverse()};
    cout << "j_mean " << endl << j_mean << endl;

    p_opt1->update_mu_from_joint_mean(j_mean);
    p_opt1->update_precision_from_joint_covariance(j_cov);

    ASSERT_LE((p_opt1->mean() - p_opt1->Pk()*j_mean).norm(), tol);
    ASSERT_LE((p_opt1->covariance() - p_opt1->Pk() * j_cov * p_opt1->Pk().transpose()).norm(), tol);

    p_opt2->update_mu_from_joint_mean(j_mean);
    p_opt2->update_precision_from_joint_covariance(j_cov);

    ASSERT_LE((p_opt2->mean() - p_opt2->Pk()*j_mean).norm(), tol);
    ASSERT_LE((p_opt2->covariance() - p_opt2->Pk() * j_cov * p_opt2->Pk().transpose()).norm(), tol);

    p_opt3->update_mu_from_joint_mean(j_mean);
    p_opt3->update_precision_from_joint_covariance(j_cov);

    ASSERT_LE((p_opt3->mean() - p_opt3->Pk()*j_mean).norm(), tol);
    ASSERT_LE((p_opt3->covariance() - p_opt3->Pk() * j_cov * p_opt3->Pk().transpose()).norm(), tol);

    VectorXd sum_j_mean = p_opt1->joint_mean() + p_opt2->joint_mean() + p_opt3->joint_mean();
    sum_j_mean = (sum_j_mean.array() / row_sum.transpose().array()).matrix();
    ASSERT_LE((sum_j_mean- j_mean).norm(), tol);
    
    cout << "p_opt1->joint_covariance() " << endl << p_opt1->joint_covariance() << endl;
    cout << "p_opt2->joint_covariance() " << endl << p_opt2->joint_covariance() << endl;
    cout << "p_opt3->joint_covariance() " << endl << p_opt3->joint_covariance() << endl;
    MatrixXd sum_j_cov = p_opt1->joint_covariance() + 
                        p_opt2->joint_covariance() + 
                        p_opt3->joint_covariance();
    sum_j_cov.diagonal() = sum_j_cov.diagonal().array() / row_sum.transpose().array();
    cout << "sum_j_cov" << endl << sum_j_cov << endl;
    cout << "j_cov " << endl << j_cov << endl;


    MatrixXd sum_j_precision = p_opt1->joint_precision() + 
                                p_opt2->joint_precision() + 
                                p_opt3->joint_precision();

    cout << "p_opt1->joint_precision() " << endl << p_opt1->joint_precision() << endl;
    cout << "p_opt2->joint_precision() " << endl << p_opt2->joint_precision() << endl;
    cout << "p_opt3->joint_precision() " << endl << p_opt3->joint_precision() << endl;

    cout << "sum joint_precision() " << endl << sum_j_precision << endl;
    sum_j_precision.diagonal() = sum_j_precision.diagonal().array() / row_sum.transpose().array();
    cout << "sum joint_precision() " << endl << sum_j_precision << endl;
    cout << "j_precision " << endl << j_precision << endl;

    ASSERT_LE((sum_j_cov - j_cov).norm(), tol);
    ASSERT_LE((sum_j_precision - j_precision).norm(), tol);
    
}


TEST(TestFactorizedOptBase, costValue){
    int dim = 3;
    MatrixXd Pk{MatrixXd::Identity(dim, 2*dim)};
    std::shared_ptr<VIMPOptimizerFactorizedBase> p_opt{new VIMPOptimizerFactorizedBase{dim, Pk}};
    
    VectorXd j_mean{VectorXd::Random(2*dim)};
    MatrixXd j_precision{MatrixXd::Random(2*dim, 2*dim)};
    MatrixXd j_cov{j_precision.inverse()};

}