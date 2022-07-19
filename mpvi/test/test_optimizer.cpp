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
#include "optimizer/OptimizerGH-impl.h"
#include "optimizer/OptimizerFactorizedGHBase-impl.h"


TEST(TestFactorizedOptBase, BasicFunctions){
    int dim = 3;
    MatrixXd Pk{MatrixXd::Identity(dim, dim)};
    std::shared_ptr<VIMPOptimizerFactorizedBase> p_opt{new VIMPOptimizerFactorizedBase(dim, Pk)};

    VectorXd mean{VectorXd::Random(dim)};
    MatrixXd precision{MatrixXd::Random(dim, dim)};

    p_opt->update_mu(mean, precision);
    
    ASSERT_EQ(p_opt->mean(), mean);
    ASSERT_EQ(p_opt->precision(), precision);
}