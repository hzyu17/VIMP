/**
 * @file test_inverser.cpp
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Test inverser class
 * @version 0.1
 * @date 2022-07-19
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "optimizer/SparseInverseMatrix.h"
#include <gtest/gtest.h>

using namespace Eigen;
using namespace std;
using namespace vimp;

TEST(TEST_INVERSER, basic_inverses){
    int dim = 5;
    MatrixXd eye = MatrixXd::Identity(dim, dim);
    dense_inverser inverser{eye};
    ASSERT_EQ((inverser.inverse(eye) - eye.inverse()).norm(), 0);
    ASSERT_EQ((inverser.inverse() - eye.inverse()).norm(), 0);

    MatrixXd rand = MatrixXd::Identity(dim, dim);
    dense_inverser inverser1{rand};  
    ASSERT_EQ((inverser1.inverse(rand) - rand.inverse()).norm(), 0);
    ASSERT_EQ((inverser1.inverse() - rand.inverse()).norm(), 0);

    inverser1.update_matrix(MatrixXd::Random(dim, dim));
    ASSERT_NE((inverser1.inverse() - rand.inverse()).norm(), 0);

    inverser1.update_matrix(rand);
    ASSERT_EQ((inverser1.inverse() - rand.inverse()).norm(), 0);
}

