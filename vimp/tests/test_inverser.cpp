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

#include "../helpers/SparseInverseMatrix.h"
#include <gtest/gtest.h>
#include "../helpers/data_io.h"

using namespace Eigen;
using namespace std;
using namespace vimp;

TEST(TEST_INVERSER, basic_inverses){
    int dim = 25;
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
    ASSERT_EQ((inverser1.inverse(rand) - rand.inverse()).norm(), 0);

    ASSERT_EQ((inverser1.inverse(inverser1.inverse(rand)) - rand).norm(), 0);
    ASSERT_EQ((inverser1.inverse(inverser1.inverse()) - rand).norm(), 0);
    
}

/**
 * @brief Test with a more complicate case encountered in the experiment.
 */
TEST(TEST_INVERSER, special_case){
    MatrixIO matrix_io;
    
    MatrixXd precision = matrix_io.load_csv("precision.csv");
    MatrixXd cov_expected = matrix_io.load_csv("cov_expected.csv");

    dense_inverser inverser(precision);

    ASSERT_LE((cov_expected - precision.inverse()).norm(), 1e-10);
    ASSERT_LE((cov_expected - inverser.inverse()).norm(), 1e-10);
    
}
