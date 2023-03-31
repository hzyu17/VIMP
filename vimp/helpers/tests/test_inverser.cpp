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

#include "helpers/SparseInverseMatrix.h"
#include <gtest/gtest.h>
#include "helpers/data_io.h"

using namespace Eigen;
using namespace std;
using namespace vimp;

/**
 * @brief Test with a more complicate case encountered in the experiment.
 */
TEST(TEST_INVERSER, special_case){
    MatrixIO matrix_io;
    MatrixXd precision = matrix_io.load_csv("../test_sp_inverse_prec.csv");
    int ncols = precision.cols();
    int nrows = precision.rows();

    int ii = 5;
    // MatrixXd precision = j_precision.block(ncols*(ii-1), 0, ncols, ncols);

    dense_inverser inverser(precision, 2);
    
    MatrixXd cov;
    cov = inverser.inverse();

    matrix_io.saveData("test_sp_inverse_cov.csv", cov);
    
}
