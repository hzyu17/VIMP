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

// TEST(TEST_INVERSER, basic_inverses){
//     int dim = 25;
//     MatrixXd eye = MatrixXd::Identity(dim, dim);
//     dense_inverser inverser{eye};
//     ASSERT_EQ((inverser.inverse(eye) - eye.inverse()).norm(), 0);
//     ASSERT_EQ((inverser.inverse() - eye.inverse()).norm(), 0);

//     MatrixXd rand = MatrixXd::Identity(dim, dim);
//     dense_inverser inverser1{rand};  
//     ASSERT_EQ((inverser1.inverse(rand) - rand.inverse()).norm(), 0);
//     ASSERT_EQ((inverser1.inverse() - rand.inverse()).norm(), 0);

//     inverser1.update_matrix(MatrixXd::Random(dim, dim));
//     ASSERT_NE((inverser1.inverse() - rand.inverse()).norm(), 0);

//     inverser1.update_matrix(rand);
//     ASSERT_EQ((inverser1.inverse() - rand.inverse()).norm(), 0);
//     ASSERT_EQ((inverser1.inverse(rand) - rand.inverse()).norm(), 0);

//     ASSERT_EQ((inverser1.inverse(inverser1.inverse(rand)) - rand).norm(), 0);
//     ASSERT_EQ((inverser1.inverse(inverser1.inverse()) - rand).norm(), 0);
    
// }

/**
 * @brief Test with a more complicate case encountered in the experiment.
 */
TEST(TEST_INVERSER, special_case){
    MatrixIO matrix_io;
    MatrixXd precision = matrix_io.load_csv("/home/hongzhe/git/VIMP/vimp/test_sp_inverse_prec.csv");
    int ncols = precision.cols();
    int nrows = precision.rows();

    int ii = 5;
    // MatrixXd precision = j_precision.block(ncols*(ii-1), 0, ncols, ncols);
    // cout << "precision " << endl << precision << endl;

    dense_inverser inverser(precision, 2);
    
    MatrixXd cov;
    cov = inverser.inverse();

    matrix_io.saveData("test_sp_inverse_cov.csv", cov);
    
}
