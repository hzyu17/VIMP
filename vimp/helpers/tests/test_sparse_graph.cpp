/**
 * @file test_graph.cpp
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Test on graphs.
 * @version 0.1
 * @date 2023-01-08
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include<gtest/gtest.h>

#include "helpers/sparse_graph.h"
#include "helpers/data_io.h"
#include "helpers/timer.h"

vimp::MatrixIO m_io;
Timer timer;
EigenWrapper eigen_wrapper;

TEST(TestGraph, init){
    int state_dim = 4;
    int num_state = 3;
    vimp::TrajectoryGraph trj_graph(state_dim, num_state);

    Eigen::MatrixXd init_desired(16, 16);
    init_desired <<
    0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0,	    0,	    0,	    0,	    0,	    0,	    0,	    0,
    0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0,	    0,	    0,	    0,	    0,	    0,	    0,	    0,
    0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0,	    0,	    0,	    0,	    0,	    0,	    0,	    0,
    0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0,	    0,	    0,	    0,	    0,	    0,	    0,	    0,
    0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0,	    0,	    0,	    0,
    0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0,	    0,	    0,	    0,
    0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0,	    0,	    0,	    0,
    0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0,	    0,	    0,	    0,
    0,	    0,	    0,	    0,	    0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,
    0,	    0,	    0,	    0,	    0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,
    0,	    0,	    0,	    0,	    0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,
    0,	    0,	    0,	    0,	    0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,
    0,	    0,	    0,	    0,	    0,	    0,	    0,	    0,	    0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,
    0,	    0,	    0,	    0,	    0,	    0,	    0,	    0,	    0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,
    0,	    0,	    0,	    0,	    0,	    0,	    0,	    0,	    0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,
    0,	    0,	    0,	    0,	    0,	    0,	    0,	    0,	    0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01,	0.01;

    ASSERT_TRUE(trj_graph.precision_matrix_eq(init_desired));
    
}

TEST(TestGraph, covariance_block){
    Eigen::MatrixXd precision = m_io.load_csv("precision_10.csv");
    Eigen::MatrixXd covariance = m_io.load_csv("cov_10.csv");

    ASSERT_LE((covariance - precision.inverse()).norm(), 1e-10);

    int state_dim = 4;
    int num_state = 10;
    vimp::TrajectoryGraph trj_graph(state_dim, num_state);

    SpMat precision_sp = precision.sparseView();
    trj_graph.set_precision(precision_sp);
    std::cout << "time for sparse inverse " << std::endl;
    timer.start();
    trj_graph.inverse_precision();
    timer.end();
    SpMat cov_inv_sp{trj_graph.covariance_matrix()};
    Eigen::MatrixXd cov_inv_sp_fullview{cov_inv_sp};
    m_io.saveData("cov10_sp_inv.csv", cov_inv_sp_fullview);

    std::cout << "time for full inverse " << std::endl;
    Eigen::MatrixXd cov_full;
    timer.start();
    cov_full = precision.inverse();
    timer.end();
    
    ASSERT_LE((cov_full - cov_inv_sp).norm(), 1e-10);
    
}

int main(int argc, char **argv){

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
    
}