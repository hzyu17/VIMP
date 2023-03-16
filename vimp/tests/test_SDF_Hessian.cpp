/**
 * @file test_SDF_Hessian.cpp
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Test the gradient and Hessian of a SDF.
 * @version 0.1
 * @date 2023-03-15
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "../helpers/data_io.h"
#include <gtest/gtest.h>
#include "../helpers/HessianSDF.h"
#include "../helpers/eigen_wrapper.h"

using namespace vimp;
using namespace Eigen;

EigenWrapper ei;
// Choose autodiff scalar type for 3 variables
using ADouble = TinyAD::Double<3>;

TEST(SDFHessian, read_csv){
    MatrixIO loader;
    MatrixXd sdf = loader.load_csv("data/map_ground_truth.csv");

    MatrixXd map_ground_truth = (MatrixXd(7, 7) <<
                0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0,
                0, 0, 1, 1, 1, 0, 0,
                0, 0, 1, 1, 1, 0, 0,
                0, 0, 1, 1, 1, 0, 0,
                0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0).finished();


    ASSERT_EQ((sdf - map_ground_truth).norm(), 0);
    
}


TEST(SDFHessian, initialization){
    // === The example on the TinyAD github ===

    // Init a 3D vector of active variables and a 3D vector of passive variables
    Eigen::Vector3<ADouble> x = ADouble::make_active({0.0, -1.0, 1.0});
    Eigen::Vector3<double> y(2.0, 3.0, 5.0);

    // Compute angle using Eigen functions and retrieve gradient and Hessian w.r.t. x
    ADouble angle = acos(x.dot(y) / (x.norm() * y.norm()));
    Eigen::Vector3d g = angle.grad;
    Eigen::Matrix3d H = angle.Hess;

    ei.print_matrix(H, "Hessian");
}

TEST(SDFHessian, Hessian){
    MatrixXd Sig_obs = ei.random_psd(3);
    VectorXd x = ei.random_vector(3);
    Eigen::Vector3<ADouble> x_ad = ADouble::make_active(x);
    ADouble f_ad = x_ad.transpose() * Sig_obs * x_ad;
    Eigen::Matrix3d H = f_ad.Hess;
    ASSERT_LE((H - Sig_obs*2).norm(), 1e-10);
    
}

TEST(SDFHessian, hinge_loss_gradient){
    MatrixXd Sig_obs = ei.random_psd(3);
    VectorXd x = ei.random_vector(3);
    Eigen::Vector3<ADouble> x_ad = ADouble::make_active(x);
    ADouble f_ad = x_ad.transpose() * Sig_obs * x_ad;
    Eigen::Matrix3d H = f_ad.Hess;
    ASSERT_LE((H - Sig_obs*2).norm(), 1e-10);
}