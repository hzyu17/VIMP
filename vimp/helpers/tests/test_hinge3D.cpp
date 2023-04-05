/**
 * @file test_hinge_3D.cpp
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Test plotting the hinge loss and gradients of a 3D SDF.
 * @version 0.1
 * @date 2023-04-05
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <gtest/gtest.h>
#include <gpmp2/obstacle/SignedDistanceField.h>
#include <gpmp2/obstacle/ObstacleCost.h>
#include "../eigen_wrapper.h"
#include "../hinge3Dhelper.h"
#include <matplot/matplot.h>

using namespace vimp;
using namespace Eigen;

EigenWrapper ei;

TEST(TestHinge3D, mesh_hinge_gradient){
    vimp::MatrixIO m_io;
    VectorXd mesh_X = m_io.load_csv("/home/hongzhe/git/VIMP/matlab_helpers/PGCS-examples/3d_pR/gridX.csv");
    VectorXd mesh_Y = m_io.load_csv("/home/hongzhe/git/VIMP/matlab_helpers/PGCS-examples/3d_pR/gridY.csv");
    VectorXd mesh_Z = m_io.load_csv("/home/hongzhe/git/VIMP/matlab_helpers/PGCS-examples/3d_pR/gridZ.csv");

    int len_mesh = mesh_X.cols();
    // mesh_X.reshaped(len_mesh, 1);
    // mesh_Y.reshaped(len_mesh, 1);
    // mesh_Z.reshaped(len_mesh, 1);
    
    gpmp2::SignedDistanceField sdf = gpmp2::SignedDistanceField();
    sdf.loadSDF("/home/hongzhe/git/VIMP/matlab_helpers/PGCS-examples/3d_pR/pRSDF3D.bin");
    double eps = 0.2;
    using vec_1d = std::vector<double>;

    std::tuple<VectorXd, MatrixXd> results;

    results = mesh3D_hinge_gradient(mesh_X, mesh_Y, mesh_Z, sdf, eps);

    VectorXd mesh_hinge(len_mesh);
    mesh_hinge = std::get<0>(results);
    MatrixXd mesh_Jacobians(3, len_mesh);
    mesh_Jacobians = std::get<1>(results);

    // plot mesh hinge loss and gradients
    // https://github.com/alandefreitas/matplotplusplus/blob/master/examples/vector_fields/quiver3/quiver3_2.cpp
    using namespace matplot;   

    vec_1d x = ei.VectorXd_to_vector(mesh_X);
    vec_1d y = ei.VectorXd_to_vector(mesh_Y);
    vec_1d z = ei.VectorXd_to_vector(mesh_Z);
    vec_1d hinge = ei.VectorXd_to_vector(mesh_hinge);
    vec_1d grad_x = ei.VectorXd_to_vector(mesh_Jacobians.row(0).transpose());
    vec_1d grad_y = ei.VectorXd_to_vector(mesh_Jacobians.row(1).transpose());
    vec_1d grad_z = ei.VectorXd_to_vector(mesh_Jacobians.row(2).transpose());

    // hinge loss jacobians
    figure();
    quiver3(x, y, z, grad_x, grad_y, grad_z)->normalize(false).line_width(2);
    save("data/hingeloss_jacobians_3D", "jpeg");

    show();
    
}
