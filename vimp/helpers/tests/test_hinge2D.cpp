/**
 * @file test_AD.cpp
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Test automatic differentiation tools.
 * @version 0.1
 * @date 2023-04-05
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <gtest/gtest.h>
#include <fastad>
#include "../hinge2Dhelper.h"
#include <gpmp2/obstacle/PlanarSDF.h>
#include <matplot/matplot.h>

using namespace vimp;
using namespace Eigen;

EigenWrapper ei;

TEST(TestHinge2D, hinge_loss){
    vimp::MatrixIO m_io;
    MatrixXd grid_X = m_io.load_csv("/home/hongzhe/git/VIMP/vimp/data/sdf_grid_x.csv");
    MatrixXd grid_Y = m_io.load_csv("/home/hongzhe/git/VIMP/vimp/data/sdf_grid_y.csv");

    MatrixXd field = m_io.load_csv("/home/hongzhe/git/VIMP/vimp/data/vimp/2d_pR/field_multiobs_entropy.csv");

    Vector2d origin(-20, -10);
    double cell_size = 0.1;
    double eps = 2.0;

    gpmp2::PlanarSDF sdf = gpmp2::PlanarSDF(origin, cell_size, field);  
    
    MatrixXd hinge_loss = mesh_hingeloss(grid_X, grid_Y, sdf, eps);
    MatrixXd hinge_loss_groundtruth = m_io.load_csv("/home/hongzhe/git/VIMP/vimp/data/vimp/2d_pR/map_multiobs_entropy_hinge_loss_groundtruth.csv");
    ASSERT_LE((hinge_loss - hinge_loss_groundtruth).norm(), 1e-10);

}


TEST(TestHinge2D, hinge_loss_gradients){
    vimp::MatrixIO m_io;
    MatrixXd grid_X = m_io.load_csv("/home/hongzhe/git/VIMP/vimp/data/sdf_grid_x.csv");
    MatrixXd grid_Y = m_io.load_csv("/home/hongzhe/git/VIMP/vimp/data/sdf_grid_y.csv");

    MatrixXd field = m_io.load_csv("/home/hongzhe/git/VIMP/vimp/data/vimp/2d_pR/field_multiobs_entropy.csv");

    Vector2d origin(-20, -10);
    double cell_size = 0.1;
    double eps = 2.0;

    gpmp2::PlanarSDF sdf = gpmp2::PlanarSDF(origin, cell_size, field); 
    MatrixXd Jacobians;

    std::pair<MatrixXd, MatrixXd> hingeloss_gradient;

    using vec_1d = std::vector<double>;
    using vec_2d = std::vector<vec_1d>;
    std::tuple<MatrixXd, vec_2d, vec_2d> results;
    results = hingeloss_gradient_mesh(grid_X, grid_Y, sdf, eps, Jacobians);

    // plot mesh hinge loss and gradients
    using namespace matplot;
    int xlen = 400;
    int ylen = 300;
    MatrixXd x_mesh = grid_X.transpose().replicate(1, ylen);
    MatrixXd y_mesh = grid_Y.replicate(xlen, 1);
    
    vector_2d x = ei.eigen_to_vector(x_mesh);
    vector_2d y = ei.eigen_to_vector(y_mesh);

    // hinge loss at a point
    MatrixXd Jacobian = MatrixXd::Zero(1, 2);
    std::pair<double, MatrixXd> hinge_jacobian_point = hingeloss_gradient_point(-15.0, 0.0, sdf, eps, Jacobian);

    // ei.print_matrix(hinge_jacobian_point.second, "hinge loss jacobian at (-15.0, 0.0)");

    // plot hinge loss
    figure();
    MatrixXd hinge_loss = std::get<0>(results);
    vector_2d hinge_vec = ei.eigen_to_vector(hinge_loss);
    surf(x, y, hinge_vec);

    save("data/hinge_loss", "jpeg");
    
    // hinge loss jacobians
    figure();
    vec_2d grad_x_mesh = std::get<1>(results);
    vec_2d grad_y_mesh = std::get<2>(results);
    quiver(x, y, grad_x_mesh, grad_y_mesh, 4.0);
    save("data/hinge_loss_jacobians", "jpeg");


    show();
    
}
