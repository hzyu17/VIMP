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
#include "../EigenWrapper.h"
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
    sdf.loadSDF("/home/hongzhe/git/VIMP/matlab_helpers/PGCS-examples/3dSDFs/pRSDF3D.bin");
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

    VectorXd pos(3);
    pos << 15.1175, 15.1175, 15.1175;
    double hinge_pos;
    hinge_pos = gpmp2::hingeLossObstacleCost(pos, sdf, eps);
    std::cout << "hinge_pos " << std::endl << hinge_pos << std::endl;
    // save hinge loss
    m_io.saveData("mesh_hinge3D.csv", mesh_hinge);
    // hinge loss jacobians
    figure();
    quiver3(x, y, z, grad_x, grad_y, grad_z)->normalize(false).line_width(2);
    save("data/hingeloss_jacobians_3D", "jpeg");

    show();
    
}


#include <gtsam/inference/Symbol.h>
#include <gpmp2/kinematics/PointRobotModel.h>
#include <gpmp2/obstacle/ObstacleSDFFactor.h>
#include <gpmp2/obstacle/SignedDistanceField.h>
/**
 * @brief Factor for 3D point robot and 3D SDF does not work! 
 * because gpmp2 only defines 2D point robot kinematics.
 */
TEST(TestHinge3D, ObstacleFactor3DpR){
    using pRSDF3D = gpmp2::ObstacleSDFFactor<gpmp2::PointRobotModel>;
    using SDF = gpmp2::SignedDistanceField;
    SDF sdf = SDF();
    sdf.loadSDF("/home/hongzhe/git/VIMP/matlab_helpers/PGCS-examples/3dSDFs/pRSDF3D.bin");

    gpmp2::PointRobot pR(3, 1);
    gpmp2::BodySphereVector body_spheres;
    body_spheres.push_back(gpmp2::BodySphere(0, 0, Point3(0.0, 0.0, 0.0)));
    gpmp2::PointRobotModel pR_model = gpmp2::PointRobotModel(pR, body_spheres);

    std::shared_ptr<pRSDF3D> _p_sdf_factor = std::make_shared<pRSDF3D>(pRSDF3D(gtsam::symbol('x', 0), pR_model, sdf, 0.0, 0.7));
    
    VectorXd pos(3), hinge;
    pos << 15.1175, 15.1175, 15.1175;
    hinge = _p_sdf_factor->evaluateError(pos);
    ei.print_matrix(pos, "pos");
    
    std::vector<gtsam::Point3> sphere_centers(1);
    pR_model.sphereCenters(pos, sphere_centers);

    for (auto cter: sphere_centers){
        ei.print_matrix(cter, "cter");
        ASSERT_GE(cter(2), 0);
    }

    ei.print_matrix(hinge, "hinge");

}
