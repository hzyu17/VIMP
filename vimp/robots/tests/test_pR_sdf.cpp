/**
 * @file test_pR_sdf.cpp
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Test the PointRobotModel and sdf class.
 * @version 0.1
 * @date 2023-04-02
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <gtest/gtest.h>
#include "robots/PlanarPointRobotSDF_pgcs.h"
#include "3rdparty/rapidxml-1.13/rapidxml.hpp"
#include "3rdparty/rapidxml-1.13/rapidxml_utils.hpp"
#include "../../helpers/eigen_wrapper.h"

using namespace rapidxml;
using namespace vimp;
using namespace Eigen;

using SDF = gpmp2::PlanarSDF;

EigenWrapper ei;

// global function
std::tuple<SDF, double, double, double> get_sdf(){  
    MatrixIO m_io;
    rapidxml::file<> xmlFile("/home/hyu419/git/VIMP/vimp/configs/pgcs/planar_pR_map2.xml"); // Default template is char
    rapidxml::xml_document<> doc;
    doc.parse<0>(xmlFile.data());
    std::string ExpNodeName = "Experiment4";
    char * c_expname = ExpNodeName.data();
    rapidxml::xml_node<>* ExpNode = doc.first_node(c_expname);
    rapidxml::xml_node<>* paramNode = ExpNode->first_node("parameters");
    double eps_sdf = atof(paramNode->first_node("eps_sdf")->value());
    double sphere_r = atof(paramNode->first_node("robot_sphere_r")->value());
    double cost_sigma = atof(paramNode->first_node("cost_sigma")->value());
    std::string field_file = static_cast<std::string>(paramNode->first_node("field_file")->value());

    // construct sdf
    MatrixXd field = m_io.load_csv(field_file);
    gtsam::Point2 origin(-20, -10);
    double cell_size = 0.1;
    SDF sdf(origin, cell_size, field);

    return std::make_tuple(sdf, cost_sigma, eps_sdf, sphere_r);
}


TEST(TestPRSDF, initialization){
    std::tuple<SDF, double, double, double> res;
    res = get_sdf();
    SDF sdf = std::get<0>(res);
    double cost_sigma = std::get<1>(res);
    double eps_sdf = std::get<2>(res);
    double sphere_r = std::get<3>(res);
    // construct the robot model with sdf
    PlanarPointRobotSDFPGCS pr_sdf{eps_sdf, sphere_r};
    pr_sdf.update_sdf(sdf);
}

TEST(TestPRSDF, pose_cost_jacobian){
    std::tuple<SDF, double, double, double> res;
    res = get_sdf();
    SDF sdf = std::get<0>(res);
    double cost_sigma = std::get<1>(res);
    double eps_sdf = std::get<2>(res);
    double sphere_r = std::get<3>(res);
    // construct the robot model with sdf
    PlanarPointRobotSDFPGCS pr_sdf{eps_sdf, sphere_r};
    pr_sdf.update_sdf(sdf);

    std::cout << "eps_sdf " << eps_sdf << std::endl << "sphere_r " << sphere_r << std::endl; 

    VectorXd pose(4);
    pose << -6.29201, 9.84276, -4.14218, -1.59315;
    double hinge_gt = 0.04276;
    
    int num_spheres = pr_sdf.pRmodel().nr_body_spheres();
    MatrixXd Jacobian_gt(num_spheres, 2);
    Jacobian_gt << -0, 1;

    std::tuple<VectorXd, MatrixXd> hinge_jac;
    hinge_jac = pr_sdf.hinge_jac(pose.block(0,0,2,1));
    VectorXd hinge = std::get<0>(hinge_jac);
    MatrixXd Jacobian = std::get<1>(hinge_jac);
    VectorXd sphere_center = pr_sdf.pRmodel().sphereCenter(0, pose);
    ei.print_matrix(sphere_center, "sphere_center");
    ei.print_matrix(hinge, "hinge 1");
    ei.print_matrix(Jacobian, "Jacobian 1");

    ASSERT_LE((Jacobian - Jacobian_gt).norm(), 1e-6);
    ASSERT_LE(abs(hinge(0)-hinge_gt), 1e-6);

    VectorXd pose1(4);
    pose1 << 7.93583, 16.4514, -8.00919, 1.5954;
    hinge_gt = 0.0486;
    MatrixXd Jacobian_gt1(num_spheres, 2);
    Jacobian_gt1 << -0, -1;

    hinge_jac = pr_sdf.hinge_jac(pose1.block(0,0,2,1));
    hinge = std::get<0>(hinge_jac);
    Jacobian = std::get<1>(hinge_jac);

    ASSERT_LE((Jacobian - Jacobian_gt1).norm(), 1e-6);
    ASSERT_LE(abs(hinge(0)-hinge_gt), 1e-6);

    ei.print_matrix(hinge, "hinge 2");
    ei.print_matrix(Jacobian, "Jacobian 2");
}