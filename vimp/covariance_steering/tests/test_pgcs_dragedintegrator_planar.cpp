/**
 * @file test_pgcs_plannar_sdf.cpp
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Test file for pgcs with plannar obstacles (plannar sdf). 
 * Modification to file vimp/covariance_steering/PGCSPlanarSDF.h should pass this test. 
 * @version 0.1
 * @date 2023-03-27
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <gtest/gtest.h>
#include "dynamics/DoubleIntegratorDraged.h"
#include "covariance_steering/PGCSPlanarSDF.h"
#include "3rdparty/rapidxml-1.13/rapidxml.hpp"
#include "3rdparty/rapidxml-1.13/rapidxml_utils.hpp"

using namespace Eigen;
using namespace vimp;

TEST(TestPgcsSdf2D, initialization){
    MatrixIO m_io;
    EigenWrapper ei;
    VectorXd m0(4), mT(4);
    MatrixXd Sig0(4,4), SigT(4,4);

    /// reading XML configs
    rapidxml::file<> xmlFile("/home/hongzhe/git/VIMP/vimp/covariance_steering/tests/data/test_config_dintegrator_map2.xml"); // Default template is char
    rapidxml::xml_document<> doc;
    doc.parse<0>(xmlFile.data());
    rapidxml::xml_node<>* paramNode = doc.first_node("parameters");

    std::string field_file = static_cast<std::string>(paramNode->first_node("field_file")->value());

    double start_x = atof(paramNode->first_node("start_pos")->first_node("x")->value());
    double start_y = atof(paramNode->first_node("start_pos")->first_node("y")->value());

    double goal_x = atof(paramNode->first_node("goal_pos")->first_node("x")->value());
    double goal_y = atof(paramNode->first_node("goal_pos")->first_node("y")->value());
    double sig0 = atof(paramNode->first_node("sig0")->value());
    double sigT = atof(paramNode->first_node("sigT")->value());
    double eps_sdf = atof(paramNode->first_node("eps_sdf")->value());
    double speed = atof(paramNode->first_node("speed")->value());
    double sig_obs = atof(paramNode->first_node("cost_sigma")->value());
    double eta = atof(paramNode->first_node("eta")->value());
    int nt = atoi(paramNode->first_node("nt")->value());
    double Vscale = atof(paramNode->first_node("state_cost_scale")->value());

    double sig = speed * nt;

    // construct sdf
    MatrixXd field = m_io.load_csv(field_file);
    gtsam::Point2 origin(-20, -10);
    double cell_size = 0.1;
    gpmp2::PlanarSDF sdf = gpmp2::PlanarSDF(origin, cell_size, field);

    // proximal gradient parameters
    double eps=0.01;
    int nx=4, nu=2;

    m0 << start_x, start_y, 2, 0;
    Sig0 = sig0 * Eigen::MatrixXd::Identity(nx, nx);

    mT << goal_x, goal_y, -1, 0;
    SigT = sigT * Eigen::Matrix4d::Identity(nx, nx);

    MatrixXd A0(nx, nx), B(nx, nu), a0(nx, 1);
    std::shared_ptr<DoubleIntegrator> pdyn{new DoubleIntegrator(nx, nu, nt)};
    std::tuple<MatrixXd, MatrixXd, VectorXd, VectorXd> linearized_0 = pdyn->linearize_at(m0, sig, A0, Sig0);
    A0  = std::get<0>(linearized_0);
    B   = std::get<1>(linearized_0);
    a0  = std::get<2>(linearized_0);

    PGCSPlanarSDF pgcs_sdf(A0, a0, B, sig, nt, eta, eps, m0, Sig0, mT, SigT, pdyn, eps_sdf, sdf, sig_obs, Vscale);
    
    std::tuple<MatrixXd, MatrixXd> res_Kd;
    double stop_err = 1e-4;
    res_Kd = pgcs_sdf.optimize(stop_err);

    MatrixXd Kt(4*4, nt), dt(4, nt);
    Kt = std::get<0>(res_Kd);
    dt = std::get<1>(res_Kd);

    MatrixXd zk_star(4, nt), Sk_star(4*4, nt);
    zk_star = pgcs_sdf.zkt();
    Sk_star = pgcs_sdf.Sigkt();

    MatrixXd Kt_gt(4*4, nt), dt_gt(4, nt), zkt_gt(4, nt), Skt_gt(4*4, nt);
    Kt_gt = m_io.load_csv("/home/hongzhe/git/VIMP/vimp/covariance_steering/tests/data/Kt_sdf.csv");
    dt_gt = m_io.load_csv("/home/hongzhe/git/VIMP/vimp/covariance_steering/tests/data/dt_sdf.csv");
    zkt_gt = m_io.load_csv("/home/hongzhe/git/VIMP/vimp/covariance_steering/tests/data/zk_sdf.csv");
    Skt_gt = m_io.load_csv("/home/hongzhe/git/VIMP/vimp/covariance_steering/tests/data/Sk_sdf.csv");

    ASSERT_LE((Kt-Kt_gt).norm(), 1e-10);
    ASSERT_LE((dt-dt_gt).norm(), 1e-10);
    ASSERT_LE((zk_star-zkt_gt).norm(), 1e-10);
    ASSERT_LE((Sk_star-Skt_gt).norm(), 1e-10);


}