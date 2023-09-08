/**
 * @file pgcs_dintegrator.cpp
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Test file for pgcs with plannar obstacles (plannar sdf).
 * @version 0.1
 * @date 2023-03-27
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "dynamics/DoubleIntegratorDraged.h"
#include "pgcsmp/PGCSPlanarQuadSDF.h"
#include "3rdparty/rapidxml-1.13/rapidxml.hpp"
#include "3rdparty/rapidxml-1.13/rapidxml_utils.hpp"

using namespace Eigen;
using namespace vimp;

int main(){

    // MatrixIO m_io;
    // EigenWrapper ei;
    // VectorXd m0(4), mT(4);
    // MatrixXd Sig0(4,4), SigT(4,4);

    // /// reading XML configs
    // rapidxml::file<> xmlFile("/home/hzyu/git/VIMP/vimp/configs/pgcs/planar_dintegrator_map2.xml"); // Default template is char
    // // rapidxml::file<> xmlFile("/home/hzyu/git/VIMP/vimp/configs/pgcs/planar_dintegrator_map1.xml"); // Default template is char
    // rapidxml::xml_document<> doc;
    // doc.parse<0>(xmlFile.data());
    
    // // loop for 4 cases
    // for (int i=1; i<5; i++){
    //     std::string ExpNodeName = "Experiment" + std::to_string(i);
    //     char * c_expname = ExpNodeName.data();
    //     rapidxml::xml_node<>* ExpNode = doc.first_node(c_expname);
    //     rapidxml::xml_node<>* paramNode = ExpNode->first_node("parameters");

    //     std::string field_file = static_cast<std::string>(paramNode->first_node("field_file")->value());
    //     double eps_sdf = atof(paramNode->first_node("eps_sdf")->value());
    //     double speed = atof(paramNode->first_node("speed")->value());
        
    //     int nt = atoi(paramNode->first_node("nt")->value());

    //     double sig = speed * nt;

    //     std::cout << "speed " << speed << std::endl <<
    //     "nt " << nt << std::endl << 
    //     "sig " << sig << std::endl;

    //     // construct sdf
    //     MatrixXd field = m_io.load_csv(field_file);
    //     gtsam::Point2 origin(-20, -10);
    //     double cell_size = 0.1;
    //     gpmp2::PlanarSDF sdf = gpmp2::PlanarSDF(origin, cell_size, field);

    //     // proximal gradient parameters
    //     double eps=0.01;
    //     int nx=4, nu=2;

    //     double start_x = atof(paramNode->first_node("start_pos")->first_node("x")->value());
    //     double start_y = atof(paramNode->first_node("start_pos")->first_node("y")->value());

    //     double start_vx = atof(paramNode->first_node("start_pos")->first_node("vx")->value());
    //     double start_vy = atof(paramNode->first_node("start_pos")->first_node("vy")->value());

    //     double goal_x = atof(paramNode->first_node("goal_pos")->first_node("x")->value());
    //     double goal_y = atof(paramNode->first_node("goal_pos")->first_node("y")->value());

    //     double goal_vx = atof(paramNode->first_node("goal_pos")->first_node("vx")->value());
    //     double goal_vy = atof(paramNode->first_node("goal_pos")->first_node("vy")->value());

    //     double sig0 = atof(paramNode->first_node("sig0")->value());
    //     double sigT = atof(paramNode->first_node("sigT")->value());

    //     m0 << start_x, start_y, start_vx, start_vy;
    //     Sig0 = sig0 * Eigen::MatrixXd::Identity(nx, nx);

    //     mT << goal_x, goal_y, goal_vx, goal_vy;
    //     SigT = sigT * Eigen::Matrix4d::Identity(nx, nx);

    //     MatrixXd A0(nx, nx), B(nx, nu), a0(nx, 1);
    //     std::shared_ptr<DoubleIntegrator> pdyn{new DoubleIntegrator(nx, nu, nt)};
    //     std::tuple<MatrixXd, MatrixXd, VectorXd, VectorXd> linearized_0 = pdyn->linearize_at(m0, sig, A0, Sig0);
    //     A0  = std::get<0>(linearized_0);
    //     B   = std::get<1>(linearized_0);
    //     a0  = std::get<2>(linearized_0);

    //     double eta = atof(paramNode->first_node("eta")->value());
    //     double sig_obs = atof(paramNode->first_node("cost_sigma")->value());
    //     double Vscale = atof(paramNode->first_node("state_cost_scale")->value());
    //     PGCSNLDynPlanarSDF pgcs_sdf(A0, a0, B, sig, nt, eta, eps, m0, Sig0, mT, SigT, pdyn, eps_sdf, sdf, sig_obs, Vscale);
        
    //     std::tuple<MatrixXd, MatrixXd> res_Kd;

    //     double stop_err = 1e-4;
    //     res_Kd = pgcs_sdf.optimize(stop_err);

    //     MatrixXd Kt(4*4, nt), dt(4, nt);
    //     Kt = std::get<0>(res_Kd);
    //     dt = std::get<1>(res_Kd);

    //     MatrixXd zk_star(4, nt), Sk_star(4*4, nt);
    //     zk_star = pgcs_sdf.zkt();
    //     Sk_star = pgcs_sdf.Sigkt();

    //     std::string saving_prefix = static_cast<std::string>(paramNode->first_node("saving_prefix")->value());

    //     m_io.saveData(saving_prefix + std::string{"zk_sdf.csv"}, zk_star);
    //     m_io.saveData(saving_prefix + std::string{"Sk_sdf.csv"}, Sk_star);

    //     m_io.saveData(saving_prefix + std::string{"Kt_sdf.csv"}, Kt);
    //     m_io.saveData(saving_prefix + std::string{"dt_sdf.csv"}, dt);
    // }

    return 0;

}