/**
 * @file pgcs_PRModel.cpp
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief pgcs with plannar obstacles and point robot, 
 * using Robot Model which has a vector of balls to check collisions.
 * @version 0.1
 * @date 2023-03-31
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "dynamics/LinearDynamics.h"
#include "covariance_steering/PGCSLinDynPRModelPlanarSDF.h"
#include "3rd-part/rapidxml-1.13/rapidxml.hpp"
#include "3rd-part/rapidxml-1.13/rapidxml_utils.hpp"

using namespace Eigen;
using namespace vimp;

int main(){
    int nx=4, nu=2;

    MatrixIO m_io;
    EigenWrapper ei;
    VectorXd m0(nx), mT(nx);
    MatrixXd Sig0(nx,nx), SigT(nx,nx);

    /// reading XML configs
    rapidxml::file<> xmlFile("/home/hongzhe/git/VIMP/vimp/configs/pgcs/planar_pR_map2.xml"); // Default template is char
    rapidxml::xml_document<> doc;
    doc.parse<0>(xmlFile.data());

    // Common parameters
    std::string CommonNodeName = "Commons";
    char * c_commons = CommonNodeName.data();
    rapidxml::xml_node<>* CommonNode = doc.first_node(c_commons);
    rapidxml::xml_node<>* commonParams = CommonNode->first_node("parameters");
    double eps_sdf = atof(commonParams->first_node("eps_sdf")->value());
    double speed = atof(commonParams->first_node("speed")->value());
    int nt = atoi(commonParams->first_node("nt")->value());

    double sig0 = atof(commonParams->first_node("sig0")->value());
    double sigT = atof(commonParams->first_node("sigT")->value());

    double eta = atof(commonParams->first_node("eta")->value());
    double stop_err = atof(commonParams->first_node("stop_err")->value());
    int max_iterations = atoi(commonParams->first_node("max_iter")->value());

    // loop for 4 cases
    for (int i=1; i<5; i++){
        
        std::string ExpNodeName = "Experiment" + std::to_string(i);
        char * c_expname = ExpNodeName.data();
        rapidxml::xml_node<>* ExpNode = doc.first_node(c_expname);
        rapidxml::xml_node<>* paramNode = ExpNode->first_node("parameters");

        std::string field_file = static_cast<std::string>(paramNode->first_node("field_file")->value());
        
        double sig_obs = atof(paramNode->first_node("cost_sigma")->value());
        
        double sig = speed * nt;

        // construct sdf
        MatrixXd field = m_io.load_csv(field_file);
        gtsam::Point2 origin(-20, -10);
        double cell_size = 0.1;
        gpmp2::PlanarSDF sdf = gpmp2::PlanarSDF(origin, cell_size, field);

        // PlanarPointRobotSDFPGCS pR_sdf(eps_sdf, sphere_r);
        // pR_sdf.update_sdf(sdf);
        
        // proximal gradient parameters
        double eps=0.01;

        double start_x = atof(paramNode->first_node("start_pos")->first_node("x")->value());
        double start_y = atof(paramNode->first_node("start_pos")->first_node("y")->value());

        double start_vx = atof(paramNode->first_node("start_pos")->first_node("vx")->value());
        double start_vy = atof(paramNode->first_node("start_pos")->first_node("vy")->value());

        double goal_x = atof(paramNode->first_node("goal_pos")->first_node("x")->value());
        double goal_y = atof(paramNode->first_node("goal_pos")->first_node("y")->value());

        double goal_vx = atof(paramNode->first_node("goal_pos")->first_node("vx")->value());
        double goal_vy = atof(paramNode->first_node("goal_pos")->first_node("vy")->value());


        m0 << start_x, start_y, start_vx, start_vy;
        Sig0 = sig0 * Eigen::MatrixXd::Identity(nx, nx);

        mT << goal_x, goal_y, goal_vx, goal_vy;
        SigT = sigT * Eigen::Matrix4d::Identity(nx, nx);

        MatrixXd A0(nx, nx), B0(nx, nu), a0(nx, 1);
        A0.setZero(); B0.setZero(); a0.setZero();
        
        std::shared_ptr<ConstantVelDynamics> pdyn{new ConstantVelDynamics(nx, nu, nt)};
        A0 = pdyn->A0() * sig;
        B0 = pdyn->B0() * sig;
        a0 = pdyn->a0() * sig;

        PGCSLinDynPRModelPlanarSDF pgcs_lin_sdf(A0, a0, B0, 
                                                sig, nt, eta, eps, 
                                                m0, Sig0, mT, SigT, 
                                                pdyn, eps_sdf, sig_obs, max_iterations);
        
        std::tuple<MatrixXd, MatrixXd> res_Kd;

        res_Kd = pgcs_lin_sdf.optimize(stop_err);

        MatrixXd Kt(nx*nx, nt), dt(nx, nt);
        Kt = std::get<0>(res_Kd);
        dt = std::get<1>(res_Kd);

        MatrixXd zk_star(nx, nt), Sk_star(nx*nx, nt);
        zk_star = pgcs_lin_sdf.zkt();
        Sk_star = pgcs_lin_sdf.Sigkt();

        std::string saving_prefix = static_cast<std::string>(paramNode->first_node("saving_prefix")->value());

        m_io.saveData(saving_prefix + std::string{"zk_sdf.csv"}, zk_star);
        m_io.saveData(saving_prefix + std::string{"Sk_sdf.csv"}, Sk_star);

        m_io.saveData(saving_prefix + std::string{"Kt_sdf.csv"}, Kt);
        m_io.saveData(saving_prefix + std::string{"dt_sdf.csv"}, dt);

    }
    

}