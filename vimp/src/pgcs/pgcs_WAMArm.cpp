/**
 * @file pgcs_pR_RobotModel.cpp
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief pgcs with plannar obstacles and arm robot, 
 * using Robot Model which has a vector of balls to check collisions.
 * @version 0.1
 * @date 2023-03-31
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "dynamics/LinearDynamics.h"
#include "covariance_steering/PGCSLinDynArmSDF.h"
#include "3rd-part/rapidxml-1.13/rapidxml.hpp"
#include "3rd-part/rapidxml-1.13/rapidxml_utils.hpp"

using namespace Eigen;
using namespace vimp;

int main(){
    int nx = 14, nu=7;
    MatrixIO m_io;
    EigenWrapper ei;
    VectorXd m0(nx), mT(nx);
    MatrixXd Sig0(nx,nx), SigT(nx,nx);

    /// reading XML configs
    rapidxml::file<> xmlFile("/home/hongzhe/git/VIMP/vimp/configs/pgcs/wam_arm_Hess.xml"); // Default template is char
    rapidxml::xml_document<> doc;
    doc.parse<0>(xmlFile.data());

    // loop for 4 cases
    for (int i=1; i<4; i++){
        std::string ExpNodeName = "Experiment" + std::to_string(i);
        char * c_expname = ExpNodeName.data();
        rapidxml::xml_node<>* ExpNode = doc.first_node(c_expname);
        rapidxml::xml_node<>* paramNode = ExpNode->first_node("parameters");

        std::string sdf_file = static_cast<std::string>(paramNode->first_node("sdf_file")->value());
        double eps_sdf = atof(paramNode->first_node("eps_sdf")->value());
        double speed = atof(paramNode->first_node("speed")->value());
        double sig_obs = atof(paramNode->first_node("cost_sigma")->value());
        int max_iter = atof(paramNode->first_node("max_iter")->value());
        int nt = atoi(paramNode->first_node("nt")->value());

        double sig = speed * nt;

        // construct sdf
        gpmp2::SignedDistanceField sdf = SDF();
        sdf.loadSDF(sdf_file);
        
        // proximal gradient parameters
        double eps=0.01;

        double start_1 = atof(paramNode->first_node("start_pos")->first_node("1")->value());
        double start_2 = atof(paramNode->first_node("start_pos")->first_node("2")->value());
        double start_3 = atof(paramNode->first_node("start_pos")->first_node("3")->value());
        double start_4 = atof(paramNode->first_node("start_pos")->first_node("4")->value());
        double start_5 = atof(paramNode->first_node("start_pos")->first_node("5")->value());
        double start_6 = atof(paramNode->first_node("start_pos")->first_node("6")->value());
        double start_7 = atof(paramNode->first_node("start_pos")->first_node("7")->value());
        
        double goal_1 = atof(paramNode->first_node("goal_pos")->first_node("1")->value());
        double goal_2 = atof(paramNode->first_node("goal_pos")->first_node("2")->value());
        double goal_3 = atof(paramNode->first_node("goal_pos")->first_node("3")->value());
        double goal_4 = atof(paramNode->first_node("goal_pos")->first_node("4")->value());
        double goal_5 = atof(paramNode->first_node("goal_pos")->first_node("5")->value());
        double goal_6 = atof(paramNode->first_node("goal_pos")->first_node("6")->value());
        double goal_7 = atof(paramNode->first_node("goal_pos")->first_node("7")->value());

        double sig0 = atof(paramNode->first_node("sig0")->value());
        double sigT = atof(paramNode->first_node("sigT")->value());

        Eigen::VectorXd m0_pos(nx/2);
        m0_pos << start_1, start_2, start_3, start_4, start_5, start_6, start_7;
        Sig0 = sig0 * Eigen::MatrixXd::Identity(nx, nx);
        m0.block(0,0,nx/2,1) = m0_pos;
        m0.block(nx/2,0,nx/2,1) = Eigen::VectorXd::Zero(nx/2);
        Eigen::VectorXd mT_pos(nx/2);
        mT_pos << goal_1, goal_2, goal_3, goal_4, goal_5, goal_6, goal_7;
        SigT = sigT * Eigen::MatrixXd::Identity(nx, nx);
        mT.block(0,0,nx/2,1) = mT_pos;
        mT.block(nx/2,0,nx/2,1) = Eigen::VectorXd::Zero(nx/2);
        
        MatrixXd A0(nx, nx), B0(nx, nu), a0(nx, 1);
        A0.setZero(); B0.setZero(); a0.setZero();

        std::shared_ptr<ConstantVelDynamics> pdyn{new ConstantVelDynamics(nx, nu, nt)};
        A0 = pdyn->A0() * sig;
        B0 = pdyn->B0() * sig;
        a0 = pdyn->a0() * sig;

        double eta = atof(paramNode->first_node("eta")->value());
        double Vscale = atof(paramNode->first_node("state_cost_scale")->value());
        PGCSLinDynArmSDF pgcs_lin_sdf(A0, a0, B0, sig, nt, eta, eps, m0, Sig0, mT, SigT, pdyn, eps_sdf, sdf, sig_obs, Vscale, max_iter);

        std::tuple<MatrixXd, MatrixXd> res_Kd;

        double stop_err = atof(paramNode->first_node("stop_err")->value());
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