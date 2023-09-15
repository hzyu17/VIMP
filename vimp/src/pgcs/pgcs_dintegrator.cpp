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
// #include "pgcsmp/PGCSPlanarQuadSDF.h"
#include "3rdparty/rapidxml-1.13/rapidxml.hpp"
#include "3rdparty/rapidxml-1.13/rapidxml_utils.hpp"
// #include "helpers/ExperimentRunner.h"
#include "pgcsmp/PGCSNonLinDynPlanarSDF.h"

#define STRING(x) #x
#define XSTRING(x) STRING(x)

using namespace Eigen;
using namespace vimp;

int main(int argc, char* argv[]){

    MatrixIO m_io;
    EigenWrapper ei;
    VectorXd m0(4), mT(4);
    MatrixXd Sig0(4,4), SigT(4,4);
    int num_exp = 4;
    std::string file_name = "planar_pR_map2_BRM_demo";

    // arguments: num_exp, file_name
    if (argc == 3){
        num_exp = std::stoi(argv[1]);
        file_name = static_cast<std::string>(argv[2]);
    }

    std::string source_root{XSTRING(SOURCE_ROOT)};
    std::string config_file{source_root+"/configs/pgcs/" + file_name + ".xml"};
    std::cout<<"config file loaded from: "<<config_file<<std::endl;

    /// reading XML configs
    rapidxml::file<> xmlFile(config_file.data()); // Default template is char

//     rapidxml::file<> xmlFile("/home/hzyu/git/VIMP/vimp/configs/pgcs/planar_dintegrator_map2.xml"); // Default template is char
// >>>>>>> BRM
    // rapidxml::file<> xmlFile("/home/hzyu/git/VIMP/vimp/configs/pgcs/planar_dintegrator_map1.xml"); // Default template is char
    rapidxml::xml_document<> doc;
    doc.parse<0>(xmlFile.data());

    // define the data in Commons, following planar_pR_map2 definition
    rapidxml::xml_node<>* CommonNode = doc.first_node("Commons");
    int nt = atoi(CommonNode->first_node("nt")->value());
    double eta = atof(CommonNode->first_node("eta")->value());
    double stop_err = atof(CommonNode->first_node("stop_err")->value());
    double eps_sdf = atof(CommonNode->first_node("eps_sdf")->value());
    // TODO: Need to add in the Commons in xml file
    double speed = atof(CommonNode->first_node("speed")->value());
    std::string field_file = static_cast<std::string>(CommonNode->first_node("field_file")->value());

    using NominalHistory = std::tuple<std::vector<Matrix3D>, std::vector<Matrix3D>>;

    // construct sdf
    MatrixXd field = m_io.load_csv(field_file);
    gtsam::Point2 origin(-20, -10);
    double cell_size = 0.1;
    gpmp2::PlanarSDF sdf = gpmp2::PlanarSDF(origin, cell_size, field);
    
    // loop for 4 cases
    for (int i=1; i<= num_exp; i++){
        std::string ExpNodeName = "Experiment" + std::to_string(i);
        char * c_expname = ExpNodeName.data();
        rapidxml::xml_node<>* ExpNode = doc.first_node(c_expname);
        
        double sig_obs = atof(ExpNode->first_node("sig_obs")->value());
        double sig = speed * nt;

        // proximal gradient parameters
        double eps=0.01;
        int nx=4, nu=2;

        double start_x = atof(ExpNode->first_node("start_pos")->first_node("x")->value());
        double start_y = atof(ExpNode->first_node("start_pos")->first_node("y")->value());
        double start_vx = atof(ExpNode->first_node("start_pos")->first_node("vx")->value());
        double start_vy = atof(ExpNode->first_node("start_pos")->first_node("vy")->value());
        double goal_x = atof(ExpNode->first_node("goal_pos")->first_node("x")->value());
        double goal_y = atof(ExpNode->first_node("goal_pos")->first_node("y")->value());
        double goal_vx = atof(ExpNode->first_node("goal_pos")->first_node("vx")->value());
        double goal_vy = atof(ExpNode->first_node("goal_pos")->first_node("vy")->value());
        double sig0 = atof(ExpNode->first_node("sig0")->value());
        double sigT = atof(ExpNode->first_node("sigT")->value());

        m0 << start_x, start_y, start_vx, start_vy;
        Sig0 = sig0 * Eigen::MatrixXd::Identity(nx, nx);

        mT << goal_x, goal_y, goal_vx, goal_vy;
        SigT = sigT * Eigen::Matrix4d::Identity(nx, nx);

        MatrixXd A0(nx, nx), B(nx, nu), a0(nx, 1);
        std::shared_ptr<DoubleIntegrator> pdyn{new DoubleIntegrator(nx, nu, nt)};
        std::tuple<MatrixXd, MatrixXd, VectorXd, VectorXd> linearized_0 = pdyn->linearize_at(m0, A0, Sig0);
        A0  = std::get<0>(linearized_0);
        B   = std::get<1>(linearized_0);
        a0  = std::get<2>(linearized_0);

        PGCSNonLinDynPlanarSDF pgcs_sdf(A0, a0, B, sig, nt, eta, eps, m0, Sig0, mT, SigT, pdyn, eps_sdf, sdf, sig_obs, stop_err);
        
        std::tuple<MatrixXd, MatrixXd, NominalHistory> res_Kd;

        // double stop_err = 1e-4;
        // res_Kd = pgcs_sdf.optimize(stop_err);
        
        // res_Kd = pgcs_sdf.backtrack();
        res_Kd = pgcs_sdf.optimize();

        MatrixXd Kt(4*4, nt), dt(4, nt);
        Kt = std::get<0>(res_Kd);
        dt = std::get<1>(res_Kd);

        MatrixXd zk_star(4, nt), Sk_star(4*4, nt);
        zk_star = pgcs_sdf.zkt();
        Sk_star = pgcs_sdf.Sigkt();

        std::string saving_prefix = static_cast<std::string>(ExpNode->first_node("saving_prefix")->value());

        m_io.saveData(saving_prefix + std::string{"zk_sdf.csv"}, zk_star);
        m_io.saveData(saving_prefix + std::string{"Sk_sdf.csv"}, Sk_star);

        m_io.saveData(saving_prefix + std::string{"Kt_sdf.csv"}, Kt);
        m_io.saveData(saving_prefix + std::string{"dt_sdf.csv"}, dt);
    }

    return 0;

}