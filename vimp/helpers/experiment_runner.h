/**
 * @file run_experiment_template.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief The template of running a motion planning experiment.
 * @version 0.1
 * @date 2023-04-14
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "Eigen/Dense"
#include "../helpers/data_io.h"
#include "../helpers/eigen_wrapper.h"
#include "../3rd-part/rapidxml-1.13/rapidxml.hpp"
#include "../3rd-part/rapidxml-1.13/rapidxml_utils.hpp"

using namespace Eigen;

namespace vimp{

template <typename PGCSOptimizer>
class ExperimentRunner{
public:
    ExperimentRunner(int nx, int nu, int num_exp, const std::string & config): 
                                            _nx(nx),
                                            _nu(nu),
                                            _num_exp(num_exp),
                                            m0(nx), 
                                            mT(nx),
                                            Sig0(nx,nx), 
                                            SigT(nx,nx),
                                            _config_file{config}{}

    void read_config_file(){
        rapidxml::file<> xmlFile(_config_file.data()); // Default template is char
        rapidxml::xml_document<> doc;
        doc.parse<0>(xmlFile.data());

        // Common parameters
        std::string CommonNodeName = "Commons";
        char * c_commons = CommonNodeName.data();
        rapidxml::xml_node<>* CommonNode = doc.first_node(c_commons);
        rapidxml::xml_node<>* commonParams = CommonNode->first_node("parameters");

        eps_sdf = atof(commonParams->first_node("eps_sdf")->value());
        speed = atof(commonParams->first_node("speed")->value());
        nt = atoi(commonParams->first_node("nt")->value());

        sig0 = atof(commonParams->first_node("sig0")->value());
        sigT = atof(commonParams->first_node("sigT")->value());

        eta = atof(commonParams->first_node("eta")->value());
        stop_err = atof(commonParams->first_node("stop_err")->value());
        max_iterations = atoi(commonParams->first_node("max_iter")->value());

    }

    void run(){
        rapidxml::file<> xmlFile(_config_file.data()); // Default template is char
        rapidxml::xml_document<> doc;
        doc.parse<0>(xmlFile.data());

        for (int i=1; i<_num_exp+1; i++){
        
            std::string ExpNodeName = "Experiment" + std::to_string(i);
            char * c_expname = ExpNodeName.data();
            rapidxml::xml_node<>* ExpNode = doc.first_node(c_expname);

            rapidxml::xml_node<>* paramNode = ExpNode->first_node("parameters");            
            double sig_obs = atof(paramNode->first_node("cost_sigma")->value());
            
            double sig = speed * nt;

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
            Sig0 = sig0 * Eigen::MatrixXd::Identity(_nx, _nx);

            mT << goal_x, goal_y, goal_vx, goal_vy;
            SigT = sigT * Eigen::Matrix4d::Identity(_nx, _nx);

            MatrixXd A0(_nx, _nx), B0(_nx, _nu), a0(_nx, 1);
            A0.setZero(); B0.setZero(); a0.setZero();
            
            std::shared_ptr<ConstantVelDynamics> pdyn{new ConstantVelDynamics(_nx, _nu, nt)};
            A0 = pdyn->A0() * sig;
            B0 = pdyn->B0() * sig;
            a0 = pdyn->a0() * sig;

            PGCSOptimizer pgcs_lin_sdf(A0, a0, B0, 
                                        sig, nt, eta, eps, 
                                        m0, Sig0, mT, SigT, 
                                        pdyn, eps_sdf, sig_obs, max_iterations);
            
            std::tuple<MatrixXd, MatrixXd> res_Kd;

            res_Kd = pgcs_lin_sdf.optimize(stop_err);

            MatrixXd Kt(_nx*_nx, nt), dt(_nx, nt);
            Kt = std::get<0>(res_Kd);
            dt = std::get<1>(res_Kd);

            MatrixXd zk_star(_nx, nt), Sk_star(_nx*_nx, nt);
            zk_star = pgcs_lin_sdf.zkt();
            Sk_star = pgcs_lin_sdf.Sigkt();

            std::string saving_prefix = static_cast<std::string>(paramNode->first_node("saving_prefix")->value());

            m_io.saveData(saving_prefix + std::string{"zk_sdf.csv"}, zk_star);
            m_io.saveData(saving_prefix + std::string{"Sk_sdf.csv"}, Sk_star);

            m_io.saveData(saving_prefix + std::string{"Kt_sdf.csv"}, Kt);
            m_io.saveData(saving_prefix + std::string{"dt_sdf.csv"}, dt);

        }
    }


private:
    
    MatrixIO m_io;
    EigenWrapper ei;
    VectorXd m0, mT;
    MatrixXd Sig0, SigT;

    double speed, eps_sdf, sig0, sigT, eta, stop_err;
    int nt, max_iterations, _num_exp;
    int _nx, _nu;

    std::string _config_file;

};


}