/**
 * @file ExperimentRunner.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief The template of running a motion planning experiment.
 * Including reading a configuration file and run an experiment. 
 * It takes in 
 * 1. an optimizer class
 * 2. a parameter class which defines the experiment hyperparameter types.
 * GVIExperimentRunner and PGCSExperimentRunner are 2 main derived classes.
 * @version 0.1
 * @date 2023-04-14
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "helpers/ExperimentParams.h"
#include "dynamics/LinearDynamics.h"
#include "3rdparty/rapidxml-1.13/rapidxml.hpp"
#include "3rdparty/rapidxml-1.13/rapidxml_utils.hpp"
#include <memory>

using namespace Eigen;

namespace vimp{

template <typename ExperimentParams>
class ExperimentRunner{

public:
    virtual ~ExperimentRunner(){}
    ExperimentRunner(int nx, 
                int nu, 
                int num_exp, 
                const std::string & config):
                _nx(nx),
                _nu(nu),
                _config_file(config),
                _num_exp(num_exp){}

    virtual void read_config_file(ExperimentParams& params) = 0;
    virtual void read_boundary_conditions(const rapidxml::xml_node<>* paramNode, ExperimentParams& param) = 0;
    virtual void run_one_exp(int exp, ExperimentParams& params) = 0;

    void read_config_file(){
        read_config_file(_params);
    }

    void run(){
        std::cout << "run " << std::endl;
        rapidxml::file<> xmlFile(_config_file.data()); // Default template is char
        rapidxml::xml_document<> doc;
        doc.parse<0>(xmlFile.data());
        for (int i=1; i<_num_exp+1; i++){
            this->run_one_exp(i, _params);
        }
    }


public:
    MatrixIO m_io;
    EigenWrapper _ei;
    std::string _config_file;
    int _num_exp, _nx, _nu, _nt;
    ExperimentParams _params;

};

template <typename GVIMPOptimizer>
class GVIMPRunner : public ExperimentRunner<GVIMPExperimentParams>{
public:
    virtual ~GVIMPRunner(){}
    GVIMPRunner(int nx, 
                int nu, 
                int num_exp, 
                const std::string & config):
                ExperimentRunner<GVIMPExperimentParams>(nx, nu, num_exp, config)
                {
                    read_config_file(_params);
                }
    
    void read_config_file(GVIMPExperimentParams& params) override {
        rapidxml::file<> xmlFile(_config_file.data()); // Default template is char
        rapidxml::xml_document<> doc;
        doc.parse<0>(xmlFile.data());
        
        // Common parameters
        std::string CommonNodeName = "Commons";
        char * c_commons = CommonNodeName.data();
        rapidxml::xml_node<>* commonParams = doc.first_node(c_commons);

        double total_time = atof(commonParams->first_node("total_time")->value());
        double coeff_Qc = atof(commonParams->first_node("coeff_Qc")->value());
        double sig_obs = atof(commonParams->first_node("sig_obs")->value());
        double eps_sdf = atof(commonParams->first_node("eps_sdf")->value());
        double radius = atof(commonParams->first_node("radius")->value());
        double step_size = atof(commonParams->first_node("step_size")->value());
        double stop_err = atof(commonParams->first_node("stop_err")->value());
        double init_precision_factor = atof(commonParams->first_node("init_precision_factor")->value());
        double boundary_penalties = atof(commonParams->first_node("boundary_penalties")->value());
        double temperature = atof(commonParams->first_node("temperature")->value());
        double high_temperature = atof(commonParams->first_node("high_temperature")->value());
        
        int nt = atoi(commonParams->first_node("n_states")->value());
        int low_temp_iterations = atoi(commonParams->first_node("low_temp_iterations")->value());
        int max_iterations = atoi(commonParams->first_node("max_iterations")->value());
        int max_n_backtracking = atoi(commonParams->first_node("max_n_backtracking")->value());

        params = GVIMPExperimentParams(_nx, _nu, total_time, nt, coeff_Qc, sig_obs, eps_sdf, radius, 
                                       step_size, max_iterations, init_precision_factor, boundary_penalties, 
                                       temperature, high_temperature, low_temp_iterations, stop_err, max_n_backtracking);
    }

    void read_boundary_conditions(const rapidxml::xml_node<>* paramNode, GVIMPExperimentParams& param) override {
        double start_x = atof(paramNode->first_node("start_pos")->first_node("x")->value());
        double start_y = atof(paramNode->first_node("start_pos")->first_node("y")->value());

        double start_vx = atof(paramNode->first_node("start_pos")->first_node("vx")->value());
        double start_vy = atof(paramNode->first_node("start_pos")->first_node("vy")->value());

        double goal_x = atof(paramNode->first_node("goal_pos")->first_node("x")->value());
        double goal_y = atof(paramNode->first_node("goal_pos")->first_node("y")->value());

        double goal_vx = atof(paramNode->first_node("goal_pos")->first_node("vx")->value());
        double goal_vy = atof(paramNode->first_node("goal_pos")->first_node("vy")->value());

        VectorXd m0(_nx), mT(_nx); 
        m0 << start_x, start_y, start_vx, start_vy;
        mT << goal_x, goal_y, goal_vx, goal_vy;

        param.set_m0(m0);
        param.set_mT(mT);

        double cost_sigma = atof(paramNode->first_node("cost_sigma")->value());
        param.update_sig_obs(cost_sigma);

        double eta = atof(paramNode->first_node("eta")->value());
        param.update_step_size(eta);

        std::string saving_prefix = static_cast<std::string>(paramNode->first_node("saving_prefix")->value());
        param.set_saving_prefix(saving_prefix);

    }

    void run_one_exp(int exp, GVIMPExperimentParams& params) override {
        rapidxml::file<> xmlFile(_config_file.data()); // Default template is char
        rapidxml::xml_document<> doc;
        doc.parse<0>(xmlFile.data());        
        std::string ExpNodeName = "Experiment" + std::to_string(exp);

        char * c_expname = ExpNodeName.data();
        rapidxml::xml_node<>* paramNode = doc.first_node(c_expname);
        
        read_boundary_conditions(paramNode, params);
                
        MatrixIO matrix_io;
        // An example pr and sdf
        GVIMPOptimizer gvimp_robotsdf(params);
        gvimp_robotsdf.run_optimization(params);

    }

};

template <typename PGCSOptimizer>
class PGCSRunner: public ExperimentRunner<PGCSExperimentParams>{
public:
    // PGCSRunner(){}
    virtual ~PGCSRunner(){}

    PGCSRunner(int nx, int nu, int num_exp, const std::string & config): 
                    ExperimentRunner<PGCSExperimentParams>(nx, nu, num_exp, config)
                    {
                        read_config_file(_params);
                    }

    void read_boundary_conditions(const rapidxml::xml_node<>* paramNode){
        this->read_boundary_conditions(paramNode, _params);
    }

    void run_one_exp(int exp, PGCSExperimentParams& param) override{
        rapidxml::file<> xmlFile(_config_file.data()); // Default template is char
        rapidxml::xml_document<> doc;
        doc.parse<0>(xmlFile.data());
        
        std::string ExpNodeName = "Experiment" + std::to_string(exp);

        char * c_expname = ExpNodeName.data();
        rapidxml::xml_node<>* paramNode = doc.first_node(c_expname);
        
        this->read_boundary_conditions(paramNode, param);

        MatrixXd A0(_nx, _nx), B0(_nx, _nu), a0(_nx, 1);
        A0.setZero(); B0.setZero(); a0.setZero();
        std::shared_ptr<ConstantVelDynamics> pdyn{new ConstantVelDynamics(param.nx(), param.nu(), param.nt())};
        A0 = pdyn->A0();
        B0 = pdyn->B0();
        a0 = pdyn->a0();
        
        PGCSOptimizer pgcs_lin_sdf(A0, a0, B0, pdyn, param);

        using NominalHistory = std::tuple<std::vector<Matrix3D>, std::vector<Matrix3D>>;
        std::tuple<MatrixXd, MatrixXd, NominalHistory> res_Kd;
        // res_Kd = pgcs_lin_sdf.optimize();
        res_Kd = pgcs_lin_sdf.backtrack();

        MatrixXd Kt(_nx*_nx, param.nt()), dt(_nx, param.nt());
        Kt = std::get<0>(res_Kd);
        dt = std::get<1>(res_Kd);

        NominalHistory ztSigt;
        ztSigt = std::get<2>(res_Kd);
        Matrix3D h_zt, h_Sigt;
        h_zt = _ei.vec2mat3d(std::get<0>(ztSigt));
        h_Sigt = _ei.vec2mat3d(std::get<1>(ztSigt));

        MatrixXd zk_star(_nx, param.nt()), Sk_star(_nx*_nx, param.nt());
        zk_star = pgcs_lin_sdf.zkt();
        Sk_star = pgcs_lin_sdf.Sigkt();

        std::string saving_prefix = static_cast<std::string>(paramNode->first_node("saving_prefix")->value());

        m_io.saveData(saving_prefix + std::string{"zk_sdf.csv"}, zk_star);
        m_io.saveData(saving_prefix + std::string{"Sk_sdf.csv"}, Sk_star);

        m_io.saveData(saving_prefix + std::string{"zk_history.csv"}, h_zt);
        m_io.saveData(saving_prefix + std::string{"Sk_history.csv"}, h_Sigt);

        m_io.saveData(saving_prefix + std::string{"Kt_sdf.csv"}, Kt);
        m_io.saveData(saving_prefix + std::string{"dt_sdf.csv"}, dt);

        pgcs_lin_sdf.save_costs(saving_prefix + std::string{"costs.csv"});

    }

    void read_config_file(PGCSExperimentParams& param) override {
        rapidxml::file<> xmlFile(_config_file.data()); // Default template is char
        rapidxml::xml_document<> doc;
        doc.parse<0>(xmlFile.data());

        // Common parameters
        std::string CommonNodeName = "Commons";
        char * c_commons = CommonNodeName.data();
        rapidxml::xml_node<>* commonParams = doc.first_node(c_commons);

        double eps = atoi(commonParams->first_node("eps")->value());
        double eps_sdf = atof(commonParams->first_node("eps_sdf")->value());
        double radius = atof(commonParams->first_node("radius")->value());
        double total_time = atof(commonParams->first_node("total_time")->value());
        _nt = atoi(commonParams->first_node("nt")->value());

        double sig0 = atof(commonParams->first_node("sig0")->value());
        double sigT = atof(commonParams->first_node("sigT")->value());

        double eta = atof(commonParams->first_node("eta")->value());
        double stop_err = atof(commonParams->first_node("stop_err")->value());
        int max_iterations = atoi(commonParams->first_node("max_iter")->value());
        double sig_obs = atof(commonParams->first_node("cost_sigma")->value());
        double backtracking_ratio = atof(commonParams->first_node("backtracking_ratio")->value());
        int max_n_backtracking = atoi(commonParams->first_node("max_n_backtracking")->value());
        // std::string sdf_file = static_cast<std::string>(commonParams->first_node("sdf_file")->value());

        param = PGCSExperimentParams(_nx, _nu, eps_sdf, radius, eps, total_time, _nt, sig0, sigT, eta, stop_err, sig_obs, max_iterations, backtracking_ratio, max_n_backtracking);

    }

    virtual void read_boundary_conditions(const rapidxml::xml_node<>* paramNode, PGCSExperimentParams& param){
        double start_x = atof(paramNode->first_node("start_pos")->first_node("x")->value());
        double start_y = atof(paramNode->first_node("start_pos")->first_node("y")->value());

        double start_vx = atof(paramNode->first_node("start_pos")->first_node("vx")->value());
        double start_vy = atof(paramNode->first_node("start_pos")->first_node("vy")->value());

        double goal_x = atof(paramNode->first_node("goal_pos")->first_node("x")->value());
        double goal_y = atof(paramNode->first_node("goal_pos")->first_node("y")->value());

        double goal_vx = atof(paramNode->first_node("goal_pos")->first_node("vx")->value());
        double goal_vy = atof(paramNode->first_node("goal_pos")->first_node("vy")->value());

        VectorXd m0(_nx), mT(_nx); 
        m0 << start_x, start_y, start_vx, start_vy;
        mT << goal_x, goal_y, goal_vx, goal_vy;

        param.set_m0(m0);
        param.set_mT(mT);

        double cost_sigma = atof(paramNode->first_node("cost_sigma")->value());
        param.update_sig_obs(cost_sigma);
       
    }

};

template <typename PGCSOptimizer>
class PGCSRunner3D: public PGCSRunner<PGCSOptimizer>{
public:
    
    PGCSRunner3D(int num_exp, const std::string & config):
                        PGCSRunner<PGCSOptimizer>(6, 3, num_exp, config){}

    void read_boundary_conditions(const rapidxml::xml_node<>* paramNode, PGCSExperimentParams& params) override{
        double start_x = atof(paramNode->first_node("start_pos")->first_node("x")->value());
        double start_y = atof(paramNode->first_node("start_pos")->first_node("y")->value());
        double start_z = atof(paramNode->first_node("start_pos")->first_node("z")->value());

        double start_vx = atof(paramNode->first_node("start_pos")->first_node("vx")->value());
        double start_vy = atof(paramNode->first_node("start_pos")->first_node("vy")->value());
        double start_vz = atof(paramNode->first_node("start_pos")->first_node("vz")->value());

        double goal_x = atof(paramNode->first_node("goal_pos")->first_node("x")->value());
        double goal_y = atof(paramNode->first_node("goal_pos")->first_node("y")->value());
        double goal_z = atof(paramNode->first_node("goal_pos")->first_node("z")->value());

        double goal_vx = atof(paramNode->first_node("goal_pos")->first_node("vx")->value());
        double goal_vy = atof(paramNode->first_node("goal_pos")->first_node("vy")->value());
        double goal_vz = atof(paramNode->first_node("goal_pos")->first_node("vz")->value());
        
        VectorXd m0(this->_nx), mT(this->_nx); 

        m0 << start_x, start_y, start_z, start_vx, start_vy, start_vz;
        mT << goal_x, goal_y, goal_z, goal_vx, goal_vy, goal_vz;

        params.set_m0(m0);
        params.set_mT(mT);

        std::string sdf_file = static_cast<std::string>(paramNode->first_node("sdf_file")->value());
        params.update_sdf_file(sdf_file);

        double sig_obs = atof(paramNode->first_node("cost_sigma")->value());
        params.update_sig_obs(sig_obs);
    }
};

template <typename PGCSOptimizer>
class PGCSRunner7D: public PGCSRunner<PGCSOptimizer>{
public:
    
    PGCSRunner7D(int num_exp, const std::string & config):
                        PGCSRunner<PGCSOptimizer>(14, 7, num_exp, config){}

    void read_boundary_conditions(const rapidxml::xml_node<>* paramNode, PGCSExperimentParams& param) override{
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

        VectorXd m0(this->_nx), mT(this->_nx); 

        Eigen::VectorXd mo_pos(7);
        mo_pos << start_1, start_2, start_3, start_4, start_5, start_6, start_7;
        m0.block(0, 0, 7, 1) = mo_pos;
        m0.block(7, 0, 7, 1) = Eigen::VectorXd::Zero(7);

        Eigen::VectorXd mT_pos(7);
        mT_pos << goal_1, goal_2, goal_3, goal_4, goal_5, goal_6, goal_7;
        mT.block(0, 0, 7, 1) = mT_pos;
        mT.block(7, 0, 7, 1) = Eigen::VectorXd::Zero(7);

        double sig_obs = atof(paramNode->first_node("cost_sigma")->value());

        param.set_m0(m0);
        param.set_mT(mT);
        param.update_sig_obs(sig_obs);
    }

};



}