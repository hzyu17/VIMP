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
#include "dynamics/NonlinearDynamics.h"
#include "3rdparty/rapidxml-1.13/rapidxml.hpp"
#include "3rdparty/rapidxml-1.13/rapidxml_utils.hpp"
#include <memory>

using namespace Eigen;

namespace vimp{

using NominalHistory = std::tuple<std::vector<Matrix3D>, std::vector<Matrix3D>>;
using PGCSResultHistory = std::tuple<MatrixXd, MatrixXd, NominalHistory>;
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

    virtual void read_config(ExperimentParams& params) = 0;
    virtual void read_boundary_conditions(const rapidxml::xml_node<>* paramNode, ExperimentParams& params) = 0;
    virtual void run_one_exp(int exp, ExperimentParams& params, bool verbose=true) = 0;

    void read_config(){
        read_config(_params);
    }

    virtual void run(){
        rapidxml::file<> xmlFile(_config_file.data()); // Default template is char
        rapidxml::xml_document<> doc;
        doc.parse<0>(xmlFile.data());
        
        for (int i=1; i<_num_exp+1; i++){
            this->run_one_exp(i, _params);
        }

        std::cout << "========== End of all experiments ==========" << std::endl;
    }


public:
    MatrixIO m_io;
    EigenWrapper _ei;
    std::string _config_file;
    int _num_exp, _nx, _nu, _nt;
    ExperimentParams _params;

};

template <typename GVIMPOptimizer>
class GVIMPRunner : public ExperimentRunner<GVIMPParams>{
public:
    virtual ~GVIMPRunner(){}
    GVIMPRunner(int nx, 
                int nu, 
                int num_exp, 
                const std::string & config):
                ExperimentRunner<GVIMPParams>(nx, nu, num_exp, config)
                {
                    read_config(_params);
                    _gvimp_robotsdf = GVIMPOptimizer(_params);
                }
    
    void read_config(GVIMPParams& params) override {
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

        std::string map_name = static_cast<std::string>(commonParams->first_node("map_name")->value());

        params = GVIMPParams(this->_nx, this->_nu, total_time, nt, coeff_Qc, sig_obs, eps_sdf, radius, 
                            step_size, max_iterations, init_precision_factor, boundary_penalties, 
                            temperature, high_temperature, low_temp_iterations, stop_err, max_n_backtracking, map_name);
    }

    void read_boundary_conditions(const rapidxml::xml_node<>* paramNode, GVIMPParams& params) override {
        double start_x = atof(paramNode->first_node("start_pos")->first_node("x")->value());
        double start_y = atof(paramNode->first_node("start_pos")->first_node("y")->value());

        double start_vx = atof(paramNode->first_node("start_pos")->first_node("vx")->value());
        double start_vy = atof(paramNode->first_node("start_pos")->first_node("vy")->value());

        double goal_x = atof(paramNode->first_node("goal_pos")->first_node("x")->value());
        double goal_y = atof(paramNode->first_node("goal_pos")->first_node("y")->value());

        double goal_vx = atof(paramNode->first_node("goal_pos")->first_node("vx")->value());
        double goal_vy = atof(paramNode->first_node("goal_pos")->first_node("vy")->value());

        VectorXd m0(this->_nx), mT(this->_nx); 
        m0 << start_x, start_y, start_vx, start_vy;
        mT << goal_x, goal_y, goal_vx, goal_vy;

        params.set_m0(m0);
        params.set_mT(mT);

        if (paramNode->first_node("step_size")){
            double step_size = atof(paramNode->first_node("step_size")->value());
            params.update_step_size(step_size);
        }

        if (paramNode->first_node("max_iterations")){
            int max_iterations = atoi(paramNode->first_node("max_iterations")->value());
            params.update_max_iter(max_iterations);
        }

        if (paramNode->first_node("low_temp_iterations")){
            int low_temp_iterations = atoi(paramNode->first_node("low_temp_iterations")->value());
            params.update_lowtemp_iter(low_temp_iterations);
        }

        double cost_sigma = atof(paramNode->first_node("sig_obs")->value());
        params.update_sig_obs(cost_sigma);

        std::string saving_prefix = static_cast<std::string>(paramNode->first_node("saving_prefix")->value());
        params.set_saving_prefix(saving_prefix);

    }

    void run_one_exp(int exp, GVIMPParams& params, bool verbose=true) override {
        rapidxml::file<> xmlFile(_config_file.data()); // Default template is char
        rapidxml::xml_document<> doc;
        doc.parse<0>(xmlFile.data());        
        std::string ExpNodeName = "Experiment" + std::to_string(exp);

        char * c_expname = ExpNodeName.data();
        rapidxml::xml_node<>* paramNode = doc.first_node(c_expname);
        
        this->read_boundary_conditions(paramNode, params);

        params.print_params();
                
        MatrixIO matrix_io;
        // An example pr and sdf
        
        _gvimp_robotsdf.run_optimization(params, verbose);

    }

    std::tuple<VectorXd, SpMat> get_mu_precision() {
        return  _gvimp_robotsdf.get_mu_precision();
    }

    GVIMPOptimizer optimizer_robot_sdf() const { return _gvimp_robotsdf; }

private:
    GVIMPOptimizer _gvimp_robotsdf;

};

template <typename PGCSOptimizer>
class PGCSRunnerBase: public ExperimentRunner<PGCSParams>{
    
public:
    // PGCSRunner(){}
    virtual ~PGCSRunnerBase(){}
    
    PGCSRunnerBase(int nx, int nu, int num_exp, const std::string & config): 
                    ExperimentRunner<PGCSParams>(nx, nu, num_exp, config)
                    {
                        read_config(_params);
                    }

    void read_boundary_conditions(const rapidxml::xml_node<>* paramNode){
        this->read_boundary_conditions(paramNode, _params);
    }

    void run_and_save(PGCSOptimizer & opt_sdf,
                    const PGCSParams & params, 
                    const rapidxml::xml_node<>* paramNode){
        
        PGCSResultHistory res_Kd;

        res_Kd = opt_sdf.backtrack();

        std::cout << "debug 2" << std::endl;

        MatrixXd Kt(this->_nx*this->_nx, params.nt()), dt(this->_nx, params.nt());
        Kt = std::get<0>(res_Kd);
        dt = std::get<1>(res_Kd);

        NominalHistory ztSigt = std::get<2>(res_Kd);
        // ztSigt 
        Matrix3D h_zt = _ei.vec2mat3d(std::get<0>(ztSigt));
        Matrix3D h_Sigt = _ei.vec2mat3d(std::get<1>(ztSigt));

        MatrixXd zk_star(this->_nx, params.nt()), Sk_star(this->_nx*this->_nx, params.nt());
        zk_star = opt_sdf.zkt();
        Sk_star = opt_sdf.Sigkt();

        std::cout << "debug 3" << std::endl;

        std::string saving_prefix = static_cast<std::string>(paramNode->first_node("saving_prefix")->value());

        m_io.saveData(saving_prefix + std::string{"zk_sdf.csv"}, zk_star);
        m_io.saveData(saving_prefix + std::string{"Sk_sdf.csv"}, Sk_star);

        m_io.saveData(saving_prefix + std::string{"zk_history.csv"}, h_zt);
        m_io.saveData(saving_prefix + std::string{"Sk_history.csv"}, h_Sigt);

        m_io.saveData(saving_prefix + std::string{"Kt_sdf.csv"}, Kt);
        m_io.saveData(saving_prefix + std::string{"dt_sdf.csv"}, dt);

        Matrix3D hAkt_star = opt_sdf.hAkt();
        Matrix3D hakt_star = opt_sdf.hakt();

        m_io.saveData(saving_prefix + std::string{"hAkt_sdf.csv"}, hAkt_star);
        m_io.saveData(saving_prefix + std::string{"hakt_sdf.csv"}, hakt_star);

        opt_sdf.save_costs(saving_prefix + std::string{"costs.csv"});
    }

    virtual void run_one_exp(int exp, PGCSParams& params, bool verbose=true) = 0;

    void read_config(PGCSParams& params) override {
        rapidxml::file<> xmlFile(_config_file.data()); // Default template is char
        rapidxml::xml_document<> doc;
        doc.parse<0>(xmlFile.data());

        // Common parameters
        std::string CommonNodeName = "Commons";
        char * c_commons = CommonNodeName.data();
        rapidxml::xml_node<>* commonParams = doc.first_node(c_commons);

        double eps = atof(commonParams->first_node("eps")->value());
        double eps_sdf = atof(commonParams->first_node("eps_sdf")->value());
        double radius = atof(commonParams->first_node("radius")->value());

        double total_time = atof(commonParams->first_node("total_time")->value());
        this->_nt = atoi(commonParams->first_node("nt")->value());

        double sig0 = atof(commonParams->first_node("sig0")->value());
        double sigT = atof(commonParams->first_node("sigT")->value());

        double eta = atof(commonParams->first_node("eta")->value());
        double stop_err = atof(commonParams->first_node("stop_err")->value());
        double sig_obs = atof(commonParams->first_node("sig_obs")->value());
        double backtracking_ratio = atof(commonParams->first_node("backtracking_ratio")->value());
        
        int max_iterations = atoi(commonParams->first_node("max_iter")->value());
        int max_n_backtracking = atoi(commonParams->first_node("max_n_backtracking")->value());
        // std::string sdf_file = static_cast<std::string>(commonParams->first_node("sdf_file")->value());

        params = PGCSParams(this->_nx, this->_nu, eps_sdf, radius, eps, total_time, this->_nt, 
                            sig0, sigT, eta, stop_err, sig_obs, 
                            max_iterations, backtracking_ratio, max_n_backtracking);

        // if (commonParams->first_node("field_file")){
        //     std::string field = static_cast<std::string>(commonParams->first_node("field_file")->value());
        //     params.update_field_file(field);
        // }

        if (commonParams->first_node("sdf_file")){
            std::string sdf_file = static_cast<std::string>(commonParams->first_node("sdf_file")->value());
            params.update_sdf_file(sdf_file);
        }

    }

    virtual void read_boundary_conditions(const rapidxml::xml_node<>* paramNode, PGCSParams& params) = 0;

};

template <typename PGCSOptimizer>
class PGCSRunnerLinDynBase: public PGCSRunnerBase<PGCSOptimizer>{
public:
    // PGCSRunner(){}
    virtual ~PGCSRunnerLinDynBase(){}

    PGCSRunnerLinDynBase(int nx, int nu, int num_exp, const std::string & config):
                        PGCSRunnerBase<PGCSOptimizer>(nx, nu, num_exp, config){}

void run_one_exp(int exp, PGCSParams& params, bool verbose=true)
{
    rapidxml::file<> xmlFile(this->_config_file.data()); // Default template is char

    std::cout << "debug 1" << std::endl;

    rapidxml::xml_document<> doc;
    doc.parse<0>(xmlFile.data());
    
    std::string ExpNodeName = "Experiment" + std::to_string(exp);

    char * c_expname = ExpNodeName.data();
    rapidxml::xml_node<>* paramNode = doc.first_node(c_expname);
    
    this->read_boundary_conditions(paramNode, params);

    MatrixXd A0(this->_nx, this->_nx), B0(this->_nx, this->_nu), a0(this->_nx, 1);
    A0.setZero(); B0.setZero(); a0.setZero();
    std::shared_ptr<ConstantVelDynamics> pdyn{new ConstantVelDynamics(params.nx(), params.nu(), params.nt())};
    A0 = pdyn->A0();
    B0 = pdyn->B0();
    a0 = pdyn->a0();
    
    PGCSOptimizer opt_sdf(A0, a0, B0, pdyn, params);

    this->run_and_save(opt_sdf, params, paramNode);
}
};

template <typename PGCSOptimizer>
class PGCSRunner: public PGCSRunnerLinDynBase<PGCSOptimizer>{
public:
    // PGCSRunner(){}
    virtual ~PGCSRunner(){}

    PGCSRunner(int nx, int nu, int num_exp, const std::string & config): 
                    PGCSRunnerLinDynBase<PGCSOptimizer>(nx, nu, num_exp, config)
                    {}

    void read_boundary_conditions(const rapidxml::xml_node<>* paramNode, PGCSParams& params) override{
        double start_x = atof(paramNode->first_node("start_pos")->first_node("x")->value());
        double start_y = atof(paramNode->first_node("start_pos")->first_node("y")->value());

        double start_vx = atof(paramNode->first_node("start_pos")->first_node("vx")->value());
        double start_vy = atof(paramNode->first_node("start_pos")->first_node("vy")->value());

        double goal_x = atof(paramNode->first_node("goal_pos")->first_node("x")->value());
        double goal_y = atof(paramNode->first_node("goal_pos")->first_node("y")->value());

        double goal_vx = atof(paramNode->first_node("goal_pos")->first_node("vx")->value());
        double goal_vy = atof(paramNode->first_node("goal_pos")->first_node("vy")->value());

        VectorXd m0(this->_nx), mT(this->_nx); 
        m0 << start_x, start_y, start_vx, start_vy;
        mT << goal_x, goal_y, goal_vx, goal_vy;

        params.set_m0(m0);
        params.set_mT(mT);

        if (paramNode->first_node("eta")){
            double eta = atof(paramNode->first_node("eta")->value());
            params.update_step_size(eta);
        }

        double cost_sigma = atof(paramNode->first_node("sig_obs")->value());
        params.update_sig_obs(cost_sigma);

        params.print_params();
       
    }

};


template <typename PGCSOptimizer>
class PGCSRunner3D: public PGCSRunnerLinDynBase<PGCSOptimizer>{
public:
    // PGCSRunner(){}
    virtual ~PGCSRunner3D(){}

    PGCSRunner3D(int num_exp, const std::string & config):
                        PGCSRunnerLinDynBase<PGCSOptimizer>(6, 3, num_exp, config){}


    void read_boundary_conditions(const rapidxml::xml_node<>* paramNode, PGCSParams& params) override{
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

        if (paramNode->first_node("eta")){
            double eta = atof(paramNode->first_node("eta")->value());
            params.update_step_size(eta);
        }

        double sig_obs = atof(paramNode->first_node("sig_obs")->value());
        params.update_sig_obs(sig_obs);
    }
};

template <typename PGCSOptimizer>
class PGCSRunner7D: public PGCSRunnerLinDynBase<PGCSOptimizer>{
public:
    // PGCSRunner(){}
    virtual ~PGCSRunner7D(){}

    PGCSRunner7D(int num_exp, const std::string & config):
                        PGCSRunnerLinDynBase<PGCSOptimizer>(14, 7, num_exp, config){}
    

    void read_boundary_conditions(const rapidxml::xml_node<>* paramNode, PGCSParams& params) override{
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

        double sig_obs = atof(paramNode->first_node("sig_obs")->value());

        params.set_m0(m0);
        params.set_mT(mT);
        params.update_sig_obs(sig_obs);
    }

};


template <typename PGCSOptimizer, typename Dynamics>
class PGCSRunnerNonLinear: public PGCSRunnerBase<PGCSOptimizer>{
public:
    // PGCSRunner(){}
    virtual ~PGCSRunnerNonLinear(){}

    PGCSRunnerNonLinear(int nx, int nu, int num_exp, const std::string & config): 
            PGCSRunnerBase<PGCSOptimizer>(nx, nu, num_exp, config)
    {}

    void read_boundary_conditions(const rapidxml::xml_node<>* paramNode, PGCSParams& params) override{
        double start_1 = atof(paramNode->first_node("start_pos")->first_node("1")->value());
        double start_2 = atof(paramNode->first_node("start_pos")->first_node("2")->value());
        double start_3 = atof(paramNode->first_node("start_pos")->first_node("3")->value());
        double start_4 = atof(paramNode->first_node("start_pos")->first_node("4")->value());
        double start_5 = atof(paramNode->first_node("start_pos")->first_node("5")->value());
        double start_6 = atof(paramNode->first_node("start_pos")->first_node("6")->value());

        double goal_1 = atof(paramNode->first_node("goal_pos")->first_node("1")->value());
        double goal_2 = atof(paramNode->first_node("goal_pos")->first_node("2")->value());
        double goal_3 = atof(paramNode->first_node("goal_pos")->first_node("3")->value());
        double goal_4 = atof(paramNode->first_node("goal_pos")->first_node("4")->value());
        double goal_5 = atof(paramNode->first_node("goal_pos")->first_node("5")->value());
        double goal_6 = atof(paramNode->first_node("goal_pos")->first_node("6")->value());

        Eigen::VectorXd mo_pos(this->_nx);
        mo_pos << start_1, start_2, start_3, start_4, start_5, start_6;

        Eigen::VectorXd mT_pos(this->_nx);
        mT_pos << goal_1, goal_2, goal_3, goal_4, goal_5, goal_6;

        double sig_obs = atof(paramNode->first_node("sig_obs")->value());

        params.set_m0(mo_pos);
        params.set_mT(mT_pos);
        params.update_sig_obs(sig_obs);

    }

    void run_one_exp(int exp, PGCSParams& params, bool verbose=true) override{

        rapidxml::file<> xmlFile(this->_config_file.data()); // Default template is char
        rapidxml::xml_document<> doc;
        doc.parse<0>(xmlFile.data());
        
        std::string ExpNodeName = "Experiment" + std::to_string(exp);

        char * c_expname = ExpNodeName.data();
        rapidxml::xml_node<>* paramNode = doc.first_node(c_expname);
        
        this->read_boundary_conditions(paramNode, params);

        std::shared_ptr<Dynamics> pdyn{new Dynamics(params.nx(), params.nu(), params.nt())};
        VectorXd m0 = params.m0();
        MatrixXd Sig0 = params.Sig0();
        MatrixXd A0(this->_nx, this->_nx), B0(this->_nx, this->_nu), a0(this->_nx, 1);
        A0.setZero(); B0.setZero(); a0.setZero();
        std::tuple<MatrixXd, MatrixXd, VectorXd, VectorXd> hABhanTr;
        hABhanTr = pdyn->linearize_at(m0, A0, Sig0);

        A0 = std::get<0>(hABhanTr);
        B0 = std::get<1>(hABhanTr);
        a0 = std::get<2>(hABhanTr);

        PGCSOptimizer opt_sdf(A0, a0, B0, pdyn, params);

        this->run_and_save(opt_sdf, params, paramNode);
        
    }

};



}