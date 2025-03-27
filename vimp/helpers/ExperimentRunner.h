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

#define STRING(x) #x
#define XSTRING(x) STRING(x)

#include "helpers/ExperimentParams.h"
#include "dynamics/LinearDynamics.h"
#include "3rdparty/rapidxml-1.13/rapidxml.hpp"
#include "3rdparty/rapidxml-1.13/rapidxml_utils.hpp"
#include <memory>
#include "helpers/timer.h"

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
    virtual double run_one_exp(int exp, ExperimentParams& params, bool verbose=true) = 0;

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
    gvi::MatrixIO m_io;
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
        double alpha = atof(commonParams->first_node("alpha")->value());
        double init_precision_factor = atof(commonParams->first_node("init_precision_factor")->value());
        double boundary_penalties = atof(commonParams->first_node("boundary_penalties")->value());
        double temperature = atof(commonParams->first_node("temperature")->value());
        double high_temperature = atof(commonParams->first_node("high_temperature")->value());
        
        int gh_degree = atoi(commonParams->first_node("GH_deg")->value());
        int nt = atoi(commonParams->first_node("n_states")->value());
        int low_temp_iterations = atoi(commonParams->first_node("low_temp_iterations")->value());
        int max_iterations = atoi(commonParams->first_node("max_iterations")->value());
        int max_n_backtracking = atoi(commonParams->first_node("max_n_backtracking")->value());

        std::string map_name = static_cast<std::string>(commonParams->first_node("map_name")->value());
        std::string sdf_file = "";
        if (commonParams->first_node("sdf_file")){
            sdf_file = static_cast<std::string>(commonParams->first_node("sdf_file")->value());
        }

        params = GVIMPParams(this->_nx, this->_nu, total_time, nt, coeff_Qc, gh_degree, sig_obs, eps_sdf, radius, 
                            step_size, max_iterations, init_precision_factor, boundary_penalties, 
                            temperature, high_temperature, low_temp_iterations, stop_err, max_n_backtracking, map_name, sdf_file, alpha);
    }

    void read_boundary_conditions(const rapidxml::xml_node<>* paramNode, GVIMPParams& params) override {

        VectorXd m0(this->_nx), mT(this->_nx); 
        if (this->_nx == 4){
            double start_x = atof(paramNode->first_node("start_pos")->first_node("x")->value());
            double start_y = atof(paramNode->first_node("start_pos")->first_node("y")->value());

            double start_vx = atof(paramNode->first_node("start_pos")->first_node("vx")->value());
            double start_vy = atof(paramNode->first_node("start_pos")->first_node("vy")->value());

            double goal_x = atof(paramNode->first_node("goal_pos")->first_node("x")->value());
            double goal_y = atof(paramNode->first_node("goal_pos")->first_node("y")->value());

            double goal_vx = atof(paramNode->first_node("goal_pos")->first_node("vx")->value());
            double goal_vy = atof(paramNode->first_node("goal_pos")->first_node("vy")->value());

            m0 << start_x, start_y, start_vx, start_vy;
            mT << goal_x, goal_y, goal_vx, goal_vy;
        }
        else if (this->_nx == 6){
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

            m0 << start_x, start_y, start_z, start_vx, start_vy, start_vz;
            mT << goal_x, goal_y, goal_z, goal_vx, goal_vy, goal_vz;
        }

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

        if (paramNode->first_node("init_precision_factor")){
            double init_precision_factor = atof(paramNode->first_node("init_precision_factor")->value());
            params.update_initial_precision_factor(init_precision_factor);
        }

        if (paramNode->first_node("sig_obs")){
            double cost_sigma = atof(paramNode->first_node("sig_obs")->value());
            params.update_sig_obs(cost_sigma);
        }

        if (paramNode->first_node("total_time")){
            double total_time = atof(paramNode->first_node("total_time")->value());
            params.set_total_time(total_time);
        }

        if (paramNode->first_node("temperature")){
            double temperature = atof(paramNode->first_node("temperature")->value());
            params.set_temperature(temperature);
        }

        if (paramNode->first_node("high_temperature")){
            double high_temperature = atof(paramNode->first_node("high_temperature")->value());
            params.set_high_temperature(high_temperature);
        }

        if (paramNode->first_node("step_size")){
            double step_size = atof(paramNode->first_node("step_size")->value());
            params.update_step_size(step_size);
        }

        if (paramNode->first_node("boundary_penalties")){
            double boundary_penalties = atof(paramNode->first_node("boundary_penalties")->value());
            params.update_boundary_penalties(boundary_penalties);
        }

        if (paramNode->first_node("alpha")){
            double alpha = atof(paramNode->first_node("alpha")->value());
            params.update_alpha(alpha);
        }

        std::string source_root{XSTRING(SOURCE_ROOT)};
        std::string saving_prefix_relative = static_cast<std::string>(paramNode->first_node("saving_prefix")->value());
        std::string saving_prefix = source_root + "/../" + saving_prefix_relative;
        params.set_saving_prefix(saving_prefix);
    }

    double run_one_exp(int exp, GVIMPParams& params, bool verbose=true) override {
        rapidxml::file<> xmlFile(_config_file.data()); // Default template is char
        rapidxml::xml_document<> doc;
        doc.parse<0>(xmlFile.data());
        std::string ExpNodeName = "Experiment" + std::to_string(exp);

        char * c_expname = ExpNodeName.data();
        rapidxml::xml_node<>* paramNode = doc.first_node(c_expname);

        std::cout << ExpNodeName.data() << std::endl;
        
        std::cout << "----- Reading Boundary Conditions -----" << std::endl;
        
        this->read_boundary_conditions(paramNode, params);

        if (verbose){
            params.print_params();
        }
        
        return _gvimp_robotsdf.run_optimization_withtime(params, verbose);
        
    }

    // Run one experiment, return the optimized means and precision matrices.
    std::tuple<Eigen::VectorXd, SpMat> run_one_exp_return(GVIMPParams& params, bool verbose=true) 
    {
        params.print_params();
        return _gvimp_robotsdf.run_optimization_return(params, verbose);
    }

    std::tuple<VectorXd, SpMat> get_mu_precision() {
        return  _gvimp_robotsdf.get_mu_precision();
    }

    GVIMPOptimizer optimizer_robot_sdf() const { return _gvimp_robotsdf; }

private:
    GVIMPOptimizer _gvimp_robotsdf;

};


template <typename GVIMPOptimizer>
class GVIMPRunner_Quadrotor : public ExperimentRunner<GVIMPParams_nonlinear>{
public:
    virtual ~GVIMPRunner_Quadrotor(){}
    GVIMPRunner_Quadrotor(int nx, int nu, int num_exp, const std::string & config)
        : ExperimentRunner<GVIMPParams_nonlinear>(nx, nu, num_exp, config) 
        {
            read_config(_params);
            _gvimp_robotsdf = GVIMPOptimizer(_params);
        }

    void read_config(GVIMPParams_nonlinear& params) override {
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
        double alpha = atof(commonParams->first_node("alpha")->value());
        double init_precision_factor = atof(commonParams->first_node("init_precision_factor")->value());
        double boundary_penalties = atof(commonParams->first_node("boundary_penalties")->value());
        double temperature = atof(commonParams->first_node("temperature")->value());
        double high_temperature = atof(commonParams->first_node("high_temperature")->value());
        
        int gh_degree = atoi(commonParams->first_node("GH_deg")->value());
        int nt = atoi(commonParams->first_node("n_states")->value());
        int low_temp_iterations = atoi(commonParams->first_node("low_temp_iterations")->value());
        int max_iterations = atoi(commonParams->first_node("max_iterations")->value());
        int max_n_backtracking = atoi(commonParams->first_node("max_n_backtracking")->value());
        int max_linear_iter = atoi(commonParams->first_node("max_linear_iter")->value());

        std::string map_name = static_cast<std::string>(commonParams->first_node("map_name")->value());

        params = GVIMPParams_nonlinear(this->_nx, this->_nu, total_time, nt, coeff_Qc, gh_degree, sig_obs, eps_sdf, radius, 
                            step_size, max_iterations, init_precision_factor, boundary_penalties, 
                            temperature, high_temperature, low_temp_iterations, stop_err, max_n_backtracking, map_name, "", alpha, max_linear_iter);
    }

    void read_boundary_conditions(const rapidxml::xml_node<>* paramNode, GVIMPParams_nonlinear& params) override {
        VectorXd m0(this->_nx), mT(this->_nx); 

        double start_x = atof(paramNode->first_node("start_pos")->first_node("x")->value());
        double start_z = atof(paramNode->first_node("start_pos")->first_node("z")->value());
        double start_phi = atof(paramNode->first_node("start_pos")->first_node("phi")->value());

        double start_vx = atof(paramNode->first_node("start_pos")->first_node("vx")->value());
        double start_vz = atof(paramNode->first_node("start_pos")->first_node("vz")->value());
        double start_vphi = atof(paramNode->first_node("start_pos")->first_node("vphi")->value());

        double goal_x = atof(paramNode->first_node("goal_pos")->first_node("x")->value());
        double goal_z = atof(paramNode->first_node("goal_pos")->first_node("z")->value());
        double goal_phi = atof(paramNode->first_node("goal_pos")->first_node("phi")->value());

        double goal_vx = atof(paramNode->first_node("goal_pos")->first_node("vx")->value());
        double goal_vz = atof(paramNode->first_node("goal_pos")->first_node("vz")->value());
        double goal_vphi = atof(paramNode->first_node("goal_pos")->first_node("vphi")->value());

        m0 << start_x, start_z, start_phi, start_vx, start_vz, start_vphi;
        mT << goal_x, goal_z, goal_phi, goal_vx, goal_vz, goal_vphi;

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

        if (paramNode->first_node("init_precision_factor")){
            double init_precision_factor = atof(paramNode->first_node("init_precision_factor")->value());
            params.update_initial_precision_factor(init_precision_factor);
        }

        if (paramNode->first_node("sig_obs")){
            double cost_sigma = atof(paramNode->first_node("sig_obs")->value());
            params.update_sig_obs(cost_sigma);
        }

        if (paramNode->first_node("total_time")){
            double total_time = atof(paramNode->first_node("total_time")->value());
            params.set_total_time(total_time);
        }

        if (paramNode->first_node("temperature")){
            double temperature = atof(paramNode->first_node("temperature")->value());
            params.set_temperature(temperature);
        }

        if (paramNode->first_node("high_temperature")){
            double high_temperature = atof(paramNode->first_node("high_temperature")->value());
            params.set_high_temperature(high_temperature);
        }

        if (paramNode->first_node("step_size")){
            double step_size = atof(paramNode->first_node("step_size")->value());
            params.update_step_size(step_size);
        }

        if (paramNode->first_node("boundary_penalties")){
            double boundary_penalties = atof(paramNode->first_node("boundary_penalties")->value());
            params.update_boundary_penalties(boundary_penalties);
        }

        if (paramNode->first_node("alpha")){
            double alpha = atof(paramNode->first_node("alpha")->value());
            params.update_alpha(alpha);
        }

        if (paramNode->first_node("max_linear_iter")){
            int max_linear_iter = atoi(paramNode->first_node("max_linear_iter")->value());
            params.update_max_linear_iter(max_linear_iter);
        }

        if (paramNode->first_node("map_name")){
            std::string map_name = static_cast<std::string>(paramNode->first_node("map_name")->value());
            params.update_map_name(map_name);
        }

        std::string source_root{XSTRING(SOURCE_ROOT)};
        std::string saving_prefix_relative = static_cast<std::string>(paramNode->first_node("saving_prefix")->value());
        std::string saving_prefix = source_root + "/../" + saving_prefix_relative;
        params.set_saving_prefix(saving_prefix);
    }

    double run_one_exp(int exp, GVIMPParams_nonlinear& params, bool verbose=true) override {
        rapidxml::file<> xmlFile(_config_file.data()); // Default template is char
        rapidxml::xml_document<> doc;
        doc.parse<0>(xmlFile.data());
        std::string ExpNodeName = "Experiment" + std::to_string(exp);

        char * c_expname = ExpNodeName.data();
        rapidxml::xml_node<>* paramNode = doc.first_node(c_expname);

        std::cout << ExpNodeName.data() << std::endl;
        
        std::cout << "----- Reading Boundary Conditions -----" << std::endl;
        
        this->read_boundary_conditions(paramNode, params);

        if (verbose){
            params.print_params();
        }
        
        return _gvimp_robotsdf.run_optimization_withtime(params, verbose);
    }

    // Run one experiment, return the optimized means and precision matrices.
    std::tuple<Eigen::VectorXd, SpMat> run_one_exp_return(GVIMPParams_nonlinear& params, bool verbose=true) 
    {
        params.print_params();
        return _gvimp_robotsdf.run_optimization_return(params, verbose);
    }

    std::tuple<VectorXd, SpMat> get_mu_precision() {
        return  _gvimp_robotsdf.get_mu_precision();
    }

    GVIMPOptimizer optimizer_robot_sdf() const { return _gvimp_robotsdf; }

private:
    GVIMPOptimizer _gvimp_robotsdf;

};


template <typename GVIOptimizer>
class GVIMPRunner7D: public GVIMPRunner<GVIOptimizer>{
public:
    // PCSRunner(){}
    virtual ~GVIMPRunner7D(){}

    GVIMPRunner7D(int num_exp, const std::string & config):
                        GVIMPRunner<GVIOptimizer>(14, 7, num_exp, config){}
    

    void read_boundary_conditions(const std::string& config_file, int i_exp, GVIMPParams& params){
        rapidxml::file<> xmlFile(config_file.data()); // Default template is char
        rapidxml::xml_document<> doc;
        doc.parse<0>(xmlFile.data());        
        std::string ExpNodeName = "Experiment" + std::to_string(i_exp);
        char * c_expname = ExpNodeName.data();
        rapidxml::xml_node<>* paramNode = doc.first_node(c_expname);
        
        this->read_boundary_conditions(paramNode, params);

    }

    void read_boundary_conditions(const rapidxml::xml_node<>* paramNode, GVIMPParams& params) override{
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

        Eigen::VectorXd m0_pos(7);
        m0_pos << start_1, start_2, start_3, start_4, start_5, start_6, start_7;
        m0.block(0, 0, 7, 1) = m0_pos;
        m0.block(7, 0, 7, 1) = Eigen::VectorXd::Zero(7);

        Eigen::VectorXd mT_pos(7);
        mT_pos << goal_1, goal_2, goal_3, goal_4, goal_5, goal_6, goal_7;
        mT.block(0, 0, 7, 1) = mT_pos;
        mT.block(7, 0, 7, 1) = Eigen::VectorXd::Zero(7);

        params.set_m0(m0);
        params.set_mT(mT);

        if (paramNode->first_node("temperature")){
            double temperature = atof(paramNode->first_node("temperature")->value());
            params.set_temperature(temperature);
        }

        if (paramNode->first_node("high_temperature")){
            double high_temperature = atof(paramNode->first_node("high_temperature")->value());
            params.set_high_temperature(high_temperature);
        }

        if (paramNode->first_node("step_size")){
            double step_size = atof(paramNode->first_node("step_size")->value());
            params.update_step_size(step_size);
        }

        if (paramNode->first_node("sig_obs")){
            double cost_sigma = atof(paramNode->first_node("sig_obs")->value());
            params.update_sig_obs(cost_sigma);
        }

        if (paramNode->first_node("total_time")){
            double total_time = atof(paramNode->first_node("total_time")->value());
            params.set_total_time(total_time);
        }

        if (paramNode->first_node("alpha")){
            double alpha = atof(paramNode->first_node("alpha")->value());
            params.update_alpha(alpha);
        }

        std::string source_root{XSTRING(SOURCE_ROOT)};
        std::string saving_prefix_relative = static_cast<std::string>(paramNode->first_node("saving_prefix")->value());
        std::string saving_prefix = source_root + "/../" + saving_prefix_relative;
        params.set_saving_prefix(saving_prefix);

    }

};

template <typename PCSOptimizer>
class PCSRunnerBase: public ExperimentRunner<PGCSParams>{
    
public:
    // PCSRunner(){}
    virtual ~PCSRunnerBase(){}
    
    PCSRunnerBase(int nx, int nu, int num_exp, const std::string & config): 
                    ExperimentRunner<PGCSParams>(nx, nu, num_exp, config)
                    {
                        this->read_config(_params);
                    }

    void read_boundary_conditions(const rapidxml::xml_node<>* paramNode){
        this->read_boundary_conditions(paramNode, _params);
    }

    double run_and_save(PCSOptimizer & opt_sdf,
                    const PGCSParams & params, 
                    const rapidxml::xml_node<>* paramNode){
        
        Timer timer;
        timer.start();

        PGCSResultHistory res_Kd;

        res_Kd = opt_sdf.backtrack();

        double time_spend = timer.end_sec();

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

        return time_spend;
    }

    virtual double run_one_exp(int exp, PGCSParams& params, bool verbose=true) = 0;

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
        std::string map_name = static_cast<std::string>(commonParams->first_node("map_name")->value());

        params = PGCSParams(this->_nx, this->_nu, eps_sdf, radius, eps, total_time, this->_nt, 
                            sig0, sigT, eta, stop_err, sig_obs, 
                            max_iterations, backtracking_ratio, max_n_backtracking, map_name);

        if (commonParams->first_node("sdf_file")){
            std::string sdf_file = static_cast<std::string>(commonParams->first_node("sdf_file")->value());
            params.update_sdf_file(sdf_file);
        }

    }

    virtual void read_boundary_conditions(const rapidxml::xml_node<>* paramNode, PGCSParams& params) = 0;

};

template <typename PCSOptimizer>
class PCSRunnerLinDynBase: public PCSRunnerBase<PCSOptimizer>{
public:
    // PCSRunner(){}
    virtual ~PCSRunnerLinDynBase(){}

    PCSRunnerLinDynBase(int nx, int nu, int num_exp, const std::string & config):
                        PCSRunnerBase<PCSOptimizer>(nx, nu, num_exp, config){}

double run_one_exp(int exp, PGCSParams& params, bool verbose=true)
{
    rapidxml::file<> xmlFile(this->_config_file.data()); // Default template is char

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
    
    PCSOptimizer opt_sdf(A0, a0, B0, pdyn, params);

    return this->run_and_save(opt_sdf, params, paramNode);
}
};

template <typename PCSOptimizer>
class PCSRunner: public PCSRunnerLinDynBase<PCSOptimizer>{
public:
    // PCSRunner(){}
    virtual ~PCSRunner(){}

    PCSRunner(int nx, int nu, int num_exp, const std::string & config): 
                    PCSRunnerLinDynBase<PCSOptimizer>(nx, nu, num_exp, config)
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


template <typename PCSOptimizer>
class PCSRunner3D: public PCSRunnerLinDynBase<PCSOptimizer>{
public:
    // PCSRunner(){}
    virtual ~PCSRunner3D(){}

    PCSRunner3D(int num_exp, const std::string & config):
                        PCSRunnerLinDynBase<PCSOptimizer>(6, 3, num_exp, config){}


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

template <typename PCSOptimizer>
class PCSRunner7D: public PCSRunnerLinDynBase<PCSOptimizer>{
public:
    // PCSRunner(){}
    virtual ~PCSRunner7D(){}

    PCSRunner7D(int num_exp, const std::string & config):
                        PCSRunnerLinDynBase<PCSOptimizer>(14, 7, num_exp, config){}
    

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

        Eigen::VectorXd m0_pos(7);
        m0_pos << start_1, start_2, start_3, start_4, start_5, start_6, start_7;
        m0.block(0, 0, 7, 1) = m0_pos;
        m0.block(7, 0, 7, 1) = Eigen::VectorXd::Zero(7);

        Eigen::VectorXd mT_pos(7);
        mT_pos << goal_1, goal_2, goal_3, goal_4, goal_5, goal_6, goal_7;
        mT.block(0, 0, 7, 1) = mT_pos;
        mT.block(7, 0, 7, 1) = Eigen::VectorXd::Zero(7);

        if (paramNode->first_node("total_time")){
            double total_time = atof(paramNode->first_node("total_time")->value());
            params.set_total_time(total_time);
        }

        if (paramNode->first_node("eta")){
            double eta = atof(paramNode->first_node("eta")->value());
            params.update_step_size(eta);
        }

        double sig_obs = atof(paramNode->first_node("sig_obs")->value());

        params.set_m0(m0);
        params.set_mT(mT);
        params.update_sig_obs(sig_obs);
    }

};

}