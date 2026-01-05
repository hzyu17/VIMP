/**
 * @file ExperimentRunner.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Template for running motion planning experiments.
 * Reads configuration files and runs experiments with various optimizers.
 * GVIExperimentRunner and PGCSExperimentRunner are the main derived classes.
 * @version 0.1
 * @date 2023-04-14
 * 
 * @copyright Copyright (c) 2023
 */

#pragma once

#define STRING(x) #x
#define XSTRING(x) STRING(x)

#include "helpers/ExperimentParams.h"
#include "dynamics/LinearDynamics.h"
#include "thirdparty/rapidxml-1.13/rapidxml.hpp"
#include "thirdparty/rapidxml-1.13/rapidxml_utils.hpp"
#include "helpers/timer.h"
#include <memory>

namespace vimp {

using namespace Eigen;
using NominalHistory = std::tuple<std::vector<Matrix3D>, std::vector<Matrix3D>>;
using PGCSResultHistory = std::tuple<MatrixXd, MatrixXd, NominalHistory>;

//=============================================================================
// Helper functions for XML parsing
//=============================================================================

namespace xml_helpers {

inline double getDouble(const rapidxml::xml_node<>* node, const char* name) {
    return atof(node->first_node(name)->value());
}

inline int getInt(const rapidxml::xml_node<>* node, const char* name) {
    return atoi(node->first_node(name)->value());
}

inline std::string getString(const rapidxml::xml_node<>* node, const char* name) {
    return static_cast<std::string>(node->first_node(name)->value());
}

inline bool hasNode(const rapidxml::xml_node<>* node, const char* name) {
    return node->first_node(name) != nullptr;
}

template<typename T, typename Base>
inline void readOptional(const rapidxml::xml_node<>* node, const char* name, 
                         T& params, void (Base::*setter)(double)) {
    if (hasNode(node, name)) {
        (params.*setter)(getDouble(node, name));
    }
}

template<typename T, typename Base>
inline void readOptionalInt(const rapidxml::xml_node<>* node, const char* name,
                            T& params, void (Base::*setter)(int)) {
    if (hasNode(node, name)) {
        (params.*setter)(getInt(node, name));
    }
}

inline std::string buildSavingPrefix(const rapidxml::xml_node<>* paramNode) {
    std::string source_root{XSTRING(SOURCE_ROOT)};
    std::string saving_prefix_relative = getString(paramNode, "saving_prefix");
    return source_root + "/../" + saving_prefix_relative;
}

} // namespace xml_helpers

//=============================================================================
// Base ExperimentRunner
//=============================================================================

template <typename ExperimentParams>
class ExperimentRunner {
public:
    virtual ~ExperimentRunner() = default;
    
    ExperimentRunner(int nx, int nu, int num_exp, const std::string& config)
        : _nx(nx), _nu(nu), _num_exp(num_exp), _config_file(config) {}

    virtual void read_config(ExperimentParams& params) = 0;
    virtual void read_boundary_conditions(const rapidxml::xml_node<>* paramNode, 
                                          ExperimentParams& params) = 0;
    virtual double run_one_exp(int exp, ExperimentParams& params, bool verbose = true) = 0;

    void read_config() { read_config(_params); }

    virtual void run() {
        rapidxml::file<> xmlFile(_config_file.data());
        rapidxml::xml_document<> doc;
        doc.parse<0>(xmlFile.data());
        
        for (int i = 1; i <= _num_exp; ++i) {
            run_one_exp(i, _params);
        }
        std::cout << "========== End of all experiments ==========" << std::endl;
    }

protected:
    gvi::MatrixIO m_io;
    EigenWrapper _ei;
    std::string _config_file;
    int _num_exp, _nx, _nu, _nt;
    ExperimentParams _params;
};

//=============================================================================
// GVIMP Runner
//=============================================================================

template <typename GVIMPOptimizer>
class GVIMPRunner : public ExperimentRunner<GVIMPParams> {
public:
    virtual ~GVIMPRunner() = default;
    
    GVIMPRunner(int nx, int nu, int num_exp, const std::string& config)
        : ExperimentRunner<GVIMPParams>(nx, nu, num_exp, config) {
        read_config(_params);
        _gvimp_robotsdf = GVIMPOptimizer(_params);
    }
    
    void read_config(GVIMPParams& params) override {
        using namespace xml_helpers;
        
        rapidxml::file<> xmlFile(_config_file.data());
        rapidxml::xml_document<> doc;
        doc.parse<0>(xmlFile.data());
        
        auto* commonParams = doc.first_node("Commons");

        params = GVIMPParams(
            _nx, _nu,
            getDouble(commonParams, "total_time"),
            getInt(commonParams, "n_states"),
            getDouble(commonParams, "coeff_Qc"),
            getInt(commonParams, "GH_deg"),
            getDouble(commonParams, "sig_obs"),
            getDouble(commonParams, "eps_sdf"),
            getDouble(commonParams, "radius"),
            getDouble(commonParams, "step_size"),
            getInt(commonParams, "max_iterations"),
            getDouble(commonParams, "init_precision_factor"),
            getDouble(commonParams, "boundary_penalties"),
            getDouble(commonParams, "temperature"),
            getDouble(commonParams, "high_temperature"),
            getInt(commonParams, "low_temp_iterations"),
            getDouble(commonParams, "stop_err"),
            getInt(commonParams, "max_n_backtracking"),
            getString(commonParams, "map_name"),
            hasNode(commonParams, "sdf_file") ? getString(commonParams, "sdf_file") : "",
            getDouble(commonParams, "alpha")
        );
    }

    void read_boundary_conditions(const rapidxml::xml_node<>* paramNode, 
                                  GVIMPParams& params) override {
        using namespace xml_helpers;
        
        VectorXd m0(_nx), mT(_nx);
        
        auto* start = paramNode->first_node("start_pos");
        auto* goal = paramNode->first_node("goal_pos");
        
        if (_nx == 4) {
            m0 << getDouble(start, "x"), getDouble(start, "y"),
                  getDouble(start, "vx"), getDouble(start, "vy");
            mT << getDouble(goal, "x"), getDouble(goal, "y"),
                  getDouble(goal, "vx"), getDouble(goal, "vy");
        } else if (_nx == 6) {
            m0 << getDouble(start, "x"), getDouble(start, "y"), getDouble(start, "z"),
                  getDouble(start, "vx"), getDouble(start, "vy"), getDouble(start, "vz");
            mT << getDouble(goal, "x"), getDouble(goal, "y"), getDouble(goal, "z"),
                  getDouble(goal, "vx"), getDouble(goal, "vy"), getDouble(goal, "vz");
        }
        
        params.set_m0(m0);
        params.set_mT(mT);

        // Read optional parameters
        readOptional(paramNode, "step_size", params, &GVIMPParams::update_step_size);
        readOptionalInt(paramNode, "max_iterations", params, &GVIMPParams::update_max_iter);
        readOptionalInt(paramNode, "low_temp_iterations", params, &GVIMPParams::update_lowtemp_iter);
        readOptional(paramNode, "init_precision_factor", params, &GVIMPParams::update_initial_precision_factor);
        readOptional(paramNode, "sig_obs", params, &GVIMPParams::update_sig_obs);
        readOptional(paramNode, "total_time", params, &GVIMPParams::set_total_time);
        readOptional(paramNode, "temperature", params, &GVIMPParams::set_temperature);
        readOptional(paramNode, "high_temperature", params, &GVIMPParams::set_high_temperature);
        readOptional(paramNode, "boundary_penalties", params, &GVIMPParams::update_boundary_penalties);
        readOptional(paramNode, "alpha", params, &GVIMPParams::update_alpha);

        params.set_saving_prefix(buildSavingPrefix(paramNode));
    }

    double run_one_exp(int exp, GVIMPParams& params, bool verbose = true) override {
        rapidxml::file<> xmlFile(_config_file.data());
        rapidxml::xml_document<> doc;
        doc.parse<0>(xmlFile.data());
        
        std::string expNodeName = "Experiment" + std::to_string(exp);
        auto* paramNode = doc.first_node(expNodeName.data());

        std::cout << expNodeName << std::endl;
        std::cout << "----- Reading Boundary Conditions -----" << std::endl;
        
        read_boundary_conditions(paramNode, params);

        if (verbose) {
            params.print_params();
        }
        
        return _gvimp_robotsdf.run_optimization_withtime(params, verbose);
    }

    std::tuple<VectorXd, SpMat> run_one_exp_return(GVIMPParams& params, bool verbose = true) {
        params.print_params();
        return _gvimp_robotsdf.run_optimization_return(params, verbose);
    }

    std::tuple<VectorXd, SpMat> get_mu_precision() {
        return _gvimp_robotsdf.get_mu_precision();
    }

    GVIMPOptimizer optimizer_robot_sdf() const { return _gvimp_robotsdf; }

protected:
    GVIMPOptimizer _gvimp_robotsdf;
};

//=============================================================================
// GVIMP Runner for Quadrotor (6D nonlinear dynamics)
//=============================================================================

template <typename GVIMPOptimizer>
class GVIMPRunner_Quadrotor : public ExperimentRunner<GVIMPParams_nonlinear> {
public:
    virtual ~GVIMPRunner_Quadrotor() = default;
    
    GVIMPRunner_Quadrotor(int nx, int nu, int num_exp, const std::string& config)
        : ExperimentRunner<GVIMPParams_nonlinear>(nx, nu, num_exp, config) {
        read_config(_params);
        _gvimp_robotsdf = GVIMPOptimizer(_params);
    }

    void read_config(GVIMPParams_nonlinear& params) override {
        using namespace xml_helpers;
        
        rapidxml::file<> xmlFile(_config_file.data());
        rapidxml::xml_document<> doc;
        doc.parse<0>(xmlFile.data());
        
        auto* commonParams = doc.first_node("Commons");

        params = GVIMPParams_nonlinear(
            _nx, _nu,
            getDouble(commonParams, "total_time"),
            getInt(commonParams, "n_states"),
            getDouble(commonParams, "coeff_Qc"),
            getInt(commonParams, "GH_deg"),
            getDouble(commonParams, "sig_obs"),
            getDouble(commonParams, "eps_sdf"),
            getDouble(commonParams, "radius"),
            getDouble(commonParams, "step_size"),
            getInt(commonParams, "max_iterations"),
            getDouble(commonParams, "init_precision_factor"),
            getDouble(commonParams, "boundary_penalties"),
            getDouble(commonParams, "temperature"),
            getDouble(commonParams, "high_temperature"),
            getInt(commonParams, "low_temp_iterations"),
            getDouble(commonParams, "stop_err"),
            getInt(commonParams, "max_n_backtracking"),
            getString(commonParams, "map_name"),
            "",
            getDouble(commonParams, "alpha"),
            getInt(commonParams, "max_linear_iter")
        );
    }

    void read_boundary_conditions(const rapidxml::xml_node<>* paramNode,
                                  GVIMPParams_nonlinear& params) override {
        using namespace xml_helpers;
        
        VectorXd m0(_nx), mT(_nx);
        
        auto* start = paramNode->first_node("start_pos");
        auto* goal = paramNode->first_node("goal_pos");
        
        m0 << getDouble(start, "x"), getDouble(start, "z"), getDouble(start, "phi"),
              getDouble(start, "vx"), getDouble(start, "vz"), getDouble(start, "vphi");
        mT << getDouble(goal, "x"), getDouble(goal, "z"), getDouble(goal, "phi"),
              getDouble(goal, "vx"), getDouble(goal, "vz"), getDouble(goal, "vphi");

        params.set_m0(m0);
        params.set_mT(mT);
        
        // Read optional parameters
        readOptional(paramNode, "step_size", params, &GVIMPParams_nonlinear::update_step_size);
        readOptionalInt(paramNode, "max_iterations", params, &GVIMPParams_nonlinear::update_max_iter);
        readOptionalInt(paramNode, "low_temp_iterations", params, &GVIMPParams_nonlinear::update_lowtemp_iter);
        readOptional(paramNode, "init_precision_factor", params, &GVIMPParams_nonlinear::update_initial_precision_factor);
        readOptional(paramNode, "sig_obs", params, &GVIMPParams_nonlinear::update_sig_obs);
        readOptional(paramNode, "total_time", params, &GVIMPParams_nonlinear::set_total_time);
        readOptional(paramNode, "temperature", params, &GVIMPParams_nonlinear::set_temperature);
        readOptional(paramNode, "high_temperature", params, &GVIMPParams_nonlinear::set_high_temperature);
        readOptional(paramNode, "boundary_penalties", params, &GVIMPParams_nonlinear::update_boundary_penalties);
        readOptional(paramNode, "alpha", params, &GVIMPParams_nonlinear::update_alpha);
        readOptionalInt(paramNode, "max_linear_iter", params, &GVIMPParams_nonlinear::update_max_linear_iter);
        
        if (hasNode(paramNode, "map_name")) {
            params.update_map_name(getString(paramNode, "map_name"));
        }

        params.set_saving_prefix(buildSavingPrefix(paramNode));
    }

    double run_one_exp(int exp, GVIMPParams_nonlinear& params, bool verbose = true) override {
        rapidxml::file<> xmlFile(_config_file.data());
        rapidxml::xml_document<> doc;
        doc.parse<0>(xmlFile.data());
        
        std::string expNodeName = "Experiment" + std::to_string(exp);
        auto* paramNode = doc.first_node(expNodeName.data());

        std::cout << expNodeName << std::endl;
        std::cout << "----- Reading Boundary Conditions -----" << std::endl;
        
        read_boundary_conditions(paramNode, params);

        if (verbose) {
            params.print_params();
        }
        
        return _gvimp_robotsdf.run_optimization_withtime(params, verbose);
    }

    std::tuple<VectorXd, SpMat> run_one_exp_return(GVIMPParams_nonlinear& params, bool verbose = true) {
        params.print_params();
        return _gvimp_robotsdf.run_optimization_return(params, verbose);
    }

    std::tuple<VectorXd, SpMat> get_mu_precision() {
        return _gvimp_robotsdf.get_mu_precision();
    }

    GVIMPOptimizer optimizer_robot_sdf() const { return _gvimp_robotsdf; }

private:
    GVIMPOptimizer _gvimp_robotsdf;
};

//=============================================================================
// GVIMP Runner for 7DOF Arms
//=============================================================================

template <typename GVIOptimizer>
class GVIMPRunner7D : public GVIMPRunner<GVIOptimizer> {
public:
    virtual ~GVIMPRunner7D() = default;

    GVIMPRunner7D(int num_exp, const std::string& config)
        : GVIMPRunner<GVIOptimizer>(14, 7, num_exp, config) {}

    void read_boundary_conditions(const std::string& config_file, int i_exp, GVIMPParams& params) {
        rapidxml::file<> xmlFile(config_file.data());
        rapidxml::xml_document<> doc;
        doc.parse<0>(xmlFile.data());
        
        std::string expNodeName = "Experiment" + std::to_string(i_exp);
        auto* paramNode = doc.first_node(expNodeName.data());
        
        read_boundary_conditions(paramNode, params);
    }

    void read_boundary_conditions(const rapidxml::xml_node<>* paramNode, 
                                  GVIMPParams& params) override {
        using namespace xml_helpers;
        
        auto* start = paramNode->first_node("start_pos");
        auto* goal = paramNode->first_node("goal_pos");
        
        VectorXd m0(this->_nx), mT(this->_nx);
        VectorXd m0_pos(7), mT_pos(7);
        
        for (int i = 0; i < 7; ++i) {
            std::string idx = std::to_string(i + 1);
            m0_pos(i) = getDouble(start, idx.c_str());
            mT_pos(i) = getDouble(goal, idx.c_str());
        }
        
        m0.head(7) = m0_pos;
        m0.tail(7).setZero();
        mT.head(7) = mT_pos;
        mT.tail(7).setZero();

        params.set_m0(m0);
        params.set_mT(mT);

        readOptional(paramNode, "temperature", params, &GVIMPParams::set_temperature);
        readOptional(paramNode, "high_temperature", params, &GVIMPParams::set_high_temperature);
        readOptional(paramNode, "step_size", params, &GVIMPParams::update_step_size);
        readOptional(paramNode, "sig_obs", params, &GVIMPParams::update_sig_obs);
        readOptional(paramNode, "total_time", params, &GVIMPParams::set_total_time);
        readOptional(paramNode, "alpha", params, &GVIMPParams::update_alpha);

        params.set_saving_prefix(buildSavingPrefix(paramNode));
    }
};

//=============================================================================
// PCS Runner Base
//=============================================================================

template <typename PCSOptimizer>
class PCSRunnerBase : public ExperimentRunner<PGCSParams> {
public:
    virtual ~PCSRunnerBase() = default;
    
    PCSRunnerBase(int nx, int nu, int num_exp, const std::string& config)
        : ExperimentRunner<PGCSParams>(nx, nu, num_exp, config) {
        read_config(_params);
    }

    void read_boundary_conditions(const rapidxml::xml_node<>* paramNode) {
        read_boundary_conditions(paramNode, _params);
    }

    double run_and_save(PCSOptimizer& opt_sdf, const PGCSParams& params,
                        const rapidxml::xml_node<>* paramNode) {
        Timer timer;
        timer.start();

        auto res_Kd = opt_sdf.backtrack();
        double time_spend = timer.end_sec();

        MatrixXd Kt = std::get<0>(res_Kd);
        MatrixXd dt = std::get<1>(res_Kd);

        NominalHistory ztSigt = std::get<2>(res_Kd);
        Matrix3D h_zt = _ei.vec2mat3d(std::get<0>(ztSigt));
        Matrix3D h_Sigt = _ei.vec2mat3d(std::get<1>(ztSigt));

        MatrixXd zk_star = opt_sdf.zkt();
        MatrixXd Sk_star = opt_sdf.Sigkt();

        std::string saving_prefix = xml_helpers::getString(paramNode, "saving_prefix");

        m_io.saveData(saving_prefix + "zk_sdf.csv", zk_star);
        m_io.saveData(saving_prefix + "Sk_sdf.csv", Sk_star);
        m_io.saveData(saving_prefix + "zk_history.csv", h_zt);
        m_io.saveData(saving_prefix + "Sk_history.csv", h_Sigt);
        m_io.saveData(saving_prefix + "Kt_sdf.csv", Kt);
        m_io.saveData(saving_prefix + "dt_sdf.csv", dt);
        m_io.saveData(saving_prefix + "hAkt_sdf.csv", opt_sdf.hAkt());
        m_io.saveData(saving_prefix + "hakt_sdf.csv", opt_sdf.hakt());
        
        opt_sdf.save_costs(saving_prefix + "costs.csv");

        return time_spend;
    }

    virtual double run_one_exp(int exp, PGCSParams& params, bool verbose = true) = 0;

    void read_config(PGCSParams& params) override {
        using namespace xml_helpers;
        
        rapidxml::file<> xmlFile(_config_file.data());
        rapidxml::xml_document<> doc;
        doc.parse<0>(xmlFile.data());

        auto* commonParams = doc.first_node("Commons");

        _nt = getInt(commonParams, "nt");

        params = PGCSParams(
            _nx, _nu,
            getDouble(commonParams, "eps_sdf"),
            getDouble(commonParams, "radius"),
            getDouble(commonParams, "eps"),
            getDouble(commonParams, "total_time"),
            _nt,
            getDouble(commonParams, "sig0"),
            getDouble(commonParams, "sigT"),
            getDouble(commonParams, "eta"),
            getDouble(commonParams, "stop_err"),
            getDouble(commonParams, "sig_obs"),
            getInt(commonParams, "max_iter"),
            getDouble(commonParams, "backtracking_ratio"),
            getInt(commonParams, "max_n_backtracking"),
            getString(commonParams, "map_name")
        );

        if (hasNode(commonParams, "sdf_file")) {
            params.update_sdf_file(getString(commonParams, "sdf_file"));
        }
    }

    virtual void read_boundary_conditions(const rapidxml::xml_node<>* paramNode, 
                                          PGCSParams& params) = 0;
};

//=============================================================================
// PCS Runner with Linear Dynamics Base
//=============================================================================

template <typename PCSOptimizer>
class PCSRunnerLinDynBase : public PCSRunnerBase<PCSOptimizer> {
public:
    virtual ~PCSRunnerLinDynBase() = default;

    PCSRunnerLinDynBase(int nx, int nu, int num_exp, const std::string& config)
        : PCSRunnerBase<PCSOptimizer>(nx, nu, num_exp, config) {}

    double run_one_exp(int exp, PGCSParams& params, bool verbose = true) override {
        rapidxml::file<> xmlFile(this->_config_file.data());
        rapidxml::xml_document<> doc;
        doc.parse<0>(xmlFile.data());
        
        std::string expNodeName = "Experiment" + std::to_string(exp);
        auto* paramNode = doc.first_node(expNodeName.data());
        
        this->read_boundary_conditions(paramNode, params);

        auto pdyn = std::make_shared<ConstantVelDynamics>(params.nx(), params.nu(), params.nt());
        PCSOptimizer opt_sdf(pdyn->A0(), pdyn->a0(), pdyn->B0(), pdyn, params);

        return this->run_and_save(opt_sdf, params, paramNode);
    }
};

//=============================================================================
// PCS Runner for 2D Point Robot
//=============================================================================

template <typename PCSOptimizer>
class PCSRunner : public PCSRunnerLinDynBase<PCSOptimizer> {
public:
    virtual ~PCSRunner() = default;

    PCSRunner(int nx, int nu, int num_exp, const std::string& config)
        : PCSRunnerLinDynBase<PCSOptimizer>(nx, nu, num_exp, config) {}

    void read_boundary_conditions(const rapidxml::xml_node<>* paramNode, 
                                  PGCSParams& params) override {
        using namespace xml_helpers;
        
        auto* start = paramNode->first_node("start_pos");
        auto* goal = paramNode->first_node("goal_pos");
        
        VectorXd m0(this->_nx), mT(this->_nx);
        m0 << getDouble(start, "x"), getDouble(start, "y"),
              getDouble(start, "vx"), getDouble(start, "vy");
        mT << getDouble(goal, "x"), getDouble(goal, "y"),
              getDouble(goal, "vx"), getDouble(goal, "vy");

        params.set_m0(m0);
        params.set_mT(mT);

        readOptional(paramNode, "eta", params, &PGCSParams::update_step_size);
        params.update_sig_obs(getDouble(paramNode, "sig_obs"));
        params.print_params();
    }
};

//=============================================================================
// PCS Runner for 3D Point Robot
//=============================================================================

template <typename PCSOptimizer>
class PCSRunner3D : public PCSRunnerLinDynBase<PCSOptimizer> {
public:
    virtual ~PCSRunner3D() = default;

    PCSRunner3D(int num_exp, const std::string& config)
        : PCSRunnerLinDynBase<PCSOptimizer>(6, 3, num_exp, config) {}

    void read_boundary_conditions(const rapidxml::xml_node<>* paramNode, 
                                  PGCSParams& params) override {
        using namespace xml_helpers;
        
        auto* start = paramNode->first_node("start_pos");
        auto* goal = paramNode->first_node("goal_pos");
        
        VectorXd m0(this->_nx), mT(this->_nx);
        m0 << getDouble(start, "x"), getDouble(start, "y"), getDouble(start, "z"),
              getDouble(start, "vx"), getDouble(start, "vy"), getDouble(start, "vz");
        mT << getDouble(goal, "x"), getDouble(goal, "y"), getDouble(goal, "z"),
              getDouble(goal, "vx"), getDouble(goal, "vy"), getDouble(goal, "vz");

        params.set_m0(m0);
        params.set_mT(mT);
        params.update_sdf_file(getString(paramNode, "sdf_file"));
        
        readOptional(paramNode, "eta", params, &PGCSParams::update_step_size);
        params.update_sig_obs(getDouble(paramNode, "sig_obs"));
    }
};

//=============================================================================
// PCS Runner for 7DOF Arms
//=============================================================================

template <typename PCSOptimizer>
class PCSRunner7D : public PCSRunnerLinDynBase<PCSOptimizer> {
public:
    virtual ~PCSRunner7D() = default;

    PCSRunner7D(int num_exp, const std::string& config)
        : PCSRunnerLinDynBase<PCSOptimizer>(14, 7, num_exp, config) {}

    void read_boundary_conditions(const rapidxml::xml_node<>* paramNode, 
                                  PGCSParams& params) override {
        using namespace xml_helpers;
        
        auto* start = paramNode->first_node("start_pos");
        auto* goal = paramNode->first_node("goal_pos");
        
        VectorXd m0(this->_nx), mT(this->_nx);
        VectorXd m0_pos(7), mT_pos(7);
        
        for (int i = 0; i < 7; ++i) {
            std::string idx = std::to_string(i + 1);
            m0_pos(i) = getDouble(start, idx.c_str());
            mT_pos(i) = getDouble(goal, idx.c_str());
        }
        
        m0.head(7) = m0_pos;
        m0.tail(7).setZero();
        mT.head(7) = mT_pos;
        mT.tail(7).setZero();

        params.set_m0(m0);
        params.set_mT(mT);

        readOptional(paramNode, "total_time", params, &PGCSParams::set_total_time);
        readOptional(paramNode, "eta", params, &PGCSParams::update_step_size);
        params.update_sig_obs(getDouble(paramNode, "sig_obs"));
    }
};

} // namespace vimp