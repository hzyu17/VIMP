/**
 * @file experimentParam.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Define the experiment parameters.
 * @version 0.1
 * @date 2023-05-23
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once

#include "helpers/MatrixIO.h"
#include "helpers/eigen_wrapper.h"

namespace vimp{

class GVIMPExperimentParams{
public:
    GVIMPExperimentParams(){}

    GVIMPExperimentParams(int nx,
                        int nu,
                        double total_time, 
                        int n_states, 
                        double coeff_Qc, 
                        double sigma_obs, 
                        double eps_sdf, 
                        double radius,
                        double step_size,
                        int num_iter,
                        double initial_precision_factor,
                        double boundary_penalties,
                        double temperature,
                        double high_temperature,
                        int low_temp_iterations,
                        double stop_err,
                        int max_n_backtracking):
                        _nx(nx),
                        _nu(nu),
                        _total_time(total_time),
                        _nt(n_states),
                        _coeff_Qc(coeff_Qc),
                        _sig_obs(sigma_obs),
                        _eps_sdf(eps_sdf),
                        _radius(radius),
                        _step_size(step_size),
                        _max_iterations(num_iter),
                        _initial_precision_factor(initial_precision_factor),
                        _boundary_penalties(boundary_penalties),
                        _temperature(temperature),
                        _high_temperature(high_temperature),
                        _low_temp_iterations(low_temp_iterations),
                        _stop_err(stop_err),
                        _max_n_backtrack(max_n_backtracking),
                        _m0(VectorXd::Zero(nx)),
                        _mT(VectorXd::Zero(nx)),
                        _Sig0(MatrixXd::Zero(nx, nx)),
                        _SigT(MatrixXd::Zero(nx, nx)){
    }

    // getters
    int nx() const { return _nx; }
    int nu() const { return _nu; }
    int nt() const { return _nt; }
    int max_iter() const { return _max_iterations; }

    double total_time() const { return _total_time;}
    double eps_sdf() const { return _eps_sdf; }
    double sig0() const { return _sig0; }
    double sigT() const { return _sigT; }
    double eta() const { return _eta; }
    double stop_err() const { return _stop_err; }
    double sig_obs() const { return _sig_obs; }
    double radius() const { return _radius; }
    double step_size() const {return _step_size; }
    double temperature() const { return _temperature; }
    double backtrack_ratio() const { return _backtrack_ratio; }
    double coeff_Qc() const { return _coeff_Qc; }
    double initial_precision_factor() const { return _initial_precision_factor; }
    double boundary_penalties() const { return _boundary_penalties; }
    int max_n_backtrack() const { return _max_n_backtrack; }
    std::string saving_prefix() const { return _save_prefix; }

    VectorXd m0() const { return _m0; }
    VectorXd mT() const { return _mT; }
    MatrixXd Sig0() const { return _Sig0; }
    MatrixXd SigT() const { return _SigT; }

    // setters
    void set_m0(const VectorXd& m0){ _m0 = m0; }
    void set_mT(const VectorXd& mT){ _mT = mT; }

    void update_sig_obs(double sig_obs){_sig_obs = sig_obs;}
    void set_temperature(double temperature){ _temperature = temperature; }
    void set_high_temperature(double high_temp){ _high_temperature = high_temp; }
    void set_boundary_penalties(double boundary_penalties){ _boundary_penalties = boundary_penalties; }
    void set_saving_prefix(const std::string& save_prefix){ _save_prefix = save_prefix; }

protected:
    MatrixIO _m_io;
    EigenWrapper _ei;
    VectorXd _m0, _mT;
    MatrixXd _Sig0, _SigT;

    std::string _save_prefix;

    double _total_time, _sig0, _sigT, _eta, _step_size, _stop_err, _backtrack_ratio;
    int _max_iterations, _low_temp_iterations, _max_n_backtrack;
    int _nt, _nx, _nu;
    double _coeff_Qc, _eps_sdf, _radius, _sig_obs, _initial_precision_factor, _boundary_penalties, _temperature, _high_temperature;

};

class PGCSExperimentParams{

public:
    PGCSExperimentParams(){}

    PGCSExperimentParams(int nx, int nu, double eps_sdf, double radius, double eps,
                    double speed, int nt, double sig0, double sigT, double eta, 
                    double stop_err, double sig_obs, int max_iterations, double backtracking_ratio, 
                    int max_n_backtracking, std::string sdf_file=""):
                                        _nx(nx),
                                        _nu(nu),
                                        _m0(VectorXd::Zero(nx)), 
                                        _mT(VectorXd::Zero(nx)),
                                        _sig0(sig0),
                                        _sigT(sigT),
                                        _Sig0(nx, nx), 
                                        _SigT(nx, nx),
                                        _nt(nt),
                                        _max_iterations(max_iterations),
                                        _speed(speed),
                                        _sig(nt*speed),
                                        _stop_err(stop_err),
                                        _eta(eta),
                                        _eps_sdf(eps_sdf),
                                        _radius(radius),
                                        _eps(eps),
                                        _sig_obs(sig_obs),
                                        _backtrack_ratio(backtracking_ratio),
                                        _max_n_backtrack(max_n_backtracking),
                                        _sdf_file(sdf_file){
                                            _Sig0 = MatrixXd::Identity(_nx, _nx) * sig0;
                                            _SigT = MatrixXd::Identity(_nx, _nx) * sigT;
                                         }

    void set_m0(const VectorXd& m0){ _m0 = m0; }
    void set_mT(const VectorXd& mT){ _mT = mT; }

    void set_sigobs(double sig_obs){ _sig_obs = sig_obs; }
    void set_speed(double speed){ _speed = speed; _sig = speed*_nt; }

    // getters
    int nx() const { return _nx; }
    int nu() const { return _nu; }
    int nt() const { return _nt; }
    int max_iter() const { return _max_iterations; }

    double speed() const { return _speed;}
    double eps_sdf() const { return _eps_sdf; }
    double radius() const { return _radius; }
    double sig() const { return _sig;}
    double sig0() const { return _sig0; }
    double sigT() const { return _sigT; }
    double eta() const { return _eta; }
    double eps() const { return _eps; }
    double stop_err() const { return _stop_err; }
    double sig_obs() const { return _sig_obs; }
    double backtrack_ratio() const { return _backtrack_ratio; }
    int max_n_backtrack() const { return _max_n_backtrack; }
    std::string sdf_file() const { return _sdf_file; }

    VectorXd m0() const { return _m0; }
    VectorXd mT() const { return _mT; }
    MatrixXd Sig0() const { return _Sig0; }
    MatrixXd SigT() const { return _SigT; }

    void update_sig_obs(double sig_obs){_sig_obs = sig_obs;}
    void update_sdf_file(const std::string& sdf_file){ _sdf_file = sdf_file; }


protected:
    MatrixIO _m_io;
    EigenWrapper _ei;
    VectorXd _m0, _mT;
    MatrixXd _Sig0, _SigT;

    double _speed, _eps_sdf, _radius, _sig0, _sigT, _eta, _stop_err, _backtrack_ratio;
    int _nt, _max_iterations, _max_n_backtrack;
    int _nx, _nu;

    double _eps, _sig_obs, _sig;

    std::string _sdf_file;

};

}// namespace vimp