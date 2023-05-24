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

#include "data_io.h"
#include "eigen_wrapper.h"

namespace vimp{

class ExperimentParams{

public:
    ExperimentParams(){}

    ExperimentParams(int nx, int nu):
                        _nx(nx),
                        _nu(nu),
                        _m0(nx), 
                        _mT(nx),
                        _Sig0(nx,nx), 
                        _SigT(nx,nx),
                        _sig0(0),
                        _sigT(0),
                        _nt(0),
                        _max_iterations(0),
                        _speed(0),
                        _sig(1.0),
                        _stop_err(0),
                        _eta(0),
                        _eps_sdf(0),
                        _sig_obs(0){}

    ExperimentParams(int nx, int nu, double eps_sdf, double eps,
                    double speed, int nt, double sig0, 
                    double sigT, double eta, double stop_err, double sig_obs,
                    int max_iterations, std::string sdf_file=""):_nx(nx),
                                        _nu(nu),
                                        _m0(nx), 
                                        _mT(nx),
                                        _sig0(sig0),
                                        _sigT(sigT),
                                        _Sig0(sig0 * Eigen::MatrixXd::Identity(_nx, _nx)), 
                                        _SigT(sigT * Eigen::MatrixXd::Identity(_nx, _nx)),
                                        _nt(nt),
                                        _max_iterations(max_iterations),
                                        _speed(speed),
                                        _sig(nt*speed),
                                        _stop_err(stop_err),
                                        _eta(eta),
                                        _eps_sdf(eps_sdf),
                                        _eps(eps),
                                        _sig_obs(sig_obs),
                                        _sdf_file(sdf_file){}

    void set_m0(const VectorXd& m0){ _m0 = m0; }

    void set_mT(const VectorXd& mT){ _mT = mT; }

    void set_sigobs(double sig_obs){ _sig_obs = sig_obs; }

    // getters
    const int nx(){ return _nx; }
    const int nu(){return _nu; }
    const int nt(){return _nt; }
    const int max_iter(){return _max_iterations; }

    const double speed(){return _speed;}
    const double eps_sdf(){return _eps_sdf; }
    const double sig(){return _sig;}
    const double sig0(){return _sig0; }
    const double sigT(){return _sigT; }
    const double eta(){return _eta; }
    const double eps(){return _eps; }
    const double stop_err(){return _stop_err; }
    const double sig_obs(){return _sig_obs; }
    const std::string sdf_file(){ return _sdf_file; }

    const VectorXd m0(){return _m0; }
    const VectorXd mT(){return _mT; }
    const MatrixXd Sig0(){return _Sig0; }
    const MatrixXd SigT(){return _SigT; }


protected:
    MatrixIO _m_io;
    EigenWrapper ei;
    VectorXd _m0, _mT;
    MatrixXd _Sig0, _SigT;

    double _speed, _eps_sdf, _sig0, _sigT, _eta, _stop_err;
    int _nt, _max_iterations;
    int _nx, _nu;

    double _eps, _sig_obs;

    double _sig;

    std::string _sdf_file;

};

}// namespace vimp