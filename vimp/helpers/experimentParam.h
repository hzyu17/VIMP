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

    ExperimentParams(int nx, int nu, double eps_sdf, double eps,
                    double speed, int nt, double sig0, 
                    double sigT, double eta, double stop_err, double sig_obs,
                    int max_iterations, std::string sdf_file=""):
                                        _nx(nx),
                                        _nu(nu),
                                        _m0(nx), 
                                        _mT(nx),
                                        _sig0(sig0),
                                        _sigT(sigT),
                                        _Sig0(_nx, _nx), 
                                        _SigT(_nx, _nx),
                                        _nt(nt),
                                        _max_iterations(max_iterations),
                                        _speed(speed),
                                        _sig(nt*speed),
                                        _stop_err(stop_err),
                                        _eta(eta),
                                        _eps_sdf(eps_sdf),
                                        _eps(eps),
                                        _sig_obs(sig_obs),
                                        _sdf_file(sdf_file){
                                            _m0.setZero();
                                            _mT.setZero();
                                            _Sig0 = MatrixXd::Identity(_nx, _nx) * sig0;
                                            _SigT = MatrixXd::Identity(_nx, _nx) * sigT;
                                         }

    void set_m0(const VectorXd& m0){ _m0 = m0; }

    void set_mT(const VectorXd& mT){ _mT = mT; }

    void set_sigobs(double sig_obs){ _sig_obs = sig_obs; }

    // getters
    int nx() const { return _nx; }
    int nu() const { return _nu; }
    int nt() const { return _nt; }
    int max_iter() const { return _max_iterations; }

    double speed() const { return _speed;}
    double eps_sdf() const { return _eps_sdf; }
    double sig() const { return _sig;}
    double sig0() const { return _sig0; }
    double sigT() const { return _sigT; }
    double eta() const { return _eta; }
    double eps() const { return _eps; }
    double stop_err() const { return _stop_err; }
    double sig_obs() const { return _sig_obs; }
    std::string sdf_file() const { return _sdf_file; }

    VectorXd m0() const { return _m0; }
    VectorXd mT() const { return _mT; }
    MatrixXd Sig0() const { return _Sig0; }
    MatrixXd SigT() const { return _SigT; }


protected:
    MatrixIO _m_io;
    EigenWrapper _ei;
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