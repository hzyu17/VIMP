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
#include "helpers/EigenWrapper.h"

namespace vimp{

class Params{
public:
    Params(){}

    Params(int nx, int nu, double total_time, int n_states, 
            double sigma_obs, double eps_sdf, double radius,
            double step_size, int num_iter, double stop_err,
            double backtracking_ratio, int max_n_backtracking, 
            std::string map_name="map0", std::string sdf_file=""):
            _nx(nx),
            _nu(nu),
            _total_time(total_time),
            _nt(n_states),
            _eps_sdf(eps_sdf),
            _sig_obs(sigma_obs),
            _radius(radius),
            _step_size(step_size),
            _max_iterations(num_iter),
            _stop_err(stop_err),
            _backtrack_ratio(backtracking_ratio),
            _max_n_backtrack(max_n_backtracking),
            _m0(VectorXd::Zero(nx)),
            _mT(VectorXd::Zero(nx)),
            _Sig0(MatrixXd::Zero(nx, nx)),
            _SigT(MatrixXd::Zero(nx, nx)),
            _map_name(map_name)
            {}

public:
    inline int nx() const { return _nx; }
    inline int nu() const { return _nu; }
    inline int nt() const { return _nt; }
    inline int max_iter() const { return _max_iterations; }

    inline double total_time() const { return _total_time;}
    inline double eps_sdf() const { return _eps_sdf; }
    inline double sig0() const { return _sig0; }
    inline double sigT() const { return _sigT; }
    inline double stop_err() const { return _stop_err; }
    inline double sig_obs() const { return _sig_obs; }
    inline double radius() const { return _radius; }
    inline double step_size() const {return _step_size; }
    inline double backtrack_ratio() const { return _backtrack_ratio; }

    // inline std::string field_file() const { return _field_file; }
    inline std::string sdf_file() const { return _sdf_file; }
    inline std::string map_name() const { return _map_name;}


    inline int max_n_backtrack() const { return _max_n_backtrack; }
    inline std::string saving_prefix() const { return _save_prefix; }

    inline VectorXd m0() const { return _m0; }
    inline VectorXd mT() const { return _mT; }
    inline MatrixXd Sig0() const { return _Sig0; }
    inline MatrixXd SigT() const { return _SigT; }

    // setters
    inline void set_m0(const VectorXd& m0){ _m0 = m0; }
    inline void set_mT(const VectorXd& mT){ _mT = mT; }

    inline void set_Sig0(const MatrixXd& Sig0){ _Sig0 = Sig0; }
    inline void set_SigT(const MatrixXd& SigT){ _SigT = SigT; }

    inline void set_total_time(double total_time){ _total_time = total_time; }
    inline void update_sig_obs(double sig_obs){_sig_obs = sig_obs;}
    inline void update_step_size(double step_size){ _step_size = step_size; }
    inline void update_max_iter(int max_iter){ _max_iterations = max_iter; }

    inline void set_saving_prefix(const std::string& save_prefix){ _save_prefix = save_prefix; }

    // inline void update_field_file(const std::string& field){ _field_file = field; }
    inline void update_sdf_file(const std::string& sdf_file){ _sdf_file = sdf_file; }
    inline void update_map_name(const std::string& map_name){ _map_name = map_name; }
    

    virtual inline void print_params() = 0; 

protected:
    double _total_time, _sig0, _sigT, _eta, _step_size, _stop_err, _backtrack_ratio;
    int _nt, _nx, _nu;
    double _eps_sdf, _radius, _sig_obs;

    int _max_iterations, _max_n_backtrack;

    MatrixIO _m_io;
    EigenWrapper _ei;
    VectorXd _m0, _mT;
    MatrixXd _Sig0, _SigT;

    std::string _save_prefix;
    std::string _sdf_file, _map_name;
};

class GVIMPParams: public Params{
public:
    GVIMPParams(){}

    GVIMPParams(int nx,
                int nu,
                double total_time, 
                int n_states, 
                double coeff_Qc, 
                double sig_obs, 
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
                int max_n_backtracking,
                std::string map_name="map0",
                std::string sdf_file=""
                ):
                Params(nx,
                        nu,
                        total_time, 
                        n_states, 
                        sig_obs, 
                        eps_sdf, 
                        radius,
                        step_size,
                        num_iter,
                        stop_err,
                        1,
                        max_n_backtracking,
                        map_name,
                        sdf_file),
                _coeff_Qc(coeff_Qc),
                _initial_precision_factor(initial_precision_factor),
                _boundary_penalties(boundary_penalties),
                _temperature(temperature),
                _high_temperature(high_temperature),
                _max_iter_lowtemp(low_temp_iterations)
                { }

    // getters
    
    double coeff_Qc() const { return _coeff_Qc; }
    double initial_precision_factor() const { return _initial_precision_factor; }
    double boundary_penalties() const { return _boundary_penalties; }
    double temperature() const { return _temperature; }
    double high_temperature() const { return _high_temperature; }
    int max_iter_lowtemp() const { return _max_iter_lowtemp; }
    
    void set_temperature(double temperature){ _temperature = temperature; }
    void set_high_temperature(double high_temp){ _high_temperature = high_temp; }
    void set_boundary_penalties(double boundary_penalties){ _boundary_penalties = boundary_penalties; }
    
    void inline print_params() override {
        std::cout << "================ Experiment Parameters for GVI-MP ================" << std::endl 
        << " State dimension:           " << this->nx() << std::endl 
        << " Control dimension:         " << this->nu() << std::endl 
        << " Total time span:           " << this->total_time() << std::endl 
        << " coeff_Qc:                  " << this->coeff_Qc() << std::endl 
        << " Time discretizations:      " << this->nt() << std::endl 
        << " Temperature:               " << this->temperature() << std::endl 
        << " High temperature:          " << this->high_temperature() << std::endl 
        << " Map name:                  " << this->map_name() << std::endl 
        << " Map eps:                   " << this->eps_sdf() << std::endl 
        << " Cost sigma:                " << this->sig_obs() << std::endl 
        << " Robot radius:              " << this->radius() << std::endl 
        << " boundary_penalties:        " << this->boundary_penalties() << std::endl 
        << " step size:                 " << this->step_size() << std::endl 
        << " max iterations:            " << this->max_iter() << std::endl 
        << " max iterations lowtemp:    " << this->max_iter_lowtemp() << std::endl 
        << " Backtrack ratio:           " << this->backtrack_ratio() << std::endl 
        << " backtrack iterations:      " << this->max_n_backtrack() << std::endl
        << " initial conditions:        " << std::endl;
        _ei.print_matrix(this->m0(), "m0");
        std::cout << " terminal conditions:       " << std::endl;
        _ei.print_matrix(this->mT(), "mT");
        
    }

protected:    
    int _max_iter_lowtemp;
    double _coeff_Qc, _initial_precision_factor, _boundary_penalties, _temperature, _high_temperature;

};

class PGCSParams: public Params{

public:
    PGCSParams(){}

    PGCSParams(int nx, int nu, double eps_sdf, double radius,
                    double eps, double total_time, int nt, double sig0, double sigT, double step_size, 
                    double stop_err, double sig_obs, int num_iter, double backtracking_ratio, 
                    int max_n_backtracking, std::string map_name="map0", std::string sdf_file=""):
                    Params(nx,
                            nu,
                            total_time, 
                            nt, 
                            sig_obs, 
                            eps_sdf, 
                            radius,
                            step_size,
                            num_iter,
                            stop_err,
                            backtracking_ratio,
                            max_n_backtracking,
                            map_name,
                            sdf_file),
                            _eps(eps),
                            _sig0(sig0),
                            _sigT(sigT)
                            {
                                Params::_Sig0 = MatrixXd::Identity(_nx, _nx) * sig0;
                                Params::_SigT = MatrixXd::Identity(_nx, _nx) * sigT;
                            }

    // getters
    double sig0() const { return _sig0; }
    double sigT() const { return _sigT; }
    inline double eps() const { return _eps; }

    inline void print_params() override { 
        std::cout << "================ Experiment Parameters for PGCS-MP ================" << std::endl 
        << " State dimension:           " << this->nx() << std::endl 
        << " Control dimension:         " << this->nu() << std::endl 
        << " Total time span:           " << this->total_time() << std::endl 
        << " Time discretizations:      " << this->nt() << std::endl 
        << " Noise eps:                 " << this->eps() << std::endl 
        << " Map name:                  " << this->map_name() << std::endl 
        << " Map eps:                   " << this->eps_sdf() << std::endl 
        << " Cost sigma:                " << this->sig_obs() << std::endl 
        << " Robot radius:              " << this->radius() << std::endl 
        << " sig_0:                     " << this->sig0() << std::endl 
        << " sig_T:                     " << this->sigT() << std::endl 
        << " step size:                 " << this->step_size() << std::endl 
        << " max iterations:            " << this->max_iter() << std::endl 
        << " Backtrack ratio:           " << this->backtrack_ratio() << std::endl 
        << " backtrack iterations:      " << this->max_n_backtrack() << std::endl;
    }

protected:
    double _sig0, _sigT, _eps;

};


}// namespace vimp