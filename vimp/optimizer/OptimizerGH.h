/**
 * @file OptimizerGH.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief The joint optimizer class using Gauss-Hermite quadrature. 
 * @version 0.1
 * @date 2022-03-07
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

#include <utility>
#include <memory>
#include <assert.h>
#include "../helpers/SparseInverseMatrix.h"
#include "../helpers/result_recorder.h"
#include "../helpers/data_io.h"

using namespace std;

namespace vimp{

template <typename FactorizedOptimizer>
class VIMPOptimizerGH{
public:
    /**
     * @brief Default Constructor
     */
    VIMPOptimizerGH(){}

    /**
     * @brief Construct a new VIMPOptimizerGH object
     * 
     * @param _vec_fact_optimizers vector of marginal optimizers
     * @param niters number of iterations
     */
    VIMPOptimizerGH(const std::vector<std::shared_ptr<FactorizedOptimizer>>& vec_fact_optimizers, int niters=10):
                                   _dim{vec_fact_optimizers[0]->Pk().cols()},
                                   _niters{niters},
                                   _sub_dim{vec_fact_optimizers[0]->Pk().rows()},
                                   _nsub_vars{vec_fact_optimizers.size()},
                                   _vec_factor_optimizers{std::move(vec_fact_optimizers)},
                                   _mu{VectorXd::Zero(_dim)},
                                   _precision{MatrixXd::Identity(_dim, _dim)},
                                   _inverser{_precision},
                                   _covariance{MatrixXd::Identity(_dim, _dim)},
                                   _res_recorder{_niters, _dim}{}
protected:
    /// optimization variables
    int _dim, _niters, _sub_dim, _nsub_vars;

    /// @param _vec_factor_optimizers Vector of marginal optimizers
    vector<std::shared_ptr<FactorizedOptimizer>> _vec_factor_optimizers;

    /// @param _mu mean
    /// @param _dmu incremental mean
    VectorXd _mu;
    MatrixXd _precision;
    
    /// Sparse matrix inverse helper, which utilizes the exact sparse pattern to do the matrix inversion
    dense_inverser _inverser;

    MatrixXd _covariance;

    /// Data and result storage
    VIMPResults _res_recorder;
    MatrixIO _matrix_io;

    /// step sizes by default
    double _step_size_precision = 0.9;
    double _step_size_mu = 0.9;
    double _step_size_base_mu = 0.55;
    double _step_size_base_precision = 0.05;

public:
/// **************************************************************
/// Optimizations related
    /**
     * @brief Function which computes one step of update.
     */
    void step();

    /**
     * @brief Verifier function which computes one step update in the Gaussian cost case, 
     * which has the closed form solution.
     */
    void step_closed_form();

    /**
     * @brief The optimizing process.
     */
    void optimize();

    /**
     * @brief Compute the total cost function value given a mean and covariace.
     */
    double cost_value(const VectorXd& x, const MatrixXd& Cov) const;

    /**
     * @brief Compute the total cost function value given a state, using current values.
     * @return cost value.
     */
    double cost_value() const;

    /**
     * @brief Compute the costs of all factors for a given mean and cov.
     * @param x mean
     * @param P covariance
     * @return VectorXd collection of factor costs
     */
    VectorXd factor_costs(const VectorXd& x, const MatrixXd& Cov) const;

    /**
     * @brief Compute the costs of all factors, using current values.
     * @return VectorXd collection of factor costs
     */
    VectorXd factor_costs() const;


/// **************************************************************
/// Internal data IO
    /// returns the mean
    inline VectorXd mean() const{ return _mu; }

    /// returns the precision matrix
    inline MatrixXd precision() const{ return _precision; }

    /// returns the covariance matrix
    inline MatrixXd covariance(){ return _precision.inverse(); }

    /**
     * @brief Purturb the mean by a random vector.
     * @param scale 
     * @return purturbed mean vector
     */
    inline VectorXd purturb_mean(double scale=0.01) const{
        return VectorXd{_mu + VectorXd::Random(_dim) * scale};
    }

    /**
     * @brief Purturb the precision by a random matrix.
     * @param scale 
     * @return purturbed precision matrix
     */
    inline MatrixXd purturb_precision(double scale=0.01) const{
        return MatrixXd{_precision + MatrixXd::Random(_dim, _dim) * scale};
    }   

    /**
     * @brief Purturb the variables and calculate the cost.
     * @param scale 
     * @return purturbed cost
     */
    inline double purturbed_cost(double scale=0.01) const{
        VectorXd p_mean = purturb_mean(scale);
        MatrixXd p_precision = purturb_precision(scale);
        return cost_value(p_mean, p_precision.inverse());
    }

    /// update the step sizes
    /// @param ss_mean new step size for the mean update
    /// @param ss_precision new step size for the precision matrix update
    inline void set_step_size(double ss_mean, double ss_precision){
        _step_size_mu = ss_mean;
        _step_size_precision = ss_precision; }

    /// The base step size in backtracking
    inline void set_step_size_base(double ss_base_mu, double ss_base_precision){
        _step_size_base_mu = ss_base_mu;
        _step_size_base_precision = ss_base_precision;
    }

    /// assign a mean 
    /// @param mean new mean
    inline void set_mu(const VectorXd& mean){
        assert(mean.size() == _mu.size());
        _mu = mean; 
        for (std::shared_ptr<FactorizedOptimizer> & opt_fact : _vec_factor_optimizers){
            opt_fact->update_mu_from_joint_mean(_mu);
        }
    }

    /// assign a precision matrix
    /// @param new_precision new precision
    inline void set_precision(const MatrixXd& new_precision){
        assert(new_precision.size() == _precision.size());
        _precision = new_precision;
        // _inverser.update_matrix(_precision);
        _covariance = _precision.inverse();

        for (auto & opt_fact : _vec_factor_optimizers){
            opt_fact->update_precision_from_joint_covariance(_covariance);
        }
    }

    /**
     * @brief set number of iterations
     * @param niters
     */
    inline void set_niterations(int niters){
        _niters = niters;
        _res_recorder.update_niters(niters); }

    /**
     * @brief Set initial values 
     */
    inline void set_initial_values(const VectorXd& init_mean, const MatrixXd& init_precision){
        set_mu(init_mean);
        set_precision(init_precision);
    }

    /**
     * @brief Set the degree of polynomial in gauss hermite integrator
     */
    inline void set_GH_degree(const int deg){
        for (auto & opt_fact : _vec_factor_optimizers){
            opt_fact->set_GH_points(deg);
        }
    }

    
/// **************************************************************
/// Experiment data and result recordings
    /**
     * @brief update filenames
     * @param file_mean filename for the means
     * @param file_cov filename for the covariances
     */
    inline void update_file_names(const string& file_mean, 
                                  const string& file_cov, 
                                  const string& file_precision, 
                                  const string& file_cost,
                                  const string& file_fac_costs){
        _res_recorder.update_file_names(file_mean, file_cov, file_precision, file_cost, file_fac_costs);
    }

    /**
     * @brief save process data into csv files.
     */
    inline void save_data(){
        _res_recorder.save_data();}


    /**
     * @brief save a matrix to a file. 
     */
    inline void save_matrix(const string& filename, const MatrixXd& m){
        _matrix_io.saveData<MatrixXd>(filename, m);
    }

    inline void save_vector(const string& filename, const VectorXd& vec){
        _matrix_io.saveData<VectorXd>(filename, vec);
    }

    /**
     * @brief print a given iteration data mean and covariance.
     * @param i_iter index of data
     */
    inline void print_result(const int& i_iter){
        _res_recorder.print_data(i_iter);}

    /**
     * @brief print out a given number of iterations results
     * @param iters a list of iterations to be printed
     */
    inline void print_series_results(const vector<int>& iters) {
        std::for_each(iters.begin(), iters.end(), [this](int i) { 
            cout << "--- result at iteration " << i << "---" << endl;
            print_result(i);
            }
        );

    }

    inline int dim() const{
        return _dim;
    }   


    inline int n_sub_factors() const{
        return _nsub_vars;
    }

    /**
     * @brief calculate and return the E_q{phi(x)} s for each factorized entity.
     * @return vector<double> 
     */
    vector<double> E_Phis(){
        vector<double> res;
        for (auto & p_opt: _vec_factor_optimizers){
            res.emplace_back(p_opt->E_Phi());
        }
        return res;
    }

    /**
     * @brief calculate and return the E_q{(x-mu).*phi(x)} s for each factorized entity.
     * @return vector<double> 
     */
    vector<MatrixXd> E_xMuPhis(){
        vector<MatrixXd> res;
        for (auto & p_opt: _vec_factor_optimizers){
            res.emplace_back(p_opt->E_xMuPhi());
        }
        return res;
    }

    /**
     * @brief calculate and return the E_q{(x-mu).*phi(x)} s for each factorized entity.
     * @return vector<double> 
     */
    vector<MatrixXd> E_xMuxMuTPhis(){
        vector<MatrixXd> res;
        for (auto & p_opt: _vec_factor_optimizers){
            res.emplace_back(p_opt->E_xMuxMuTPhi());
        }
        return res;
    }


    /**************************** ONLY FOR 1D CASE ***********************/
    /**
     * @brief Draw a heat map for cost function in 1d case
     * @return MatrixXd heatmap of size (nmesh, nmesh)
     */
    MatrixXd cost_map(const double& x_start, 
                      const double& x_end, const double& y_start, 
                      const double& y_end, const int& nmesh){
        double res_x = (x_end - x_start) / nmesh;
        double res_y = (y_end - y_start) / nmesh;
        MatrixXd Z = MatrixXd::Zero(nmesh, nmesh);

        for (int i=0; i<nmesh; i++){
            VectorXd mean{VectorXd::Constant(1, x_start + i*res_x)};
            for (int j=0; j<nmesh; j++){
                MatrixXd cov{MatrixXd::Constant(1, 1, 1/(y_start + j*res_y))};
                Z(j, i) = cost_value(mean, cov); /// the order of the matrix in cpp and in matlab
            }
        }
        cout << "Z(0,0) " << endl << Z(0,0) << endl;
        cout << "Z(1,0) " << endl << Z(1,0) << endl;
        cout << "Z(1,1) " << endl << Z(1,1) << endl;
        return Z;
    }

    /**
     * @brief save the cost map
     */
    void save_costmap(string filename="costmap.csv"){
        MatrixXd cost_m = cost_map(18, 25, 0.05, 1, 40);
        ofstream file(filename);
        if (file.is_open()){
            file << cost_m.format(CSVFormat);
            file.close();}
    }
    
    }; //class
} //namespace vimp

#include "../optimizer/OptimizerGH-impl.h"