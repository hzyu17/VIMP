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
#include "SparseInverseMatrix.h"
#include "../helpers/result_recorder.h"
#include <assert.h>

//using namespace GaussianSampler;
using namespace std;
using namespace SparseInverse;
typedef Triplet<double> T;

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
    VIMPOptimizerGH(const vector<std::shared_ptr<FactorizedOptimizer>>& vec_fact_optimizers, int niters=10):
                                   _dim{vec_fact_optimizers[0]->Pk().cols()},
                                   _niters{niters},
                                   _sub_dim{vec_fact_optimizers[0]->Pk().rows()},
                                   _nsub_vars{vec_fact_optimizers.size()},
                                   _vec_factor_optimizers{std::move(vec_fact_optimizers)},
                                   _mu{VectorXd::Zero(_dim)},
                                   _precision{MatrixXd::Identity(_dim, _dim) * 5.0},
                                   _inverser{MatrixXd::Identity(_dim, _dim)},
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

    /// Data and result storage
    VIMPResults _res_recorder;

    /// step sizes by default
    double _step_size_precision = 0.9;
    double _step_size_mu = 0.9;

public:
/// **************************************************************
/// Optimizations related
    /**
     * @brief Function which computes one step of update.
     * 
     */
    void step();

    /**
     * @brief Verifier function which computes one step update in the Gaussian cost case, 
     * which has the closed form solution.
     */
    void step_closed_form();

    /**
     * @brief The optimizing process.
     * 
     */
    void optimize();

    /**
     * @brief Compute the total cost function value given a state.
     * 
     * @param x input vector.
     * @param P input Covariance
     * @return cost value.
     */
    double cost_value(const VectorXd& x, const MatrixXd& P);

    /**
     * @brief Compute the total cost function value given a state, using current values.
     * 
     * @return cost value.
     */
    double cost_value();


/// **************************************************************
/// Internal data IO
    /// returns the mean
    inline VectorXd mean() const{ return _mu; }

    /// returns the precision matrix
    inline MatrixXd precision() const{ return _precision; }

    /// returns the covariance matrix
    inline MatrixXd covariance(){ return _inverser.inverse(); }

    /**
     * @brief Purturb the mean by a random vector.
     * 
     * @param scale 
     * @return purturbed mean vector
     */
    inline VectorXd purturb_mean(double scale=0.1) const{
        return VectorXd{_mu + VectorXd::Random(_dim) * scale};
    }

    /**
     * @brief Purturb the precision by a random matrix.
     * 
     * @param scale 
     * @return purturbed precision matrix
     */
    inline MatrixXd purturb_precision(double scale=0.1) const{
        return MatrixXd{_precision + MatrixXd::Random(_dim, _dim) * scale};
    }   

    /// update the step sizes
    /// @param ss_mean new step size for the mean update
    /// @param ss_precision new step size for the precision matrix update
    inline void set_step_size(double ss_mean, double ss_precision){
        _step_size_mu = ss_mean;
        _step_size_precision = ss_precision; }

    /// assign a mean 
    /// @param mean new mean
    inline void set_mu(const VectorXd& mean){
        assert(mean.size() == _mu.size());
        _mu = mean; }

    /// assign a precision matrix
    /// @param mean new mean
    inline void set_precision(const MatrixXd& new_precision){
        assert(new_precision.size() == _precision.size());
        _precision = new_precision;}

    /**
     * @brief set number of iterations
     * 
     * @param niters
     */
    inline void set_niterations(int niters){
        _niters = niters;
        _res_recorder.update_niters(niters); }

    
/// **************************************************************
/// Experiment data and result recordings
    /**
     * @brief update filenames
     * 
     * @param file_mean filename for the means
     * @param file_cov filename for the covariances
     */
    inline void update_file_names(const string& file_mean, const string& file_cov){
        _res_recorder.update_file_names(file_mean, file_cov);}

    /**
     * @brief save process data into csv files.
     */
    inline void save_data(){
        _res_recorder.save_data();}

    /**
     * @brief print a given iteration data mean and covariance.
     * 
     * @param i_iter index of data
     */
    inline void print_result(const int& i_iter){
        _res_recorder.print_data(i_iter);}

    /**
     * @brief print out a given number of iterations results
     * 
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
    

};

}
