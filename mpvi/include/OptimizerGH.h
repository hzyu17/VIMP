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

#include <random>
#include <utility>
#include "SparseInverseMatrix.h"
#include <boost/scoped_ptr.hpp>
#include "../include/helpers/result_recorder.h"

//using namespace GaussianSampler;
using namespace std;
using namespace SparseInverse;
typedef Triplet<double> T;

namespace MPVI{

template <typename FactorizedOptimizer>
class VIMPOptimizerGH{
public:
    /// @param _vec_fact_optimizers vector of marginal optimizers
    /// @param niters number of iterations
    VIMPOptimizerGH(const vector<std::shared_ptr<FactorizedOptimizer>>& _vec_fact_optimizers, int niters=10):
                                   _dim{_vec_fact_optimizers[0]->Pk().cols()},
                                   _niters{niters},
                                   _sub_dim{_vec_fact_optimizers[0]->Pk().rows()},
                                   _nsub_vars{_vec_fact_optimizers.size()},
                                   _vec_factor_optimizers{std::move(_vec_fact_optimizers)},
                                   _mu{VectorXd::Zero(_dim)},
                                   _Vdmu{VectorXd::Zero(_dim)},
                                   _dmu{VectorXd::Zero(_dim)},
                                   _precision{MatrixXd::Identity(_dim, _dim) * 5.0},
                                   _Vddmu(MatrixXd::Identity(_dim, _dim)),
                                   _dprecision(MatrixXd::Identity(_dim, _dim)),
                                   _inverser{MatrixXd::Identity(_dim, _dim)},
                                   _res_recorder{_niters, _dim}{}
protected:
    /// optimization variables
    int _dim, _niters, _sub_dim, _nsub_vars;

    ///@param vec_Pks_ Vector of all transformation matrix from joint variables to marginal variables
    // const vector<MatrixXd> vec_Pks_;

    /// @param _vec_factor_optimizers Vector of marginal optimizers
    vector<std::shared_ptr<FactorizedOptimizer>> _vec_factor_optimizers;

    /// @param _mu mean
    /// @param _dmu incremental mean
    /// @param _precision precision matrix
    /// @param _dprecision incremental precision matrix 
    VectorXd _mu, _Vdmu, _dmu;
    MatrixXd _precision, _Vddmu, _dprecision;

    /// Sparse matrix inverse helper, which utilizes the exact sparse pattern to do the matrix inversion
    dense_inverser _inverser;

    /// Data and result storage
    MPVIResults _res_recorder;

    /// step sizes by default
    double step_size_precision = 0.9;
    double _step_size_mu = 0.9;

public:
    /**
     * @brief Function which computes one step of update.
     * 
     */
    void step(){
        _Vdmu.setZero();
        _Vddmu.setZero();
        _dmu.setZero();
        _dprecision.setZero();

        MatrixXd Sigma{_inverser.inverse(_precision)};

        for (int k=0; k<_nsub_vars; k++){

            _vec_factor_optimizers[k]->update_mu(VectorXd{_mu});
            _vec_factor_optimizers[k]->update_precision_from_joint_covariance(MatrixXd{Sigma});
            _vec_factor_optimizers[k]->calculate_partial_V();

            _Vdmu = _Vdmu + _vec_factor_optimizers[k]->joint_Vdmu();
            _Vddmu = _Vddmu + _vec_factor_optimizers[k]->joint_Vddmu();

        }

        _dprecision = -_precision + _Vddmu;
        _precision = _precision + step_size_precision*_dprecision;

        _dmu = -_precision.colPivHouseholderQr().solve(_Vdmu);
        _mu = _mu + _step_size_mu * _dmu;

    }

    /**
     * @brief Verifier function which computes one step update in the Gaussian cost case, 
     * which has the closed form solution.
     */
    void step_closed_form(){

        _Vdmu.setZero();
        _Vddmu.setZero();
        _dmu.setZero();
        _dprecision.setZero();

        MatrixXd Sigma{_inverser.inverse(_precision)};

        for (int k=0; k<_nsub_vars; k++){

            std::shared_ptr<FactorizedOptimizer> optimizer_k = _vec_factor_optimizers[k];
            // optimizer_k->updateSamplerMean(VectorXd{_mu});
            // optimizer_k->updateSamplerCovarianceMatrix(MatrixXd{Sigma});

            optimizer_k->update_mu(VectorXd{_mu});
            optimizer_k->update_precision_from_joint_covariance(MatrixXd{Sigma});

            optimizer_k->calculate_exact_partial_V();

            _Vdmu = _Vdmu + optimizer_k->joint_Vdmu();
            _Vddmu = _Vddmu + optimizer_k->joint_Vddmu();
        }

        _dprecision = -_precision + _Vddmu;

        _precision = _precision + step_size_precision*_dprecision;

        _dmu = _precision.colPivHouseholderQr().solve(-_Vdmu);
        _mu = _mu + _step_size_mu * _dmu;

    }

    /// returns the mean
    inline VectorXd mean() const{
        return _mu;
    }

    /// returns the precision matrix
    inline MatrixXd precision() const{
        return _precision;
    }

    /// returns the covariance matrix
    inline MatrixXd covariance(){
        return _inverser.inverse();
    }

    /// update the step size of the iterations.
    /// @param ss_mean new step size for the mean update
    /// @param ss_precision new step size for the precision matrix update
    inline void set_step_size(double ss_mean, double ss_precision){
        _step_size_mu = ss_mean;
        step_size_precision = ss_precision;
    }

    /// manually assign a mean value
    /// @param mean new mean
    inline void set_mu(const VectorXd& mean){
        assert(mean.size() == _mu.size());
        _mu = mean;
    }

    /**
     * @brief Number of iterations
     * 
     * @param niters
     */
    inline void set_niterations(int niters){
        _niters = niters;
        _res_recorder.update_niters(niters);
    }

    /**
     * @brief The optimizing process.
     * 
     */
    void optimize(){
        double step_size = 0.9;
        for (int i = 0; i < _niters; i++) {
            step_size = step_size / pow((i + 1), 1 / 3);
            set_step_size(step_size, step_size);

            /// Collect the results
            VectorXd mean_iter{mean()};
            MatrixXd cov_iter{covariance()};

            cout << "iteration: " << i << endl;
            _res_recorder.update_data(mean_iter, cov_iter);
            step();
        }

        /// print 5 iteration datas 
        vector<int> iters{int(_niters/5), int(_niters*2/5), int(_niters*3/5), int(_niters*4/5), _niters-1};
        print_series_results(iters);

        save_data();
    } 

    /**
     * @brief update filenames
     * 
     * @param file_mean filename for the means
     * @param file_cov filename for the covariances
     */
    inline void update_file_names(const string& file_mean, const string& file_cov){
            _res_recorder.update_file_names(file_mean, file_cov);
        }

    /**
     * @brief save process data into csv files.
     */
    inline void save_data(){
        _res_recorder.save_data();
    }

    /**
     * @brief print a given iteration data mean and covariance.
     * 
     * @param i_iter index of data
     */
    inline void print_result(const int& i_iter){
        _res_recorder.print_data(i_iter);
    }

    /**
     * @brief print out a given number of iterations results
     * 
     * @param iters a list of iterations to be printed
     */
    inline void print_series_results(const vector<int>& iters){

        std::for_each(iters.begin(), iters.end(), [this](int i) { 
            cout << "--- result at iteration " << i << "---" << endl;
            print_result(i);
            }
        );

    }
    

};

}
