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

#include <gtsam/base/Matrix.h>
#include <iostream>
#include <random>
#include <utility>
#include "SparseInverseMatrix.h"
#include <boost/scoped_ptr.hpp>

//using namespace GaussianSampler;
using namespace std;
using namespace SparseInverse;
typedef Triplet<double> T;

namespace MPVI{
    template <typename FactorizedOptimizer>


class VIMPOptimizerGH{
public:
    /// @param dimension State dimension
    /// @param _sub_dim dimension of the marginal variables
    /// @param _vec_function Vector of cost functions for each marginal distribution
    VIMPOptimizerGH(const vector<std::shared_ptr<FactorizedOptimizer>>& _vec_fact_optimizers):
                                   _dim{_vec_fact_optimizers[0]->Pk().cols()},
                                   _sub_dim{_vec_fact_optimizers[0]->Pk().rows()},
                                   _nsub_vars{_vec_fact_optimizers.size()},
                                //    vec_Pks_{std::move(_vec_Pks)},
                                   _vec_factor_optimizers{std::move(_vec_fact_optimizers)},
                                   _mu{VectorXd::Zero(_dim)},
                                   _Vdmu{VectorXd::Zero(_dim)},
                                   _dmu{VectorXd::Zero(_dim)},
                                   _precision{MatrixXd::Identity(_dim, _dim) * 5.0},
                                   _Vddmu(MatrixXd::Identity(_dim, _dim)),
                                   _dprecision(MatrixXd::Identity(_dim, _dim)),
                                   _inverser{MatrixXd::Identity(_dim, _dim)}{}
protected:
    /// optimization variables
    int _dim, _sub_dim, _nsub_vars;

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

            _Vdmu = _Vdmu + _vec_factor_optimizers[k]->get_joint_Vdmu();
            _Vddmu = _Vddmu + _vec_factor_optimizers[k]->get_joint_Vddmu();

        }

        _dprecision = -_precision + _Vddmu;
        _precision = _precision + step_size_precision*_dprecision;

        _dmu = -_precision.colPivHouseholderQr().solve(_Vdmu);
        _mu = _mu + _step_size_mu * _dmu;

        cout << "_mu " << endl << _mu << endl;
        MatrixXd covariance = get_covariance();
        cout << "new covariance " << endl << covariance << endl;
        
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

            _Vdmu = _Vdmu + optimizer_k->get_joint_Vdmu();
            _Vddmu = _Vddmu + optimizer_k->get_joint_Vddmu();
        }

        _dprecision = -_precision + _Vddmu;

        _precision = _precision + step_size_precision*_dprecision;

        _dmu = _precision.colPivHouseholderQr().solve(-_Vdmu);
        _mu = _mu + _step_size_mu * _dmu;

        cout << "_mu " << endl << _mu << endl;
        MatrixXd covariance = get_covariance();
        cout << "new covariance " << endl << covariance << endl;

    }

    /// returns the mean
    inline VectorXd get_mean() const{
        return _mu;
    }

    /// returns the precision matrix
    inline MatrixXd get_precision() const{
        return _precision;
    }

    /// returns the covariance matrix
    inline MatrixXd get_covariance(){
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
        _mu = mean;
    }

};

}
