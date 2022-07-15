/**
 * @file OptimizerGH.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief 
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

/// Description: The joint optimizer class using Gauss-Hermite quadrature. 
/// 
/// Detailed description: Joint optimizer, all the factorized optimizers use the same cost functional.
class VIMPOptimizerGH{
public:
    /// @param dimension State dimension
    /// @param sub_dim dimension of the marginal variables
    /// @param _vec_function Vector of cost functions for each marginal distribution
    /// @param _vec_Pks Vector of matrices for the transition from joint variables to marginal variables 
    VIMPOptimizerGH(const vector<MatrixXd>& _vec_Pks, const vector<FactorizedOptimizer>& _vec_fact_optimizers):
                                   dim{_vec_Pks[0].cols()},
                                   sub_dim{_vec_Pks[0].rows()},
                                   num_sub_vars{static_cast<int>(_vec_Pks.size())},
                                   vec_Pks_{_vec_Pks},
                                   vec_factor_optimizers_{_vec_fact_optimizers},
                                   mu_{VectorXd::Zero(dim)},
                                   Vdmu_{VectorXd::Zero(dim)},
                                   d_mu_{VectorXd::Zero(dim)},
                                   precision_{MatrixXd::Identity(dim, dim) * 5.0},
                                   Vddmu_(MatrixXd::Identity(dim, dim)),
                                   d_precision_(MatrixXd::Identity(dim, dim)),
                                   inverser_{MatrixXd::Identity(dim, dim)}{}
protected:
    /// optimization variables
    int dim, sub_dim, num_sub_vars;

    ///@param vec_Pks_ Vector of all transformation matrix from joint variables to marginal variables
    const vector<MatrixXd> vec_Pks_;

    /// @param vec_factor_optimizers_ Vector of marginal optimizers
    vector<FactorizedOptimizer> vec_factor_optimizers_;

    /// @param mu_ mean vector
    /// @param d_mu_ incremental mean in update steps
    /// @param precision_ precision matrix
    /// @param d_precision_ incremental precision matrix in update steps
    VectorXd mu_, Vdmu_, d_mu_;
    MatrixXd precision_, Vddmu_, d_precision_;

    /// Sparse matrix inverse helper, which utilizes the exact sparse pattern to do the matrix inversion
    dense_inverser inverser_;

    /// step sizes by default
    double step_size_precision = 0.9;
    double step_size_mu = 0.9;

public:
    /// Function which computes one step of update.
    void step(){
        cout << "mu_ " << endl << mu_ << endl;

        Vdmu_.setZero();
        Vddmu_.setZero();
        d_mu_.setZero();
        d_precision_.setZero();

        MatrixXd Sigma{inverser_.inverse(precision_)};

        for (int k=0; k<num_sub_vars; k++){

            const MatrixXd& Pk = vec_Pks_[k];

            FactorizedOptimizer& optimizer_k = vec_factor_optimizers_[k];

            optimizer_k.update_mu(VectorXd{Pk*mu_});
            optimizer_k.update_precision(MatrixXd{(Pk * Sigma * Pk.transpose()).inverse()});
            optimizer_k.calculate_partial_V();

            Vdmu_ = Vdmu_ + Pk.transpose() * optimizer_k.get_Vdmu();
            Vddmu_ = Vddmu_ + Pk.transpose().eval() * optimizer_k.get_Vddmu() * Pk;

        }

        cout << "Vddmu_" << Vddmu_ << endl;
        cout << "Vdmu_" << Vdmu_ << endl;

        cout << "precision " << endl << precision_ << endl;

        d_precision_ = -precision_ + Vddmu_;

        precision_ = precision_ + step_size_precision*d_precision_;

        d_mu_ = precision_.colPivHouseholderQr().solve(-Vdmu_);

        mu_ = mu_ + step_size_mu * d_mu_;

        cout << "mu_ " << endl << mu_ << endl;
        cout << "new precision " << endl << precision_ << endl;
        
    }

    /// Verifier function which computes one step update in the Gaussian cost case, which has the closed form solution.
    void step_closed_form(){

        Vdmu_.setZero();
        Vddmu_.setZero();
        d_mu_.setZero();
        d_precision_.setZero();

        MatrixXd Sigma{inverser_.inverse(precision_)};

        for (int k=0; k<num_sub_vars; k++){

            const MatrixXd& Pk = vec_Pks_[k];

            FactorizedOptimizer& optimizer_k = vec_factor_optimizers_[k];
            optimizer_k.updateSamplerMean(VectorXd{Pk * mu_});
            optimizer_k.updateSamplerCovarianceMatrix(MatrixXd{Pk * Sigma * Pk.transpose()});

            optimizer_k.update_mu(VectorXd{Pk*mu_});
            optimizer_k.update_precision(MatrixXd{(Pk * Sigma * Pk.transpose()).inverse()});

            optimizer_k.calculate_partial_V();

            Vdmu_ = Vdmu_ + Pk.transpose() * optimizer_k.get_Vdmu();
            Vddmu_ = Vddmu_ + Pk.transpose().eval() * optimizer_k.get_Vddmu() * Pk;
        }

        d_precision_ = -precision_ + Vddmu_;

        precision_ = precision_ + step_size_precision*d_precision_;

        d_mu_ = precision_.colPivHouseholderQr().solve(-Vdmu_);
        mu_ = mu_ + step_size_mu * d_mu_;

        cout << "mu_ " << endl << mu_ << endl;
        cout << "new precision " << endl << precision_ << endl;

    }

    /// returns the mean
    gtsam::Vector get_mean(){
        return mu_;
    }

    /// returns the precision matrix
    gtsam::Matrix get_precision(){
        return precision_;
    }

    /// returns the covariance matrix
    gtsam::Matrix get_covariance(){
        return inverser_.inverse();
    }

    /// update the step size of the iterations.
    /// @param ss_mean new step size for the mean update
    /// @param ss_precision new step size for the precision matrix update
    void set_step_size(double ss_mean, double ss_precision){
        step_size_mu = ss_mean;
        step_size_precision = ss_precision;
    }

    /// manually assign a mean value
    /// @param mean new mean
    void set_mu(const VectorXd& mean){
        mu_ = mean;
    }

};

}
