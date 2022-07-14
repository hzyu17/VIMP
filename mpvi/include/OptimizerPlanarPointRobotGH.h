/**
 * @file OptimizerPlanarPointRobotGH.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief 
 * @version 0.1
 * @date 2022-07-07
 * 
 * @copyright Copyright (c) 2022
 * 
 */

/// Description: The joint optimizer for a planar point robot case. The costs should include: 
///                 a) the prior and collision for each support states,
///                 b) the prior and collision for all interpolated states,
///                 c) the prior for the start and goal states.

#include <gtsam/base/Matrix.h>
#include <iostream>
#include <random>
#include <utility>
#include "SparseInverseMatrix.h"

#include "OptimizerFactorizedPriorCollisionPointRobot.h"
#include "OptimizerFactorizedPriorPointRobot.h"
#include <boost/scoped_ptr.hpp>

//using namespace GaussianSampler;
using namespace std;
using namespace SparseInverse;
typedef Triplet<double> T;

namespace MPVI{
    template <typename FunctionPrior, typename FunctionPriorCol>

/// Description: The joint optimizer class using Gauss-Hermite quadrature. 
/// 
/// Detailed description: Joint optimizer, all the factorized optimizers use the same cost functional.
class VIMPOptimizerPlanarPointRobotGH{
    // using FactorizedOptimizerPriorGH = OptimizerFactorPriorPointRobotGH<FunctionPrior, ArgsPrior ...>;
    // using FactorizedOptimizerPriorColGH = OptimizerFactorPriorColPointRobotGH<FunctionPriorCol, ArgsPriCol ...>;

public:
    /// @param dimension State dimension
    /// @param sub_dim dimension of the marginal variables
    /// @param _vec_func_priors Vector of factorized cost functions for prior cost
    /// @param _vec_func_prior_cols Vector of factorized cost functions for prior + collision cost
    /// @param _vec_Pks_prior Vector of matrices mapping from joint variables to marginal variables for prior factors
    /// @param _vec_Pks_prior_col Vector of matrices mapping from joint variables to marginal variables for prior+collision factors 
    VIMPOptimizerPlanarPointRobotGH(const int& dimension, const int& sub_dim, const vector<FunctionPrior>& _vec_func_priors,
                    const vector<FunctionPriorCol>& _vec_func_prior_cols, const vector<MatrixXd>& _vec_Pks_prior, const vector<MatrixXd>& _vec_Pks_prior_col):
                                   dim{dimension},
                                   sub_dim{sub_dim},
                                   num_sub_vars{static_cast<int>(_vec_Pks_prior_col.size())},
                                   vec_cost_func_prior_col_{_vec_func_prior_cols},
                                   vec_cost_func_prior_{_vec_func_priors},
                                   vecPks_prior_{_vec_Pks_prior},
                                   vecPks_prior_col_{_vec_Pks_prior_col},
                                   mu_{VectorXd::Zero(dimension)},
                                   d_mu_{VectorXd::Zero(dimension)},
                                   precision_{MatrixXd::Identity(dim, dim) * 5.0},
                                   d_precision_(MatrixXd::Identity(dim, dim)),
                                   Vdmu_{VectorXd::Zero(dimension)},
                                   Vddmu_(MatrixXd::Identity(dim, dim)),
                                   inverser_{MatrixXd::Identity(dim, dim)}
    {}
protected:
    /// optimization variables
    int dim, sub_dim, num_sub_vars;

    /// @param mu_ mean vector
    /// @param d_mu_ incremental mean in update steps
    /// @param precision_ precision matrix
    /// @param d_precision_ incremental precision matrix in update steps
    VectorXd mu_, Vdmu_, d_mu_;
    MatrixXd precision_, Vddmu_, d_precision_;

    /// @param vector of marginal optimizers
    vector<OptimizerFactorPriorPointRobotGH> vec_factor_optimizers_prior_;
    vector<OptimizerFactorPriorColPointRobotGH> vec_factor_optimizers_prior_col_;

    ///@param vec_cost_function_ Vector of the marginal cost functions for prior and collisions
    const vector<FunctionPriorCol> vec_cost_func_prior_col_;
    
    ///@param vec_cost_function_ Vector of the marginal cost functions for priors
    const vector<FunctionPrior> vec_cost_func_prior_;

    ///@param vecPks_prior_ Vector of all mapping matrix from joint variables to marginal variables, for the prior factor optimizer
    const vector<MatrixXd> vecPks_prior_;

    ///@param vecPks_prior_col_ Vector of all mapping matrix from joint variables to marginal variables, for the prior + collision factor optimizer
    const vector<MatrixXd> vecPks_prior_col_;

    /// Sparse matrix inverse helper, which utilizes the exact sparse pattern to do the matrix inversion
    dense_inverser inverser_;
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

            const MatrixXd& Pk_prior = vecPks_prior_[k];
            const MatrixXd& Pk_prior_col = vecPks_prior_col_[k];

            auto &optimizer_prior_k = vec_factor_optimizers_prior_[k];
            auto &optimizer_prior_col_k = vec_factor_optimizers_prior_col_[k];
            
            /// update for the prior factor
            optimizer_prior_k.update_mu(VectorXd{Pk_prior*mu_});
            optimizer_prior_k.update_precision(MatrixXd{(Pk_prior * Sigma * Pk_prior.transpose()).inverse()});
            optimizer_prior_k.calculate_partial_V();

            Vdmu_ = Vdmu_ + Pk_prior.transpose() * optimizer_prior_k.get_Vdmu();
            Vddmu_ = Vddmu_ + Pk_prior.transpose().eval() * optimizer_prior_k.get_Vddmu() * Pk_prior;

            /// update for the prior + collision factor
            optimizer_prior_col_k.update_mu(VectorXd{Pk_prior_col * mu_});
            optimizer_prior_col_k.update_precision(MatrixXd{(Pk_prior_col * Sigma * Pk_prior_col.transpose()).inverse()});
            optimizer_prior_col_k.calculate_partial_V();

            Vdmu_ = Vdmu_ + Pk_prior_col.transpose() * optimizer_prior_col_k.get_Vdmu();
            Vddmu_ = Vddmu_ + Pk_prior_col.transpose().eval() * optimizer_prior_col_k.get_Vddmu() * Pk_prior_col;

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
