/**
 * @file OptimizerFactorizedGH.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief The marginal optimizer class expecting two functions, one is the cost function, f1(x, cost1); 
 * the other is the function for GH expectation, f2(x).
 * @version 0.1
 * @date 2022-03-07
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "../optimizer/OptimizerFactorizedGHBase.h"
#include <memory>

using namespace std;
using namespace Eigen;

namespace vimp{
    template <typename Function, typename CostClass>
    class VIMPOptimizerFactorizedGaussHermite: public VIMPOptimizerFactorizedBase{

        using OptBase = VIMPOptimizerFactorizedBase;
        using GHFunction = std::function<MatrixXd(const VectorXd&)>;

    public:
        ///@param dimension The dimension of the state
        ///@param function_ Template function class which calculate the cost
        VIMPOptimizerFactorizedGaussHermite(const int& dimension, const Function& function_, const CostClass& cost_class_, const MatrixXd& Pk_):
                OptBase(dimension, Pk_)
                {
                    /// Override of the base classes.
                    OptBase::_func_phi = [this, function_, cost_class_](const VectorXd& x){return MatrixXd{MatrixXd::Constant(1, 1, function_(x, cost_class_))};};
                    OptBase::_func_Vmu = [this, function_, cost_class_](const VectorXd& x){return (x-OptBase::_mu) * function_(x, cost_class_);};
                    OptBase::_func_Vmumu = [this, function_, cost_class_](const VectorXd& x){return MatrixXd{(x-OptBase::_mu) * (x-OptBase::_mu).transpose().eval() * function_(x, cost_class_)};};
                    OptBase::_gauss_hermite = GaussHermite<GHFunction>{10, OptBase::_dim, OptBase::_mu, OptBase::_covariance, OptBase::_func_phi};
                }
    public:
        typedef std::shared_ptr<VIMPOptimizerFactorizedGaussHermite> shared_ptr;

    };

    
}