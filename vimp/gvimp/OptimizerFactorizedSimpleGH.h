/**
 * @file OptimizerFactorizedSimpleGH.h
 * @author Hongzhe Yu (hyu419@getach.edu)
 * @brief Simple optimizer which takes no gpmp factors, 
 * just to verify the algorithm itself.
 * @version 0.1
 * @date 2022-07-25
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "../gvimp/OptimizerFactorizedGHBase.h"
#include <memory>
#include "../helpers/repeated_includes.h"

using namespace std;
using namespace Eigen;

namespace vimp{
    template <typename Function>
    class VIMPOptimizerFactorizedSimpleGH: public VIMPOptimizerFactorizedBase{

        using OptBase = VIMPOptimizerFactorizedBase;
        using GHFunction = std::function<MatrixXd(const VectorXd&)>;

    public:
        ///@param dimension The dimension of the state
        ///@param function Template function class which calculate the cost
        VIMPOptimizerFactorizedSimpleGH(int dimension, int state_dim, int num_states, int start_index, const Function& function):
                OptBase(dimension, state_dim, num_states, start_index)
                {
                    /// Override of the base classes.
                    OptBase::_func_phi = [this, function](const VectorXd& x){return MatrixXd{MatrixXd::Constant(1, 1, function(x))};};
                    OptBase::_func_Vmu = [this, function](const VectorXd& x){return (x-OptBase::_mu) * function(x);};
                    OptBase::_func_Vmumu = [this, function](const VectorXd& x){return MatrixXd{(x-OptBase::_mu) * (x-OptBase::_mu).transpose().eval() * function(x)};};
                    OptBase::_gauss_hermite = GaussHermite<GHFunction>{10, OptBase::_dim, OptBase::_mu, OptBase::_covariance, OptBase::_func_phi};
                }

        
    public:
        typedef std::shared_ptr<VIMPOptimizerFactorizedSimpleGH> shared_ptr;

    };

    
}