/**
 * @file GVIFactorizedSimpleGH.h
 * @author Hongzhe Yu (hyu419@getach.edu)
 * @brief Simple optimizer which takes no gpmp factors, 
 * just to verify the algorithm itself.
 * @version 0.1
 * @date 2022-07-25
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "gvimp/GVIFactorizedBase.h"
#include <memory>
#include "helpers/repeated_includes.h"

using namespace std;
using namespace Eigen;

namespace vimp{
    template <typename Function>
    class GVIFactorizedSimpleGH: public GVIFactorizedBase{

        using OptBase = GVIFactorizedBase;
        using GHFunction = std::function<MatrixXd(const VectorXd&)>;
        using GH = GaussHermite<GHFunction>;

    public:
        ///@param dimension The dimension of the state
        ///@param function Template function class which calculate the cost
        GVIFactorizedSimpleGH(int dimension, int state_dim, int num_states, int start_index, const Function& function):
                OptBase(dimension, state_dim, num_states, start_index)
                {
                    /// Override of the base classes.
                    OptBase::_func_phi = std::make_shared<GHFunction>([this, function](const VectorXd& x){return MatrixXd{MatrixXd::Constant(1, 1, function(x))};});
                    OptBase::_func_Vmu = std::make_shared<GHFunction>([this, function](const VectorXd& x){return (x-OptBase::_mu) * function(x);});
                    OptBase::_func_Vmumu = std::make_shared<GHFunction>([this, function](const VectorXd& x){return MatrixXd{(x-OptBase::_mu) * (x-OptBase::_mu).transpose().eval() * function(x)};});
                    OptBase::_gh = std::make_shared<GH>(GH{10, OptBase::_dim, OptBase::_mu, OptBase::_covariance});
                }

        
    public:
        typedef std::shared_ptr<GVIFactorizedSimpleGH> shared_ptr;

    };

    
}