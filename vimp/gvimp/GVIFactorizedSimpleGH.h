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
#include "helpers/common_includes.h"

using namespace std;
using namespace Eigen;

namespace vimp{
    template <typename Function>
    class GVIFactorizedSimpleGH: public GVIFactorizedBase{

        using Base = GVIFactorizedBase;
        using GHFunction = std::function<MatrixXd(const VectorXd&)>;
        using GH = GaussHermite<GHFunction>;

    public:
        ///@param dimension The dimension of the state
        ///@param function Template function class which calculate the cost
        GVIFactorizedSimpleGH(int dimension, int state_dim, int num_states, int start_index, const Function& function, 
                              double temperature=1.0, double high_temperature=10.0):
                Base(dimension, state_dim, num_states, start_index, temperature, high_temperature)
                {
                    /// Override of the base classes.
                    Base::_func_phi = [this, function](const VectorXd& x){return MatrixXd::Constant(1, 1, function(x));};
                    Base::_func_Vmu = [this, function](const VectorXd& x){return (x-Base::_mu) * function(x);};
                    Base::_func_Vmumu = [this, function](const VectorXd& x){return MatrixXd{(x-Base::_mu) * (x-Base::_mu).transpose().eval() * function(x)};};
                    Base::_gh = std::make_shared<GH>(GH{10, Base::_dim, Base::_mu, Base::_covariance, Base::_func_phi});
                }
        
    public:
        typedef std::shared_ptr<GVIFactorizedSimpleGH> shared_ptr;

    };

    
}