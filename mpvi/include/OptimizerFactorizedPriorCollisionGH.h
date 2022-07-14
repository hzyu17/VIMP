/**
 * @file OptimizerFactorizedPriorCollisionGH.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief 
 * @version 0.1
 * @date 2022-05-26
 * 
 * @copyright Copyright (c) 2022
 * 
 */

using namespace MPVI;

#include "OptimizerFactorizedGH.h"

// IOFormat CleanFmt(4, 0, ", ", "\n");
namespace MPVI{
    template <typename Function, typename PriorClass, typename CollisionClass, typename... Args>
     /// Description: Inheritance of the factorized optimizer, added two templates: the prior and collision class.
     ///
     /// Detailed description: The class takes the prior and collision class as two new templates. These templates will be instantiated in its
     /// children classes.
    class OptimizerFactorizedPriorColGH: public VIMPOptimizerFactorizedGaussHermite<Function, Args ...>{
    public:
        OptimizerFactorizedPriorColGH(const int& dimension, 
                                        Function _function, 
                                        PriorClass _prior, 
                                        CollisionClass _collision):
                prior_{_prior},
                collision_{_collision}
        {
            VIMPOptimizerFactorizedGaussHermite(dimension, _function);
        }
    protected:

        /// Caller of the cost function
        auto cost_function(Args... args){
            return cost_function_(args..., prior_, collision_);
        }

    private:
        PriorClass prior_;
        CollisionClass collision_;

    };
}

