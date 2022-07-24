/**
 * @file PriorCol.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Joint optimizer with prior and collision factors, 
 * templated for input type and robot model.
 * @version 0.1
 * @date 2022-07-17
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <vimp/optimizer/OptimizerGH.h>
#include <vimp/optimizer/OptimizerFactorizedTwoFactorsGH.h>
#include <vimp/instances/GaussianPriorUnaryTranslation.h>
#include <gpmp2/obstacle/ObstaclePlanarSDFFactor.h>


namespace vimp{
    /**
     * @brief Declaration of the factorized optimizer.
     * 
     */
    template <class T, class ROBOT>
    using FunctionPriorCol = std::function<double(const VectorXd&, 
                                            const UnaryFactorTranslation<T>&, 
                                            const gpmp2::ObstaclePlanarSDFFactor<ROBOT>&)>;

    template <class T, class ROBOT>
    using OptFactPriColGH = VIMPOptimizerFactorizedTwoClassGH<FunctionPriorCol<T, ROBOT>, 
                                                                UnaryFactorTranslation<T>, 
                                                                gpmp2::ObstaclePlanarSDFFactor<ROBOT>>;

}

