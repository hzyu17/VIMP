/**
 * @file PriorCol.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Joint optimizer with prior and collision factors, 
 * templated for input type and robot model.
 * @version 0.1
 * @date 2022-07-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "../optimizer/OptimizerGH.h"
#include "../optimizer/OptimizerFactorizedThreeFactorsGH.h"
#include "GaussianPriorUnaryTranslation.h"
#include <gpmp2/obstacle/ObstaclePlanarSDFFactor.h>


namespace vimp{
    /**
     * @brief Declaration of the factorized optimizer.
     */
    template <typename T, typename ROBOT>
    using FunctionPriorVelCol = std::function<double(const VectorXd&, 
                                            const UnaryFactorTranslation<T>&, 
                                            const UnaryFactorTranslation<T>&, 
                                            const gpmp2::ObstaclePlanarSDFFactor<ROBOT>&)>;

    template <typename T, typename ROBOT>
    using OptFactPriVelColGH = VIMPOptimizerFactorizedThreeClassGH<FunctionPriorVelCol<T, ROBOT>, 
                                                                UnaryFactorTranslation<T>, 
                                                                UnaryFactorTranslation<T>, 
                                                                gpmp2::ObstaclePlanarSDFFactor<ROBOT>>;

}

