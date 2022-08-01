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

#include "../optimizer/OptimizerGH.h"
#include "../optimizer/OptimizerFactorizedTwoFactorsGH.h"
#include "GaussianPriorUnaryTranslation.h"
// #include "../gp/fixed_prior.h"
#include <gpmp2/obstacle/ObstaclePlanarSDFFactor.h>


namespace vimp{
    /**
     * @brief Declaration of the factorized optimizer.
     */
    template <typename T, typename ROBOT>
    using FunctionPriorCol = std::function<double(const VectorXd&, 
                                            const UnaryFactorTranslation<T>&, 
                                            const gpmp2::ObstaclePlanarSDFFactor<ROBOT>&)>;


    template <typename T, typename ROBOT>
    using OptFactPriColGH = VIMPOptimizerFactorizedTwoClassGH<FunctionPriorCol<T, ROBOT>, 
                                                                UnaryFactorTranslation<T>, 
                                                                gpmp2::ObstaclePlanarSDFFactor<ROBOT>>;


    template <typename T, typename ROBOT>
    class OptFactPriColGHInstance: public OptFactPriColGH<T, ROBOT>{
        public:
            /// Default constructor
            OptFactPriColGHInstance(){};
            
            /// Constructor
            OptFactPriColGHInstance(const int& dimension, 
                                      const UnaryFactorTranslation<T>& prior,
                                      const gpmp2::ObstaclePlanarSDFFactor<ROBOT>& collision,
                                      const MatrixXd& Pk): OptFactPriColGH<T, ROBOT>(dimension, 
                                                                                    errorWrapperPriorCol,
                                                                                    prior,
                                                                                    collision,
                                                                                    Pk){}

            static double errorWrapperPriorCol(const VectorXd& pose, 
                                                const UnaryFactorTranslation<T>& prior_factor,
                                                const gpmp2::ObstaclePlanarSDFFactor<ROBOT>& collision_factor) {

                /**
                 * Prior factor
                 * */
                VectorXd vec_prior_err = prior_factor.evaluateError(pose);
                MatrixXd Qc = prior_factor.get_Qc();
                double prior_err = vec_prior_err.transpose() * Qc.inverse() * vec_prior_err;

                /**
                 * Obstacle factor
                 * */
                VectorXd vec_err = collision_factor.evaluateError(pose);

                // MatrixXd precision_obs;
                MatrixXd precision_obs{MatrixXd::Identity(vec_err.rows(), vec_err.rows())};
                precision_obs = precision_obs / collision_factor.get_noiseModel()->sigmas()[0];

                double collision_cost = vec_err.transpose() * precision_obs * vec_err;

                return prior_err + collision_cost;
            }

    };

}

