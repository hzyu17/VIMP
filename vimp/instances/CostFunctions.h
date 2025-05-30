#include "GaussianVI/gp/cost_functions.h"
#include <gpmp2/obstacle/ObstaclePlanarSDFFactor.h>
#include <gpmp2/obstacle/ObstacleSDFFactor.h>

#include "thirdparty/rapidxml-1.13/rapidxml.hpp"
#include "thirdparty/rapidxml-1.13/rapidxml_utils.hpp"

/**
 * Obstacle factor: planar case
 * */
template <typename ROBOT>
double cost_obstacle_planar(const VectorXd& pose, 
                            const gpmp2::ObstaclePlanarSDFFactor<ROBOT>& obs_factor){
    VectorXd vec_err = obs_factor.evaluateError(pose);

    MatrixXd precision_obs{MatrixXd::Identity(vec_err.rows(), vec_err.rows())};
    precision_obs = precision_obs / obs_factor.get_noiseModel()->sigmas()[0];

    return vec_err.transpose().eval() * precision_obs * vec_err;

}


/**
 * Obstacle factor
 * */
template <typename ROBOT>
double cost_obstacle(const VectorXd& pose, 
                    const gpmp2::ObstacleSDFFactor<ROBOT>& obs_factor){
    // std::cout << "point in cost function= " << pose.transpose() << std::endl;
    VectorXd vec_err = obs_factor.evaluateError(pose);

    // MatrixXd precision_obs;
    MatrixXd precision_obs{MatrixXd::Identity(vec_err.rows(), vec_err.rows())};
    precision_obs = precision_obs / obs_factor.get_noiseModel()->sigmas()[0];

    // std::cout << "obs cost" << std::endl 
    //           << vec_err.transpose().eval() * precision_obs * vec_err << std::endl;

    return vec_err.transpose().eval() * precision_obs * vec_err;

}