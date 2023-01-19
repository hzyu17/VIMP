#include "../3rd-part/rapidxml-1.13/rapidxml.hpp"
#include "../3rd-part/rapidxml-1.13/rapidxml_utils.hpp"

#include "../gp/fixed_prior.h"
#include "../gp/minimum_acc_prior.h"
#include <gpmp2/obstacle/ObstaclePlanarSDFFactor.h>


// const char* config_file = "/home/hongzhe/git/VIMP/vimp/configs/planar_pR_map2.xml";
const char* config_file = "/home/hongzhe/git/VIMP/vimp/configs/planar_pR_map3.xml";
// const char* config_file = "/home/hongzhe/git/VIMP/vimp/configs/planar_2link_arm.xml";
// const char* config_file = "/home/hongzhe/git/VIMP/vimp/configs/planar_2link_arm_map2.xml";
// const char* config_file = "/home/hongzhe/git/VIMP/vimp/configs/planar_3link_arm_map1.xml";

/**
 * @brief Fixed cost with a covariance
 * @param x input
 * @param fixed_gp 
 * @return double cost
 */
double cost_fixed_gp(const VectorXd& x, const vimp::FixedPriorGP& fixed_gp){
    return fixed_gp.cost(x);
}

// std::function<double(const VectorXd&, const vimp::FixedPriorGP&)> cost_fixed_gp = [&](const VectorXd& x, const vimp::FixedPriorGP& fixed_gp){
//     return cost_fixed_gp_base(x, fixed_gp);
//     };


/**
 * @brief cost for the linear gaussian process.
 * @param pose : the combined two secutive states. [x1; v1; x2; v2]
 * @param gp_minacc : the linear gp object
 * @return double, the cost (-logporb)
 */
double cost_linear_gp(const VectorXd& pose_cmb, const vimp::MinimumAccGP& gp_minacc){
    int dim = gp_minacc.dim_posvel();
    // VectorXd x1 = pose_cmb.segment(0, dim);
    // VectorXd x2 = pose_cmb.segment(dim, dim);

    return gp_minacc.cost(pose_cmb.segment(0, dim), pose_cmb.segment(dim, dim));
}

// std::function<double(const VectorXd&, const vimp::MinimumAccGP&)> cost_linear_gp = [&](const VectorXd& x, const vimp::MinimumAccGP& gp_minacc){   
//     return cost_linear_gp_base(x, gp_minacc);
//     };


/**
 * Obstacle factor
 * */
template <typename ROBOT>
double cost_obstacle(const VectorXd& pose, 
                    const gpmp2::ObstaclePlanarSDFFactor<ROBOT>& obs_factor){
    
    VectorXd vec_err = obs_factor.evaluateError(pose);

    // MatrixXd precision_obs;
    MatrixXd precision_obs{MatrixXd::Identity(vec_err.rows(), vec_err.rows())};
    precision_obs = precision_obs / obs_factor.get_noiseModel()->sigmas()[0];

    return vec_err.transpose().eval() * precision_obs * vec_err;

}

// template <typename ROBOT>
// std::function<double(const VectorXd&, const gpmp2::ObstaclePlanarSDFFactor<ROBOT>&)> cost_obstacle = [&](const VectorXd& x, const gpmp2::ObstaclePlanarSDFFactor<ROBOT>& obs_factor){
//     return cost_obstacle_base(x, obs_factor);
// };