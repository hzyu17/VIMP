#include "instances/PriorColPlanarPointRobot.h"
#include "robots/PlanarPointRobotSDFMultiObsExample.h"
#include <gtest/gtest.h>

using namespace gpmp2;
using namespace Eigen;
using namespace vimp;
using namespace std;


TEST(ColCost, change_covariance){
    // An example pr and sdf
    vimp::PlanarPointRobotSDFMultiObsExample planar_pr_sdf;
    gpmp2::PointRobotModel pRModel = std::move(planar_pr_sdf.pRmodel());
    gpmp2::PlanarSDF sdf = std::move(planar_pr_sdf.sdf());

    /// Obs factor
    double cost_sigma = 0.5, epsilon = 4.0;

    /// Vector of base factored optimizers
    vector<std::shared_ptr<VIMPOptimizerFactorizedBase>> vec_factor_opts;

    // collision factor
    gpmp2::ObstaclePlanarSDFFactorPointRobot collision_k{gtsam::symbol('x', 0), pRModel, sdf, cost_sigma, epsilon};

    MatrixXd Pk_col{MatrixXd::Zero(2, 4)};
    Pk_col.block(0, 0, 2, 2) = std::move(MatrixXd::Identity(2, 2));

    /// Factored optimizer
    std::shared_ptr<OptPlanarSDFFactorPointRobot> p_obs{new OptPlanarSDFFactorPointRobot{2, cost_sdf_pR, collision_k, Pk_col}};
    vec_factor_opts.emplace_back(p_obs);

    /// The joint optimizer
    VIMPOptimizerGH<VIMPOptimizerFactorizedBase> optimizer{vec_factor_opts};

    /// Set initial value to the linear interpolation
    VectorXd joint_init_theta{(VectorXd(4) << 11.3321059864421, 9.16117246531728, 0, 0).finished()};
    optimizer.set_mu(joint_init_theta);

    optimizer.set_GH_degree(6);
    MatrixXd precision = MatrixXd::Identity(4, 4);
    optimizer.set_precision(precision);

    double cost = optimizer.cost_value_no_entropy();
    cout << "cost_w_o_entropy " << endl << cost << endl;

    precision = precision * 100000.0;
    optimizer.set_precision(precision);
    double cost_new = optimizer.cost_value_no_entropy();
    cout << "cost_w_o_entropy " << endl << cost_new << endl;
    double cost_expected = 6.932105970254905*6.932105970254905/0.5;

    cout << "cost_expected " << endl << cost_expected << endl;

    ASSERT_LE(abs(cost_new - cost_expected), 1e-4);



}
