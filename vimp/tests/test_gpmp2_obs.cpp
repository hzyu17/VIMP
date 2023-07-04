/**
 * @file test_gpmp2_obs.cpp
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Test the gpmp2 collision factor
 * @version 0.1
 * @date 2022-08-02
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <gpmp2/obstacle/ObstaclePlanarSDFFactorPointRobot.h>
#include "../robots/PlanarPointRobotSDFMultiObsExample.h"
#include "../gvimp/GaussHermite.h"
#include "../helpers/MatrixIO.h"
#include <gtest/gtest.h>

using namespace Eigen;
using namespace gpmp2;
using namespace vimp;

/// Functions copied from gpmp2

inline VectorXd errorWrapper(const ObstaclePlanarSDFFactorPointRobot& factor,
    const VectorXd& conf) {
  return factor.evaluateError(conf);
}

// convert sdf VectorXd to hinge loss err VectorXd
inline VectorXd convertSDFtoErr(const VectorXd& sdf, double eps) {
  VectorXd err_ori = 0.0 - sdf.array() + eps;
  return (err_ori.array() > 0.0).select(err_ori, VectorXd::Zero(err_ori.rows()));  // (R < s ? P : Q)
}


/**
 * Obstacle factor
 * */
double cost_obstacle(const VectorXd& pose, 
                    const gpmp2::ObstaclePlanarSDFFactorPointRobot& obs_factor){
    
    VectorXd vec_err = obs_factor.evaluateError(pose);

    MatrixXd precision_obs{MatrixXd::Identity(vec_err.rows(), vec_err.rows())};
    precision_obs = precision_obs / obs_factor.get_noiseModel()->sigmas()[0];

    double cost = vec_err.transpose().eval() * precision_obs * vec_err;
   
    return cost;

}

/// =========================== Definitions ============================
vimp::PlanarPointRobotSDFMultiObsExample planar_pr_sdf;
gpmp2::PointRobotModel pRModel = std::move(planar_pr_sdf.pRmodel());
gpmp2::PlanarSDF sdf = std::move(planar_pr_sdf.sdf());
double cost_sigma = 0.5, epsilon = 4.0;
gpmp2::ObstaclePlanarSDFFactorPointRobot collision_k{0, pRModel, sdf, cost_sigma, epsilon};
VectorXd conf = (VectorXd(2) << 11.3321059864421, 9.16117246531728).finished();
MatrixXd field = planar_pr_sdf.field();


/// ============================ TEST ==============================
TEST(GPMP2_OBS, signed_dist){
    ASSERT_LE(abs(field(99, 100) - 5.635601043701172), 1e-10);
    ASSERT_LE(abs(field(299, 399) - 8.655633926391602), 1e-10);

    double signed_dist_expected = -1.932105970254905;
    double signed_dist = sdf.getSignedDistance(conf);
    ASSERT_LE(abs(signed_dist - signed_dist_expected), 1e-10);
}


typedef std::function<MatrixXd(const VectorXd&)> GHFunction;
TEST(GPMP2_OBS, inegration){

    MatrixXd err_expected = MatrixXd::Constant(1,1, 6.932105970254905); 
    MatrixXd err_computed = collision_k.evaluateError(conf);

    ASSERT_LE((err_computed - err_expected).norm(), 1e-10);

    int deg = 10;
    int dim = 2;

    VectorXd mean{(VectorXd(2)<< 11.6236181383659, 10.0231357029832).finished()};
    MatrixXd cov{(MatrixXd(2,2)<<-1.19797500526647, 0.152222124094295, 
                                0.152222124094295, 0.432860865646265).finished()};
    MatrixXd prec_ground_truth{(MatrixXd(2,2)<<22.8474978561102, 0.222697015421695, 
                                                0.222697015421695,25.9506303517537).finished()};
    MatrixXd precision = cov.inverse();     


    // Gauss Hermite
    GaussHermite<GHFunction> gh{deg, dim, mean, cov, [&](const VectorXd& x){
        return MatrixXd::Constant(1,1,cost_obstacle(x, collision_k));}};

    MatrixXd inte1 = gh.Integrate();
    cout << "inte1 " << endl << inte1 << endl;

    gh.update_integrand([&](const VectorXd& x){
        return MatrixXd{(x-mean)*cost_obstacle(x, collision_k)};}
    );
    MatrixXd inte2 = gh.Integrate();
    cout << "inte2 " << endl << inte2 << endl;

    gh.update_integrand([&](const VectorXd& x){
        return MatrixXd{(x-mean)*(x-mean).transpose()*cost_obstacle(x, collision_k)};}
    );
    MatrixXd inte3 = gh.Integrate();
    cout << "inte3 " << endl << inte3 << endl;

    MatrixXd d_precision = precision * inte3 * precision - precision * inte1(0, 0) - precision;
    cout << "d_precision " << endl << d_precision << endl;

    /// scale 3 times
    gh.update_integrand([&](const VectorXd& x){
        return MatrixXd::Constant(1,1,3.0*cost_obstacle(x, collision_k));});
    MatrixXd inte1_scale = inte1 * 3.0;
    ASSERT_LE((gh.Integrate() - inte1_scale).norm(), 1e-10);

    gh.update_integrand([&](const VectorXd& x){
        return MatrixXd{(x-mean)*3.0*cost_obstacle(x, collision_k)};});
    MatrixXd inte2_scale = inte2 * 3.0;
    ASSERT_LE((gh.Integrate() - inte2_scale).norm(), 1e-10);

    gh.update_integrand([&](const VectorXd& x){
        return MatrixXd{(x-mean)*(x-mean).transpose()*3.0*cost_obstacle(x, collision_k)};});
    MatrixXd inte3_scale = inte3 * 3.0;
    ASSERT_LE((gh.Integrate() - inte3_scale).norm(), 1e-10);

}