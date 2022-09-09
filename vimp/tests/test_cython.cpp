#include "../helpers/test_cython.h"
#include <gtest/gtest.h>
#include <gtsam/inference/Symbol.h>
#include "../robots/PlanarPointRobotSDFExample.h"
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorPointRobot.h>

using namespace Eigen;
using namespace vimp;
using namespace gpmp2;

TEST(TEST_CYTHON, construction){
    VectorXd vec{VectorXd::Ones(2)};

    // An example pr and sdf
    PlanarPointRobotSDFExample planar_pr_sdf;
    PointRobotModel pRModel = std::move(planar_pr_sdf.pRmodel());
    PlanarSDF sdf = std::move(planar_pr_sdf.sdf());

    /// Obs factor
    double cost_sigma = 1.0, epsilon = 1.0;

    boost::shared_ptr<ObstaclePlanarSDFFactorPointRobot> p_obs{new ObstaclePlanarSDFFactorPointRobot{gtsam::symbol('x', 0), pRModel, sdf, cost_sigma, epsilon}};
    ObstaclePlanarSDFFactorPointRobot obs{gtsam::symbol('x', 0), pRModel, sdf, cost_sigma, epsilon};
    
    CythonTest cython_obj{vec, obs};

    CyTest2 cytest_2{pRModel};

    // SharedNoiseModel K_0 = noiseModel::Isotropic::Sigma(2, 0.5);
    // UnaryFactorTranslation2D prior_k{gtsam::symbol('x', 0), Vector2d{VectorXd::Zero(2)}, K_0};

    // CythonTest cython_obj{vec, prior_k};

    // MatrixXd input{MatrixXd::Random(2,2)};
    // MatrixXd output{cython_obj.f(input)};
    // MatrixXd output_expected{input - prior_k.get_Qc()};
    // std::cout << "f output" << std::endl << output << std::endl;
    // ASSERT_EQ((output - output_expected).norm(), 0);
    
    ASSERT_EQ((cython_obj.vec()-vec).norm(), 0);

}