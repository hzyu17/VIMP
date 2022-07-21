#include <vimp/helpers/test_cython.h>
#include <gtest/gtest.h>
#include <gtsam/inference/Symbol.h>
#include "../instances/PlanarPointRobotSDFExample.h"
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorPointRobot.h>

using namespace Eigen;
using namespace vimp;
using namespace gpmp2;

TEST(TEST_CYTHON, construction){
    VectorXd vec{VectorXd::Zero(2)};

    // An example pr and sdf
    PlanarPointRobotSDFExample planar_pr_sdf;
    PointRobotModel pRModel = std::move(planar_pr_sdf.pRmodel());
    PlanarSDF sdf = std::move(planar_pr_sdf.sdf());

    /// Obs factor
    double cost_sigma = 1.0, epsilon = 1.0;

    ObstaclePlanarSDFFactorPointRobot obs{gtsam::symbol('x', 0), pRModel, sdf, cost_sigma, epsilon};

    CythonTest cython_obj{vec, obs};

    ASSERT_EQ((cython_obj.vec()-vec).norm(), 0);

}