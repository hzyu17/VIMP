#include <vimp/instances/GaussHermiteInstance.h>
#include <gtest/gtest.h>

using namespace Eigen;

TEST(GHInstances, creation){
    MatrixXd function(const VectorXd& x){
        return MatrixXd{x * x.transpose()};
    }

    VectorXd mean{VectorXd::Ones(2)};
    MatrixXd cov{MatrixXd::Identity(2, 2)};
    vimp::GaussHermiteInstance gh_instance{5, 2, mean, cov, function};

    ASSERT_EQ((gh_instance.f(mean) - cov).norm(), 0);
}