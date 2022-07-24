#include <vimp/instances/GaussHermiteInstance.h>
#include <gtest/gtest.h>

using namespace Eigen;
using namespace std;

MatrixXd func(const VectorXd& x){
    return MatrixXd{x * x.transpose()};
}

TEST(GHInstances, creation){
    
    VectorXd mean{VectorXd::Ones(2)};
    MatrixXd cov{MatrixXd::Identity(2, 2)};
    vimp::GaussHermiteInstance gh_instance{5, 2, mean, cov, func};

    cout << "gh_instance.f(mean) " << endl << gh_instance.f(mean) << endl;
    MatrixXd expected_output = (MatrixXd(2, 2)<< 1,1,1,1).finished();
    ASSERT_EQ((gh_instance.f(mean) - expected_output).norm(), 0);
}