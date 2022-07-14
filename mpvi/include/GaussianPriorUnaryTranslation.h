/**
 * @file GaussianPriorUnaryTranslation.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief 
 * @version 0.1
 * @date 2022-04-23
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PoseTranslationPrior.h>
#include <gpmp2/gp/GPutils.h>

using namespace gtsam;
using namespace gpmp2;
using namespace Eigen;
using namespace MPVI;

template <class T>
/// A derived class of NoiseModelFactor1 in gtsam, used for the prior err probability for translation. 
class UnaryFactorTranslation: public NoiseModelFactor1<T> {
public:
/// @param Key The key for the factor
/// @param conf The state variable
/// @param model The Gaussian noise model
    UnaryFactorTranslation(Key key, const T& conf, const gtsam::SharedNoiseModel model):
            NoiseModelFactor1<T>(model, key), conf_(conf), K_(getQc(model)) {}

    VectorXd evaluateError(const T& q, boost::optional<gtsam::Matrix &> H=boost::none) const
    {
        cout << "evaluateError inside" << endl << VectorXd{q-conf_} << endl;
        return VectorXd{q-conf_};
    }

    /// Returns the covariance matrix of the noise model.
    Eigen::MatrixXd get_Qc() const{
        cout << "get_Qc" << endl;
        return K_;
    }

private:
    T conf_; ///< X and Y measurements
    Eigen::MatrixXd K_;
};

