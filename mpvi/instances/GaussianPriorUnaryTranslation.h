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

#include <gtsam/slam/PoseTranslationPrior.h>
#include <gpmp2/gp/GPutils.h>

namespace vimp{
template <class T>
/// A derived class of NoiseModelFactor1 in gtsam, used for the prior err probability for translation. 
class UnaryFactorTranslation: public gtsam::NoiseModelFactor1<T> {
public:
    /// @param Key The key for the factor
    /// @param conf The state variable
    /// @param model The Gaussian noise model
    UnaryFactorTranslation(gtsam::Key key, const T& conf, const gtsam::SharedNoiseModel model):
            gtsam::NoiseModelFactor1<T>(model, key), conf_(conf), K_(gpmp2::getQc(model)) {}

    Eigen::VectorXd evaluateError(const T& q, boost::optional<Eigen::MatrixXd &> H=boost::none) const
    {
        return Eigen::VectorXd{q-conf_};
    }

    /// Returns the covariance matrix of the noise model.
    Eigen::MatrixXd get_Qc() const{
        return K_;
    }

private:
    T conf_; 
    Eigen::MatrixXd K_;
};

}

