//
// Created by hongzhe on 4/3/22.
//
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PoseTranslationPrior.h>
#include <gpmp2/gp/GPutils.h>

using namespace gtsam;
using namespace gpmp2;

namespace MPVI{
    template <class T>
    class UnaryFactorTranslation: public NoiseModelFactor1<T> {
    public:
        UnaryFactorTranslation(Key key, const T& conf, const SharedNoiseModel& model):
                NoiseModelFactor1<T>(model, key), conf_(conf) {
            Qc_ = getQc(model);
        }

        gtsam::Vector evaluateError(const T& q, boost::optional<gtsam::Matrix &> H=boost::none) const
        {
            return T{q-conf_};
        }

        MatrixXd get_Qc() const{
            return Qc_;
        }

    private:
        MatrixXd Qc_;
        T conf_; ///< X and Y measurements
    };
}


