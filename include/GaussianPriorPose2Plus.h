//
// Created by hongzhe on 4/3/22.
//
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PoseTranslationPrior.h>
#include <gpmp2/gp/GPutils.h>

using namespace gtsam;
using namespace gpmp2;

namespace MPVI{
    class UnaryFactorVector2: public NoiseModelFactor1<gtsam::Vector2> {
    public:
        UnaryFactorVector2(Key j, const gtsam::Vector2& conf, const SharedNoiseModel& model):
                NoiseModelFactor1<gtsam::Vector2>(model, j), mx_(conf(0)), my_(conf(1)) {
            Qc_ = getQc(model);
        }

        gtsam::Vector evaluateError(const gtsam::Vector2& q, boost::optional<gtsam::Matrix &> H=boost::none) const
        {
            return (gtsam::Vector(2) << q(0) - mx_, q(1) - my_).finished();
        }

        MatrixXd get_Qc() const{
            return Qc_;
        }

    private:
        MatrixXd Qc_;
        double mx_, my_; ///< X and Y measurements
    };
}


