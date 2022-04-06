//
// Created by hongzhe on 4/3/22.
//

#include <gpmp2/gp/GaussianProcessPriorPose2.h>

class GaussianPriorPose2Plus : public GaussianProcessPriorPose2{
public:
    GaussianPriorPose2Plus(gtsam::Key poseKey1, gtsam::Key velKey1,
                           gtsam::Key poseKey2, gtsam::Key velKey2,
                           double delta_t, const gtsam::SharedNoiseModel& Qc_model):GaussianProcessPriorPose2(
                                   poseKey1,
                                   poseKey2,
                                   velKey1,
                                   velKey2,
                                   delta_t,
                                   Qc_model){
        delta_t_ = delta_t;
        Qc_ = getQc(Qc_model);

    }

public:
    double get_delta_t() const{
        return delta_t_;
    }

    MatrixXd get_Qc() const{
        return Qc_;
    }

private:
    double delta_t_;
    MatrixXd Qc_;

};

