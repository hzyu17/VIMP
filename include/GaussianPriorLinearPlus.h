//
// Created by hongzhe on 4/3/22.
//

#include <gpmp2/gp/GaussianProcessPriorLinear.h>

using namespace gtsam;
using namespace gpmp2;

namespace MPVI{
    class GaussianPriorLinearPlus: public GaussianProcessPriorLinear{

    public:
        GaussianPriorLinearPlus(gtsam::Key poseKey1, gtsam::Key velKey1,
                                gtsam::Key poseKey2, gtsam::Key velKey2,
                                double delta_t,
                                const gtsam::SharedNoiseModel Qc_model):
                                GaussianProcessPriorLinear( poseKey1,
                                                            velKey1,
                                                            poseKey2,
                                                            velKey2,
                                                            delta_t,
                                                            Qc_model){
            delta_t_ = delta_t;
            Qc_ = getQc(Qc_model);

            cout << "Qc" << endl << get_Qc() <<endl;

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
}
