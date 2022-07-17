/**
 * @file GaussianPriorLinearGetQc.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief 
 * @version 0.1
 * @date 2022-04-03
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <gpmp2/gp/GaussianProcessPriorLinear.h>

using namespace gtsam;
using namespace gpmp2;

namespace MPVI{
    class GaussianPriorLinearGetQc: public gpmp2::GaussianProcessPriorLinear{

    public:
        GaussianPriorLinearGetQc(gtsam::Key poseKey1, 
                                 gtsam::Key velKey1, 
                                 gtsam::Key poseKey2, 
                                 gtsam::Key velKey2,
                                 double delta_t,
                                 const gtsam::SharedNoiseModel Qc_model):
                            delta_t_{delta_t},
                            Qc_{getQc(Qc_model)}
        {
            gpmp2::GaussianProcessPriorLinear( poseKey1, velKey1, poseKey2, velKey2, delta_t, Qc_model);
            cout << "Qc" << endl << get_Qc() <<endl;
        }
    public:
        inline double get_delta_t() const{
            return delta_t_;
        }

        inline Eigen::MatrixXd get_Qc() const{
            return Qc_;
        }

    private:
        double delta_t_;
        Eigen::MatrixXd Qc_;
    };
}
