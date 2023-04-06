/**
 * @file PointRobotSDF3D_pgcs.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief An example experiment settings of a point robot in 3D multi obstacle env. 
 * Imported from the tested code in gpmp2.
 * @version 0.1
 * @date 2022-07-27
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <gtsam/inference/Symbol.h>
#include "../helpers/hinge3Dhelper.h"
#include "../helpers/data_io.h"
#include "../helpers/eigen_wrapper.h"

using namespace Eigen;

using SDF = gpmp2::SignedDistanceField;

namespace vimp{

class PointRobotSDFPGCS{
    public:
        PointRobotSDFPGCS(double epsilon): _eps(epsilon){
            default_sdf();
        }

        PointRobotSDFPGCS(double epsilon, double r): _eps(epsilon), _r(r){
            // default sdf
            default_sdf();
        }

        void default_sdf(){
            SDF sdf = SDF();
            sdf.loadSDF("/home/hongzhe/git/VIMP/matlab_helpers/PGCS-examples/3dSDFs/pRSDF3D.bin");
            _psdf = std::make_shared<SDF>(sdf);
        }

        /**
         * Obstacle factor: case, returns the Vector of h(x) and the Jacobian matrix.
         * */
        std::tuple<VectorXd, MatrixXd> hinge_jac(const VectorXd& pose){
            return hinge_gradient_point(pose, *_psdf, _eps);
        }

        inline void update_sdf(const SDF& sdf){
            _psdf = std::make_shared<SDF>(sdf);  
        }

        inline SDF sdf() const { return *_psdf; }

        public:
            EigenWrapper _ei;
            std::shared_ptr<SDF> _psdf;    

            double _eps, _r;

    };
} // namespace vimp