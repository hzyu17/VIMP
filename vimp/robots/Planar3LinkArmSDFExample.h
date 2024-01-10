/**
 * @file PlanarArmSDFExample.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief A commonly used planar 3-link arm model with sdf. 
 * Imported from the tested code in gpmp2.
 * @version 0.1
 * @date 2022-08-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <gpmp2/kinematics/ArmModel.h>
#include <gpmp2/obstacle/PlanarSDF.h>

using namespace gpmp2;
using namespace Eigen;

namespace vimp{

    class Planar3LinkArmSDFExample{
        public:
            Planar3LinkArmSDFExample(const string& field_file){
                
                gtsam::Point2 origin(-1, -1);
                double cell_size = 0.01;

                _field = _matrix_io.load_csv(field_file);

                _sdf = PlanarSDF(origin, cell_size, _field);

                // 2 link simple example
                gtsam::Pose3 arm_base(gtsam::Rot3(), gtsam::Point3(0.0, 0.0, 0.0));
                Vector3d a(0.5, 0.5, 0.5), alpha(0, 0, 0), d(0, 0, 0);
                Arm abs_arm(3, a, alpha, d, arm_base);

                // body info, three spheres
                BodySphereVector body_spheres;
                const double r = 0.01;

                body_spheres.push_back(BodySphere(0, r, Point3(-0.5,  0.0,  0.0)));
                body_spheres.push_back(BodySphere(0, r, Point3(-0.4,  0.0,  0.0)));
                body_spheres.push_back(BodySphere(0, r, Point3(-0.3,  0.0,  0.0)));
                body_spheres.push_back(BodySphere(0, r, Point3(-0.2,  0.0,  0.0)));
                body_spheres.push_back(BodySphere(0, r, Point3(-0.1,  0.0,  0.0)));

                body_spheres.push_back(BodySphere(1, r, Point3(-0.5, 0.0,  0.0)));
                body_spheres.push_back(BodySphere(1, r, Point3(-0.4, 0.0,  0.0)));
                body_spheres.push_back(BodySphere(1, r, Point3(-0.3, 0.0,  0.0)));
                body_spheres.push_back(BodySphere(1, r, Point3(-0.2, 0.0,  0.0)));
                body_spheres.push_back(BodySphere(1, r, Point3(-0.1, 0.0,  0.0)));

                body_spheres.push_back(BodySphere(2, r, Point3(-0.5, 0.0,  0.0)));
                body_spheres.push_back(BodySphere(2, r, Point3(-0.4, 0.0,  0.0)));
                body_spheres.push_back(BodySphere(2, r, Point3(-0.3, 0.0,  0.0)));
                body_spheres.push_back(BodySphere(2, r, Point3(-0.2, 0.0,  0.0)));
                body_spheres.push_back(BodySphere(2, r, Point3(-0.1, 0.0,  0.0)));
                body_spheres.push_back(BodySphere(2, r, Point3(-0.0, 0.0,  0.0)));

                _arm = ArmModel{abs_arm, body_spheres};

            }

            ArmModel arm_model(){
                return _arm;
            }

            PlanarSDF sdf() const { return _sdf; }
            int ndof() const {return _ndof;}
            int nlinks() const {return _nlinks;}
            MatrixXd field() const {return _field;}

        private:
            ArmModel _arm;
            PlanarSDF _sdf;
            MatrixXd _field;
            gvi::MatrixIO _matrix_io;

            /// Arm robot
            int _ndof = 1;
            int _nlinks = 3;
    };

}// namespace