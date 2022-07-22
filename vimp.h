// gpmp2 matlab wrapper declarations

/* *   Forward declarations and class definitions for Cython:
 *     - Need to specify the base class (both this forward class and base 
          class are declared in an external cython header)
 *       This is so Cython can generate proper inheritance.
 *       Example when wrapping a gtsam-based project:
 *          // forward declarations
 *          virtual class gtsam::NonlinearFactor
 *          virtual class gtsam::NoiseModelFactor : gtsam::NonlinearFactor
 *          // class definition
 *          #include <MyFactor.h>
 *          virtual class MyFactor : gtsam::NoiseModelFactor {...};
 *    - *DO NOT* re-define overriden function already declared in the external (forward-declared) base class
 *        - This will cause an ambiguity problem in Cython pxd header file*/

// gtsam deceleration
// class gtsam::Vector6;
// class gtsam::Vector3;
// class gtsam::Point3;
// class gtsam::Pose3;
// class gtsam::Point2;
// class gtsam::Pose2;

// class gtsam::GaussianFactorGraph;
// class gtsam::Values;
// virtual class gtsam::noiseModel::Base;
// virtual class gtsam::NonlinearFactor;
// virtual class gtsam::NonlinearFactorGraph;
// virtual class gtsam::NoiseModelFactor : gtsam::NonlinearFactor;

class gpmp2::PointRobotModel;
// virtual class gpmp2::ObstaclePlanarSDFFactorPointRobot;


// namespace gpmp2{

// // Abstract Point Robot class
// #include <gpmp2/kinematics/PointRobot.h>

// class PointRobot {
//   PointRobot(size_t dof, size_t nr_links);
//   // full forward kinematics
//   Matrix forwardKinematicsPose(Vector jp) const;
//   Matrix forwardKinematicsPosition(Vector jp) const;
//   Matrix forwardKinematicsVel(Vector jp, Vector jv) const;
//   // accesses
//   size_t dof() const;
//   size_t nr_links() const;
// };

// // BodySphere class
// #include <gpmp2/kinematics/RobotModel.h>

// class BodySphere {
//   BodySphere(size_t id, double r, const gtsam::Point3& c);
// };

// class BodySphereVector {
//   BodySphereVector();
//   void push_back(const gpmp2::BodySphere& sphere);
// };

// // Point Robot Model class
// #include <gpmp2/kinematics/PointRobotModel.h>

// class PointRobotModel {
//   PointRobotModel(const gpmp2::PointRobot& pR, const gpmp2::BodySphereVector& spheres);
//   // solve sphere center position in world frame
//   Matrix sphereCentersMat(Vector conf) const;
//   // accesses
//   size_t dof() const;
//   gpmp2::PointRobot fk_model() const;
//   size_t nr_body_spheres() const;
//   double sphere_radius(size_t i) const;
// };

// // planar obstacle avoid factor for Point Robot
// #include <gpmp2/obstacle/ObstaclePlanarSDFFactorPointRobot.h>

// virtual class ObstaclePlanarSDFFactorPointRobot : gtsam::NoiseModelFactor {
//   ObstaclePlanarSDFFactorPointRobot(
//       size_t posekey, const gpmp2::PointRobotModel& pR,
//       const gpmp2::PlanarSDF& sdf, double cost_sigma, double epsilon);
//   Vector evaluateError(Vector pose) const;
// };

// //  signed distance field class
// #include <gpmp2/obstacle/PlanarSDF.h>
// class PlanarSDF {
//   PlanarSDF(const gtsam::Point2& origin, double cell_size, const Matrix& data);
//   // access
//   double getSignedDistance(const gtsam::Point2& point) const;
//   void print(string s) const;
// };
// }

namespace vimp{

#include <vimp/instances/PriorColPlanarPointRobot.h>
class UnaryFactorTranslation2D{
    UnaryFactorTranslation2D();
    UnaryFactorTranslation2D(size_t key, const Vector& conf, const gtsam::noiseModel::Base* model);
    
    Matrix get_Qc() const;

};

#include <vimp/helpers/test_cython.h>
class CythonTest{
    CythonTest();
    // CythonTest(const Vector& vec);
    CythonTest(const Vector& vec, const UnaryFactorTranslation2D& prior);
    // CythonTest(const gtsam::Vector& vec, const gpmp2::ObstaclePlanarSDFFactorPointRobot& obs_fact);

    Vector vec() const;
    
    Matrix f(const Matrix& x);
    // gpmp2::ObstaclePlanarSDFFactorPointRobot obs() const;
};

virtual class CyTest2{
    CyTest2();
    CyTest2(const gpmp2::PointRobotModel& pR_model);
    
};

// #include <vimp/instances/PriorColPlanarPR.h>
// class OptFactPriColPlanarPRGH{
//     OptFactPriColPlanarPRGH();
//     OptFactPriColPlanarPRGH(const int& dimension, 
//                             const std::function<double(const Vector&, 
//                                             const vimp::UnaryFactorTranslation2D&, 
//                                             const gpmp2::ObstaclePlanarSDFFactorPointRobot&)>& function_, 
//                             const vimp::UnaryFactorTranslation2D& cost_class_, 
//                             const gpmp2::ObstaclePlanarSDFFactorPointRobot& cost_class1_, 
//                             const Matrix& Pk_);

// };

}