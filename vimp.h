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
class gtsam::Vector3;
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

namespace vimp{

#include <vimp/helpers/test_cython.h>

class CythonTest{
    CythonTest();
    CythonTest(const Vector& vec);
    Vector vec();
};

}