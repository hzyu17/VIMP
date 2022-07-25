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
class gtsam::Vector2;

class gtsam::GaussianFactorGraph;
class gtsam::Values;
virtual class gtsam::noiseModel::Base;
virtual class gtsam::NonlinearFactor;
virtual class gtsam::NonlinearFactorGraph;
virtual class gtsam::NoiseModelFactor : gtsam::NonlinearFactor;

// gpmp2 deceleration
class gpmp2::PointRobotModel;
virtual class ObstaclePlanarSDFFactorPointRobot : gtsam::NoiseModelFactor;
class gpmp2::PlanarSDF;


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

class FunctionDoubleDouble{
    FunctionDoubleDouble();
};

virtual class CyTest2{
    CyTest2();
    CyTest2(const gpmp2::PointRobotModel& pR_model);
    
};

class CyTest3{
    CyTest3();
    CyTest3(const vimp::FunctionDoubleDouble& f);
    
    double f(const double& x);
};

class CyTest4{
    CyTest4();
    CyTest4(const std::vector<double>& vec_d);

    void print_vec();
};


// #include <vimp/instances/PriorColPlanarPointRobot.h>

// class OptFactPriColGHInstancePointRobot{
//     OptFactPriColGHInstancePointRobot();
//     OptFactPriColGHInstancePointRobot(const int& dimension, 
//                                       const vimp::UnaryFactorTranslation2D& prior,
//                                       const gpmp2::ObstaclePlanarSDFFactorPointRobot& collision,
//                                       const Matrix& Pk);
// };

// class VIMPOptimizerPriColPR{
//     VIMPOptimizerPriColPR();
//     VIMPOptimizerPriColPR(const vector<std::shared_ptr<OptFactPriColGHInstancePointRobot>>& vec_fact_optimizers, int niters);

//     void optimize();
// };

// #include <vimp/instances/GaussHermiteInstance.h>

// class FunctionMatrixVector{

//     FunctionMatrixVector();
// };

// class GaussHermiteInstance{
//     GaussHermiteInstance();
//     GaussHermiteInstance(const int& deg, const int& dim, const Vector& mean, const Matrix& P, const vimp::FunctionMatrixVector& func);

//     void update_integrand(const vimp::FunctionMatrixVector& fun);
//     Matrix f(const Vector& x);
//     Matrix Integrate();
// };

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