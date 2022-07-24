/**
 * @file test_cython.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Simplest file to test cython functionality.
 * @version 0.1
 * @date 2022-07-20
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <gtsam/base/Vector.h>
// #include <gpmp2/obstacle/ObstaclePlanarSDFFactorPointRobot.h>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
// #include <vimp/instances/PriorColPlanarPointRobot.h>
#include <gpmp2/kinematics/PointRobotModel.h>
#include <boost/function.hpp>


namespace vimp{
    class CythonTest{
        public:
            CythonTest(){};
            CythonTest(const gtsam::Vector& vec, 
                       const UnaryFactorTranslation2D& prior):_vec{vec}, _prior{prior},
            _func{[this](const MatrixXd& x){return MatrixXd{x - _prior.get_Qc()};}}{};
            
            gtsam::Vector _vec;
            UnaryFactorTranslation2D _prior;
            
            std::function<MatrixXd(const MatrixXd& x)> _func;
            
            gtsam::Vector vec() const{
                return _vec;
            }

            MatrixXd f(const MatrixXd& x){
                return _func(x);
            }

    };

    // class CythonTest{
    //     public:
    //         CythonTest(){};
            
    //         CythonTest(const gtsam::Vector& vec, const gpmp2::ObstaclePlanarSDFFactorPointRobot& obs_fact):_vec{vec}, 
    //         _func{[](const gpmp2::ObstaclePlanarSDFFactorPointRobot& obs_fact){return 0;}}{};
            
    //         gtsam::Vector _vec;
            
    //         std::function<int(const gpmp2::ObstaclePlanarSDFFactorPointRobot&)> _func;
            
    //         gtsam::Vector vec() const{
    //             return _vec;
    //         }


    // };

    class CyTest2{
        public:
            CyTest2(){};
            CyTest2(const gpmp2::PointRobotModel& pR_model):_pR_model{boost::make_shared<gpmp2::PointRobotModel>(pR_model)}{}

            boost::shared_ptr<gpmp2::PointRobotModel> _pR_model;

            virtual ~CyTest2(){};

    };

    typedef boost::function<double(const double&)> FunctionDoubleDouble;

    class CyTest3{
        public:
            CyTest3(){};
            CyTest3(const FunctionDoubleDouble& f):_f{boost::make_shared<FunctionDoubleDouble>(f)}{}

            boost::shared_ptr<FunctionDoubleDouble> _f;

            double f(const double& x){
                return (*_f)(x);
            }
    };
}