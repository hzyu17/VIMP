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

#include "../../gtsam/base/Vector.h"
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include "../../gpmp2/kinematics/PointRobotModel.h"
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

    /**
     * @brief Test the use of gpmp2 class.
     */
    class CyTest2{  
        public:
            CyTest2(){};
            CyTest2(const gpmp2::PointRobotModel& pR_model):_pR_model{boost::make_shared<gpmp2::PointRobotModel>(pR_model)}{}

            boost::shared_ptr<gpmp2::PointRobotModel> _pR_model;

            virtual ~CyTest2(){};

    };

    /**
     * @brief Test the functional, and make_shared
     */
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


    /**
     * @brief Test the std::vector
     */
    class CyTest4{
        public:
            CyTest4(){};
            CyTest4(const std::vector<double>& vec_d):_vd{vec_d}{}

            std::vector<double> _vd;

            void print_vec(){
                for (auto & i_d : _vd){
                    std::cout << i_d << std::endl;
                }
            }
    };
}