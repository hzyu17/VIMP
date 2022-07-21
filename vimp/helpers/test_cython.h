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
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorPointRobot.h>


namespace vimp{
    class CythonTest{
        public:
            CythonTest(){};
            CythonTest(const gtsam::Vector& vec, const gpmp2::ObstaclePlanarSDFFactorPointRobot& obs_fact):_vec{vec}, 
            _func{[obs_fact](const gpmp2::ObstaclePlanarSDFFactorPointRobot& obs_fact){return 0;}}{};

            // copy constructors
            // CythonTest(const CythonTest& other):_vec{other._vec}, _obs{other._obs}{};

            gtsam::Vector _vec;
            
            std::function<int(const gpmp2::ObstaclePlanarSDFFactorPointRobot&)> _func;
            
            gtsam::Vector vec() const{
                return _vec;
            }

            // gpmp2::ObstaclePlanarSDFFactorPointRobot obs() const {
            //     return _obs;
            // }

    };
}