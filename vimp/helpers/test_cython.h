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


namespace vimp{
    class CythonTest{
        public:
            CythonTest(){};
            CythonTest(const gtsam::Vector& vec):_vec{vec}{};

            // copy constructors
            CythonTest(const CythonTest& other):_vec{other._vec}{};

            gtsam::Vector _vec;
            
            gtsam::Vector vec(){
                return _vec;
            }

    };
}