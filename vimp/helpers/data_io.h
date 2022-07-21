/**
 * @file data_io.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Helpers related to the data io stream
 * @version 0.1
 * @date 2022-07-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "repeated_includes.h"

namespace vimp{
    class MatrixIO{
    public:
        
        MatrixIO(){}
        
        void saveData(const string& fileName, const gtsam::Matrix& matrix){
            
            ofstream file(fileName);
            if (file.is_open()){
                file << matrix.format(CSVFormat);
                file.close();
            }
        }
        
    };

}

