#include "mex.h"
#include <Eigen/Dense>
#include "SignedDistanceField.h"
#include <exception>
#include <iostream>

// Helper function to get pointer to C++ object
SignedDistanceField* getObjectPointer(const mxArray* obj) {
    if (!mxIsUint64(obj) || mxGetNumberOfElements(obj) != 1) {
        mexErrMsgTxt("Invalid object handle.");
    }
    return reinterpret_cast<SignedDistanceField*>(*reinterpret_cast<uint64_t*>(mxGetData(obj)));
}

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {

    char* command = mxArrayToString(prhs[0]);

    if (!mxIsChar(prhs[0])) {
        mexErrMsgTxt("Error: The first argument must be a string.");
    }

    if (strcmp(command, "new") == 0) {
        double* origin_data = mxGetPr(prhs[1]);
        Eigen::Vector3d origin(origin_data[0], origin_data[1], origin_data[2]);

        double cell_size = mxGetScalar(prhs[2]);
        int field_rows = static_cast<int>(mxGetScalar(prhs[3]));
        int field_cols = static_cast<int>(mxGetScalar(prhs[4]));
        int field_z = static_cast<int>(mxGetScalar(prhs[5]));
        std::cout << "cell size = " << cell_size << ", field_rows = " << field_rows << ", field_cols = " << field_cols << ", field_z = " << field_z << std::endl;

        SignedDistanceField* sdf = new SignedDistanceField(origin, cell_size, field_rows, field_cols, field_z);

        plhs[0] = mxCreateNumericMatrix(1, 1, mxUINT64_CLASS, mxREAL);
        *((uint64_t*)mxGetData(plhs[0])) = reinterpret_cast<uint64_t>(sdf);
    } else if(strcmp(command, "initFieldData") == 0){
        SignedDistanceField* sdf = getObjectPointer(prhs[1]);
        int z_idx = static_cast<int>(mxGetScalar(prhs[2]));

        // Parse field_layer
        mwSize rows = mxGetM(prhs[3]);
        mwSize cols = mxGetN(prhs[3]);
        double* data_ptr = mxGetPr(prhs[3]);
        Eigen::MatrixXd field_layer(rows, cols);
        for (mwSize r = 0; r < rows; ++r) {
            for (mwSize c = 0; c < cols; ++c) {
                field_layer(r, c) = data_ptr[c + r * cols];
            }
        }

        sdf->initFieldData(z_idx, field_layer);

    } 
    else if (strcmp(command, "save") == 0) {
        
        uint64_t ptr = *((uint64_t*)mxGetData(prhs[1]));
        SignedDistanceField* sdf = reinterpret_cast<SignedDistanceField*>(ptr);

        if (!mxIsChar(prhs[2])) {
            mexErrMsgTxt("Error: Filename must be a char string.");
        }

        char* filename = mxArrayToString(prhs[2]);
        try {
            sdf->saveSDF(std::string(filename));
        } catch (const std::exception& e) {
            mexErrMsgTxt(e.what());
        }

        mxFree(filename);
    }
    else {
        mexErrMsgTxt("Usage: sdf = SignedDistanceField_mex(origin, cell_size, field_rows, field_cols, field_z)111");
    }

    
}