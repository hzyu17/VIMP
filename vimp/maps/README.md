# SignedDistanceField MATLAB Mex Interface

This folder provides a C++ implementation of a 3D signed distance field (`SignedDistanceField`) and a corresponding MATLAB Mex interface (`SignedDistanceField_mex`).

## Contents

- **`SignedDistanceField.h`**  
  Defines the `SignedDistanceField` class, which stores and queries a 3D signed distance field using Eigen matrices.

- **`SignedDistanceField_mex.cpp`**  
  Implements the MATLAB Mex interface for creating, initializing, and saving the signed distance field.

- **`SignedDistanceField_mex.mexa64`** (Generated after compilation)  
  The compiled Mex file that can be directly called from MATLAB.

- **Makefile**  
  The build script to compile `SignedDistanceField_mex.cpp` into a Mex file on Linux.

- **Example MATLAB script**  
  A script demonstrating how to:
  1. Generate a 3D dataset.
  2. Calculate the signed distance field.
  3. Initialize the data in the `SignedDistanceField` object via Mex.
  4. Save the resulting SDF to a `.bin` file.

## Requirements

- **MATLAB** (e.g., R2024a).  
  Ensure you have configured your MATLAB compiler with `mex -setup` and set the correct environment variables.
- **C++17 compiler** (e.g., `g++ 7.3+`).  
  The Makefile uses the `-std=c++17` flag.
- **Eigen** (e.g., version 3.3+).  
  By default, Eigen is assumed to be located in `/usr/include/eigen3`.
- **Cereal** (header-only library) for serialization.  
  The Makefile assumes the cereal headers are located in `../GaussianVI/include/cereal/include`. Adjust the path if necessary.

## How to Build

1. **Compile the Mex file**  
   Open a terminal in the directory containing the Makefile, then run:
   ```bash
   make clean
   make
   ```
   This will generate the SignedDistanceField_mex.mexa64 file (for Linux). On success, you should see the new file in the directory.

2. **Add the Mex file to your MATLAB path**  
   If the `.mexa64` file is not in your current MATLAB working directory, you can do:
   ```matlab
   addpath('/path/to/where/your/mexa64/file/is')
   ```


