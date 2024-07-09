/**
 * @file repeated_includes.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief 
 * @version 0.1
 * @date 2022-07-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once
#ifndef REPEATED_INCLUDES
#define REPEATED_INCLUDES

#include <Eigen/Dense>
#include <iostream>
#include <fstream>

// using namespace std;
// using namespace Eigen;

//https://eigen.tuxfamily.org/dox/structEigen_1_1IOFormat.html
Eigen::IOFormat CSVFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", "\n");

#endif /* REPEATED_INCLUDES */