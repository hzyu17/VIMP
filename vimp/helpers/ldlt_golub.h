/**
 * @file ldlt_golub.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief LDLT decomposition from Golub 1996: Matrix Computations, 3rd ed, Algorithm 4.1.2
 * @version 0.1
 * @date 2023-01-10
 * 
 * @copyright Copyright (c) 2023
 * 
 */

// #include <Eigen/Dense>

// void LDLT_Golub(const Eigen::MatrixXd& A, Eigen::MatrixXd L, Eigen::MatrixXd D){
//     int n = A.rows();
//     Eigen::VectorXd v = Eigen::VectorXd::Zero(n);

//     for (int j=0; j<n; j++){
//         for (int i=0; i<j; i++){
//             v.coeffRef(i) = A.coeff(j, i)*A.coeff(i, i);
//         }
//         A.coeffRef(j, j) = A.coeff(j, j) - A.row(j).segment(0, j-1) * v.segment(0,j-1);
//         A.block(j+1, j, n-j, 1) = (A.block(j+1, j, n-j, 1) - A.block(j+1,0,n-j,j-1) * v.segment(0,j-1))/A.coeff(j, j);
//     }
// }