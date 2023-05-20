/**
 * @file test_SDF_Hessian.cpp
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Test the gradients of a SDF.
 * @version 0.1
 * @date 2023-03-15
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "helpers/data_io.h"
#include <gtest/gtest.h>
#include <fastad>
#include "dynamics/DoubleIntegratorDraged.h"
    
using namespace ad;
using namespace vimp;
using namespace Eigen;

EigenWrapper ei;
// Choose autodiff scalar type for 3 variables
using ADouble = TinyAD::Double<3>;

TEST(SDFHessian, read_csv){
    MatrixIO loader;
    MatrixXd sdf = loader.load_csv("data/map_ground_truth.csv");

    MatrixXd map_ground_truth = (MatrixXd(7, 7) <<
                0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0,
                0, 0, 1, 1, 1, 0, 0,
                0, 0, 1, 1, 1, 0, 0,
                0, 0, 1, 1, 1, 0, 0,
                0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0).finished();

    ASSERT_EQ((sdf - map_ground_truth).norm(), 0);
    
}


TEST(SDFHessian, ADinitialization){
    // === The example on the TinyAD github ===

    // Init a 3D vector of active variables and a 3D vector of passive variables
    Eigen::Vector3<ADouble> x = ADouble::make_active({0.0, -1.0, 1.0});
    Eigen::Vector3<double> y(2.0, 3.0, 5.0);

    // Compute angle using Eigen functions and retrieve gradient and Hessian w.r.t. x
    ADouble angle = acos(x.dot(y) / (x.norm() * y.norm()));
    Eigen::Vector3d g = angle.grad;
    Eigen::Matrix3d H = angle.Hess;

    // ei.print_matrix(H, "Hessian");
}

TEST(SDFHessian, ADHessian){
    MatrixXd Sig_obs = ei.random_psd(3);
    VectorXd x = ei.random_vector(3);
    Eigen::Vector3<ADouble> x_ad = ADouble::make_active(x);
    ADouble f_ad = x_ad.transpose() * Sig_obs * x_ad;
    Eigen::Matrix3d H = f_ad.Hess;
    ASSERT_LE((H - Sig_obs*2).norm(), 1e-10);
    
}

TEST(SDFHessian, ADJacobian){
    MatrixXd Sig_obs = ei.random_psd(3);
    VectorXd x = ei.random_vector(3);
    Eigen::Vector3<ADouble> x_ad = ADouble::make_active(x);

    Eigen::Vector3<ADouble> f;
    f << x_ad(0)+x_ad(1), x_ad(1)+x_ad(2), x_ad(0)+x_ad(2);

    Eigen::Matrix3d Jacobian;
    Jacobian.row(0) = f(0).grad;
    Jacobian.row(1) = f(1).grad;
    Jacobian.row(2) = f(2).grad;
    
    // ei.print_matrix(Jacobian, "Jacobian");
    Eigen::Matrix3d Jacobian_ground_truth;
    Jacobian_ground_truth << 1,1,0,0,1,1,1,0,1;

    ASSERT_LE((Jacobian - Jacobian_ground_truth).norm(), 1e-10);

    // norm function
    auto norm_2 = x_ad.dot(x_ad) / 2;
    Eigen::VectorXd norm_Jacobian = norm_2.grad;
    ASSERT_LE((norm_Jacobian - x).norm(), 1e-10);
    
}

TEST(SDFHessian, LinearizeDynamics){
    using ADouble4 = TinyAD::Double<4>;
    MatrixXd Sig_obs = ei.random_psd(4);
    Vector4d x;
    x << 1.4167, 8.0000, 2.0020, 0.0003;

    // ei.print_matrix(x, "var x");

    Eigen::Vector4<ADouble4> xad = ADouble4::make_active(x);

    // time scaling factor
    double sig = 5.0;

    // drag coefficient
    double c_d = 0.005;
    
    // f & hAk
    Vector4d f;
    f << x(2), x(3), c_d*sqrt(x(2)*x(2) + x(3)*x(3))*x(2), c_d*sqrt(x(2)*x(2) + x(3)*x(3))*x(3);
    
    Eigen::Vector4<ADouble4> fad;
    fad << xad(2), xad(3), c_d*sqrt(xad(2)*xad(2) + xad(3)*xad(3))*xad(2), c_d*sqrt(xad(2)*xad(2) + xad(3)*xad(3))*xad(3);
    
    Eigen::MatrixXd grad_f_T{Eigen::MatrixXd::Zero(4, 4)};
    grad_f_T.row(0) = fad(0).grad;
    grad_f_T.row(1) = fad(1).grad;
    grad_f_T.row(2) = fad(2).grad;
    grad_f_T.row(3) = fad(3).grad;

    grad_f_T = grad_f_T * sig;

    Eigen::MatrixXd grad_f_ground_truth{Eigen::MatrixXd::Zero(4, 4)};
        grad_f_ground_truth <<  0,  0,                   5.0000,                        0,
                                0,  0,                        0,                   5.0000,
                                0,  0,        0.100097879990775,    6.550887313354089e-06,
                                0,  0,    6.550887313354089e-06,        0.050048941281552;
    
    ASSERT_LE((grad_f_T - grad_f_ground_truth).norm(), 1e-5);

    // hak
    Vector4d hAkx = grad_f_T*x;
    auto hak = sig*f - hAkx;

    Vector4d hak_ground_truth;
    hak_ground_truth << 0,0,-0.1002, -1.3115e-05;

    ASSERT_LE((hak-hak_ground_truth).norm(), 1e-5);

}


TEST(SDFHessian, TinyADnTr){
    using ADouble4 = TinyAD::Double<4>;
    MatrixXd Sig_obs = ei.random_psd(4);

    // time scaling factor
    double sig = 5.0;

    // drag coefficient
    double cd = 0.005;

    Vector4d x;
    x << 1.4167, 8.0000, 1.7397, -0.2884;

    // ei.print_matrix(x, "x data");

    Eigen::Vector4<ADouble4> xad = ADouble4::make_active(x);
 
    Eigen::Matrix4<ADouble4> grad_f_T;
    grad_f_T << 0,  0,                                 1,                                                                0,
                0,  0,                                 0,                                                                1, 
                0,  0,  -cd*(2*xad(2)*xad(2)+xad(3)*xad(3))/sqrt(xad(2)*xad(2)+xad(3)*xad(3)),   -cd*xad(2)*xad(3)/sqrt(xad(2)*xad(2)+xad(3)*xad(3)),
                0,  0,  -cd*xad(2)*xad(3)/sqrt(xad(2)*xad(2)+xad(3)*xad(3)),                     -cd*(xad(2)*xad(2)+2*xad(3)*xad(3))/sqrt(xad(2)*xad(2)+xad(3)*xad(3));
    grad_f_T = sig * grad_f_T;

    Eigen::Matrix4<ADouble4> grad_f;
    grad_f = grad_f_T.transpose();

    // // grad(Tr(BBT*(grad_f_T - Ak)*Sigk*(grad_f_T' - Ak')))
    Eigen::MatrixXd pinv_BBT(4,4);
    pinv_BBT << 0,0,0,0,
                0,0,0,0,
                0,0,1,0,
                0,0,0,1;
    pinv_BBT = pinv_BBT/sig/sig;

    Eigen::MatrixXd Sigk(4,4);
    Sigk << 0.0100,         0,    0.0019,    0.0000,
                 0,    0.0100,    0.0000,    0.0019,
            0.0019,    0.0000,    0.0183,    0.0000,
            0.0000,    0.0019,    0.0000,    0.0183;

    Eigen::MatrixXd Ak(4,4);
    Ak <<       0,         0,    5.0000,         0,
                0,         0,         0,    5.0000,
          -0.4500,    0.0000,   -2.7077,    0.0000,
           0.0000,   -0.4500,    0.0000,   -2.7078;

    Eigen::MatrixXd Ak_T(4,4);
    Ak_T = Ak.transpose();

    Eigen::Matrix4<ADouble4> temp1;
    temp1 = grad_f_T - Ak;

    Eigen::Matrix4<ADouble4> temp2;
    temp2 = grad_f - Ak_T;

    Eigen::Matrix4<ADouble4> temp3;
    temp3 = pinv_BBT*temp1;

    Eigen::Matrix4<ADouble4> temp4;
    temp4 = temp3*Sigk;
    
    auto res = temp4*temp2;
    auto nTr = res.trace().grad;

    // std::cout << "Tr.grad" << std::endl << nTr << std::endl;

    Eigen::Vector4d nTr_groundtruth;
    nTr_groundtruth << 0, 0, -2.904078820914141e-04, 4.814376953478175e-05;

    ASSERT_LE((nTr - nTr_groundtruth).norm(), 1e-6);

    // // Test function
    vimp::DoubleIntegrator dyn(4, 2, 25);
    std::tuple<MatrixXd, MatrixXd, VectorXd, VectorXd> linearize_res;
    linearize_res = dyn.linearize_timestamp(x, sig, Ak, Sigk);
    Eigen::Matrix4d fhAk = std::get<0>(linearize_res);
    Eigen::MatrixXd fB = std::get<1>(linearize_res);
    Eigen::Vector4d fhak = std::get<2>(linearize_res);
    Eigen::Vector4d fnTr = std::get<3>(linearize_res);

    ASSERT_LE((fnTr - nTr_groundtruth).norm(), 1e-6);
    
}

TEST(SDFHessian, FastADDynamics){
    Eigen::MatrixXd Sig_obs = ei.random_psd(4);
    Eigen::Vector4d x;
    x << 1.4167, 8.0000, 2.0020, 0.0003;

    Eigen::Vector4d x_adj;
    x_adj.setZero();

    // ei.print_matrix(x, "var x");

    // Initialize variable.
    VarView<double, mat> xad(x.data(), x_adj.data(), 4, 1);
    // time scaling factor
    double sig = 5.0;

    // drag coefficient
    double c_d = 0.005;

    // Create expr. Use a row buffer to store data. Then we only need to manipulate data when
    // looping.

    Eigen::Vector4d a1, a2, a3, a4;
    a1 << 1,0,0,0;
    a1 << 0,1,0,0;
    a3 << 0,0,1,0;
    a4 << 0,0,0,1;
    auto cst1 = ad::constant_view(a1.data(), 1, 4);
    auto cst2 = ad::constant_view(a2.data(), 1, 4);
    auto cst3 = ad::constant_view(a3.data(), 1, 4);
    auto cst4 = ad::constant_view(a4.data(), 1, 4);
    auto f1 = ad::bind(ad::dot(cst1, xad));
    auto f2 = ad::bind(ad::dot(cst2, xad));
    auto f3 = ad::bind(ad::dot(cst3, xad) * c_d * ad::sqrt(ad::pow<2>(ad::dot(cst3, xad)) + ad::pow<2>(ad::dot(cst4, xad))));
    auto f4 = ad::bind(ad::dot(cst4, xad) * c_d * ad::sqrt(ad::pow<2>(ad::dot(cst3, xad)) + ad::pow<2>(ad::dot(cst4, xad))));

    // Seed
    Eigen::MatrixXd seed(1, 1);
    seed.setOnes(); // Usually seed is 1. DONT'T FORGET!

    auto grad_1 = autodiff(f1, seed.array());
    // std::cout << xad.get_adj() << std::endl;
    // std::cout << "f_grad: " << grad_1 << std::endl;

    // auto f_grad = ad::bind(ad::dot(cst1, grad_1));
    
    // auto grad_grad_1 = autodiff(f_grad, seed.array());
    // std::cout << xad.get_adj() << std::endl;

    auto grad_2 = autodiff(f2, seed.array());
    auto grad_3 = autodiff(f3, seed.array());
    auto grad_4 = autodiff(f4, seed.array());
    
}

// FastAD: https://github.com/JamesYang007/FastAD#installation

TEST(SDFHessian, FastAD_FWD)
{   
    ForwardVar<double> w1(0.), w2(1.);
    w1.set_adjoint(1.); // differentiate w.r.t. w1
    ForwardVar<double> w3 = w1 * sin(w2);
    ForwardVar<double> w4 = w3 + w1 * w2;
    ForwardVar<double> w5 = exp(w4 * w3);

    // std::cout << "f(x, y) = exp((x * sin(y) + x * y) * x * sin(y))\n"
    //           << "df/dx = " << w5.get_adjoint() << std::endl;
}

TEST(SDFHessian, FastAD_BWD){
    // Create data matrix.
    Eigen::MatrixXd X(5, 2);
    X << 1, 10, 2, 20, 3, 30, 4, 40, 5, 50;
    Eigen::VectorXd y(5);
    y << 32, 64, 96, 128, 160; // y=2*x1+3*x2

    // Generating buffer.
    Eigen::MatrixXd theta_data(2, 1);
    theta_data << 1, 2;
    Eigen::MatrixXd theta_adj(2, 1);
    theta_adj.setZero(); // Set adjoints to zeros.

    // Initialize variable.
    VarView<double, mat> theta(theta_data.data(), theta_adj.data(), 2, 1);

    // Create expr. Use a row buffer to store data. Then we only need to manipulate data when
    // looping.
    Eigen::MatrixXd x_row_buffer = X.row(0);
    auto xi = constant_view(x_row_buffer.data(), 1, X.cols());
    Eigen::MatrixXd y_row_buffer = y.row(0);
    auto yi = constant_view(y_row_buffer.data(), 1, y.cols());
    auto expr = bind(pow<2>(yi - dot(xi, theta)));

    // Seed
    Eigen::MatrixXd seed(1, 1);
    seed.setOnes(); // Usually seed is 1. DONT'T FORGET!

    // Loop over each row to calulate loss.
    double loss = 0;
    for (int i = 0; i < X.rows(); ++i) {
        x_row_buffer = X.row(i);
        y_row_buffer = y.row(i);

        auto f = autodiff(expr, seed.array());
        loss += f.coeff(0);
    }

    // // Print results.
    // std::cout << "loss: " << loss << std::endl; // 6655
    // std::cout << theta.get() << std::endl;      //[1, 2]
    // std::cout << theta.get_adj() << std::endl;  //[-1210, -12100]

    theta_adj.setZero(); // Reset differential to zero after one full pass.

}



