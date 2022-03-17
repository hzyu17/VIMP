// Test the Multi-variate Gaussian sampling

#include <iostream>
#include "../include/matplotlibcpp.h"
#include "../include/MVGsampler.h"
#include "../include/SparseMatrixHelper.h"
#include <gtsam/base/Matrix.h>

namespace plt = matplotlibcpp;
using namespace Eigen;
using namespace std;
using namespace GaussianSampler;
using namespace Sparsehelper;
typedef SparseMatrix<double> SpM;
typedef Triplet<double> T;

bool test_tridiag_trunc(){
    // test the tridiagonal truncation function
    const int size_M = 10;
    tridiagonalizer tridiagtool(size_M);
    SpM inputM(10, 10);
    vector<T> triplets;
    for (int i=3; i<size_M; i++)
    {
        triplets.emplace_back(T(i,i,1));
        triplets.emplace_back(T(i-1,i,1));
        triplets.emplace_back(T(i-2,i,1));
        triplets.emplace_back(T(i-3,i,1));
        triplets.emplace_back(T(i,i-1,1));
        triplets.emplace_back(T(i,i-2,1));
        triplets.emplace_back(T(i,i-3,1));
    }

    inputM.setFromTriplets(triplets.begin(), triplets.end());
    cout << "input matrix " << endl << inputM << endl;
    SpM output{tridiagtool(inputM)};

    cout << "output matrix " << endl << output << endl;
    return true;
}


bool test_Gaussian_samplers()
{
    // sampling from Gaussian distribution
    int size = 3;
    int nn = 100000; // number of samples
    VectorXd mean(size);
    MatrixXd covar(size,size);
    MatrixXd precision(size, size);
    precision << 1, -0.3, 0,
            -0.3, 3, 0.2,
            0, 0.2, 4;
    mean << 1, 1, 1;
    covar = precision.inverse();
    cout << "covar" << endl << covar << endl;

    SparseMatrix<double> sparse_precision(size, size);
    // Insertion of sparse matrix
    typedef Triplet<double> T;
    vector< T > tripletList;
    tripletList.reserve(7);

    tripletList.emplace_back(0,0,precision(0,0));
    tripletList.emplace_back(1,1,precision(1,1));
    tripletList.emplace_back(2,2,precision(2,2));
    tripletList.emplace_back(0,1,precision(0,1));
    tripletList.emplace_back(1,0,precision(1,0));
    tripletList.emplace_back(1,2,precision(1,2));
    tripletList.emplace_back(2,1,precision(2,1));

    sparse_precision.setFromTriplets(tripletList.begin(), tripletList.end());

    gaussian_sampler_precision sampler_precision { mean, precision };
    normal_random_variable sampler_covar {mean, covar};
    GaussianSamplerSparsePrecision sampler_sparse_precision {mean, sparse_precision};

    MatrixXd samples_precision{sampler_precision(nn)};
    MatrixXd samples_sparse_precision{sampler_sparse_precision(nn)};
    MatrixXd samples_covar{sampler_covar(nn)};

    // plotting in the 2D case
//    plt::subplot(2,2,1);
//    plt::hist(vector<float> {samples_covar.row(0).data(), samples_covar.row(0).data()+samples_covar.cols()}, 20, "r");
//    plt::subplot(2,2,2);
//    plt::hist(vector<float> {samples_covar.row(1).data(), samples_covar.row(1).data()+samples_covar.cols()}, 20, "r");
//
//    plt::subplot(2,2,3);
//    plt::hist(vector<float> {samples_precision.row(0).data(), samples_precision.row(0).data()+samples_precision.cols()}, 20, "b");
//    plt::subplot(2,2,4);
//    plt::hist(vector<float> {samples_precision.row(1).data(), samples_precision.row(1).data()+samples_precision.cols()}, 20, "b");

//    plt::show();

    // imperically test the covariances
    VectorXd mu_cov {samples_covar.rowwise().sum() / nn};
    VectorXd mu_prec {samples_precision.rowwise().sum() / nn};
    VectorXd mu_sparse_prec {samples_sparse_precision.rowwise().sum() / nn};

    MatrixXd Cov_cov{((samples_covar.colwise() - mu_cov) * (samples_covar.colwise() - mu_cov).transpose()) / (nn-1)};
    MatrixXd Cov_prec{((samples_precision.colwise() - mu_prec) * (samples_precision.colwise() - mu_prec).transpose()) / (nn-1)};
    MatrixXd Cov_sparse_prec{((samples_sparse_precision.colwise() - mu_sparse_prec) * (samples_sparse_precision.colwise() - mu_sparse_prec).transpose()) / (nn-1)};

    cout << "covariance cov error norm" << endl << (Cov_cov - covar).norm() << endl;
    cout << "covariance precision error norm" << endl << (Cov_prec - covar).norm() << endl;
    cout << "covariance sparse precision error norm" << endl << (Cov_sparse_prec - covar).norm() << endl;
    cout << "covariance sparse precision matrix" << endl << Cov_sparse_prec << endl;

    // Test update covariance method
    cout << "Test the update covariance matrix method " << endl;
    sparse_precision.coeffRef(0,2) += 0.2;
    sparse_precision.coeffRef(2,0) += 0.2;

    sampler_sparse_precision.updatePrecisionMatrix(sparse_precision);
    samples_sparse_precision = sampler_sparse_precision(nn);

    mu_sparse_prec = samples_sparse_precision.rowwise().sum() / nn;
    Cov_sparse_prec = ((samples_sparse_precision.colwise() - mu_sparse_prec) *
            (samples_sparse_precision.colwise() - mu_sparse_prec).transpose()) / (nn-1);

    precision(0, 2) += 0.2;
    precision(2, 0) += 0.2;
    cout << "new covariance matrix" << endl << precision.inverse() << endl << endl;
    cout << "covariance sparse precision matrix" << endl << Cov_sparse_prec << endl;
    cout << "error norm " << (Cov_sparse_prec - precision.inverse()).norm() << endl;

    return true;
}

// Test the sampling estimation of the expectations
bool test_impirical_expectation() {

    const int ndim = 8;
    const int nn = 100000;
    const int nnz = 3*ndim-2;

    SpMatrix invSigma(ndim, ndim);
    invSigma.reserve(nnz);

    // filling sparse matrix
    vector< T > tripletList;
    tripletList.reserve(nnz);

    for (int i=0; i<ndim; i++)
    {
        tripletList.emplace_back(i,i,1);
    }
    invSigma.setFromTriplets(tripletList.begin(), tripletList.end());

    gtsam::Vector2 q1, q2, qdot1, qdot2;

    // origin zero  and stationary case
    q1    = gtsam::Vector2(0, 0);
    q2    = gtsam::Vector2(17, 14);
    qdot1 = gtsam::Vector2(0, 0);
    qdot2 = gtsam::Vector2(0, 0);

    gtsam::Vector mu = gtsam::concatVectors({q1, qdot1, q2, qdot2});

    GaussianSamplerSparsePrecision sampler_sparse_precision{mu, invSigma};
    sampler_sparse_precision.updateMean(mu);
    sampler_sparse_precision.updatePrecisionMatrix(invSigma);

    //generate samples
    MatrixXd samples(ndim, nn);
    samples = sampler_sparse_precision(nn);

    gtsam::Vector losses(nn);
    gtsam::Vector Vdmu = gtsam::Vector::Zero(ndim);
    gtsam::Matrix Vddmu = gtsam::Matrix::Zero(ndim, ndim);

    for (int i = 0; i < 2; i++) {
        Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> qr_solver(invSigma);
        // Calulating the expectations
        auto colwise = samples.colwise();
        std::for_each(colwise.begin(), colwise.end(), [&](auto const &elem) {

            q1 = elem(seq(0, 1));
            q2 = elem(seq(2, 3));
            qdot1 = elem(seq(4, 5));
            qdot2 = elem(seq(6, 7));

            cout << "elem - mu " << endl << elem - mu << endl;
            cout << "(elem - mu) * (elem - mu).transpose() " << endl << (elem - mu) * (elem - mu).transpose() << endl;
            Vdmu = Vdmu + (elem - mu);
            Vddmu = Vddmu + (elem - mu) * (elem - mu).transpose();
        });

        Vdmu = Vdmu / nn;
        Vddmu = invSigma * (Vddmu / nn);

        gtsam::Vector ground_truth_mean(ndim);
        ground_truth_mean = gtsam::zero(ndim);

        gtsam::Matrix ground_truth_Cov(ndim, ndim);
        ground_truth_Cov = gtsam::Matrix::Identity(ndim, ndim);

        cout << "Vdmu " << endl << Vdmu << endl;
        cout << "Vddmu " << endl << Vddmu << endl;

        assert((Vdmu-ground_truth_mean).norm()<1e-3);
        assert((Vddmu-ground_truth_Cov).norm()<1e-3);

    }

    return true;
}


int main() {
    std::cout << "Hello, World!" << std::endl;
    test_Gaussian_samplers();
//    test_tridiag_trunc();
//    test_impirical_expectation();
    return 0;
}
