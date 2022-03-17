//
// Created by hongzhe on 2/27/22.
//

#ifndef C_DRAFT_MGVSAMPLER_H
#define C_DRAFT_MGVSAMPLER_H

#endif //C_DRAFT_MGVSAMPLER_H
#include <gtsam/3rdparty/Eigen/Eigen/Dense>
#include <gtsam/3rdparty/Eigen/Eigen/Sparse>
#include <gtsam/3rdparty/Eigen/Eigen/SparseCore>
#include <gtsam/3rdparty/Eigen/Eigen/SparseCholesky>
#include <random>
#include <iostream>

using namespace Eigen;
using namespace std;

const double PI_ = 3.14159265;

namespace GaussianSampler {
    struct normal_random_variable {
        normal_random_variable(MatrixXd const &covar)
                : normal_random_variable(VectorXd::Zero(covar.rows()), covar) {}

        normal_random_variable(VectorXd const &mean, MatrixXd const &covar)
                : mean(mean), covariance(covar) {
            SelfAdjointEigenSolver<MatrixXd> eigenSolver(covar);
            transform = eigenSolver.eigenvectors() * eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
            cout << "covar eigen transformation " << endl << transform << endl;
        }

        VectorXd mean;
        MatrixXd transform;
        MatrixXd covariance;

        MatrixXd operator()(const int &nn) const {
            static std::mt19937 gen{std::random_device{}()};
//        std::mt19937 gen{ 1 };
            std::normal_distribution<> dist(0, 1);

            return (transform * MatrixXd::NullaryExpr(mean.size(), nn, [&](auto x) { return dist(gen); })).colwise() +
                   mean;
        }

        bool updateCovariance(const MatrixXd& new_cov){
            covariance = new_cov;
            SelfAdjointEigenSolver<MatrixXd> eigenSolver(covariance);
            transform = eigenSolver.eigenvectors() * eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
            return true;
        }

        bool updateMean(const VectorXd& new_mean){
            mean = new_mean;
            return true;
        }
    };

    struct gaussian_sampler_precision {
        gaussian_sampler_precision(MatrixXd const &precision)
                : gaussian_sampler_precision(VectorXd::Zero(precision.rows()), precision) {}

        gaussian_sampler_precision(VectorXd const &mean, MatrixXd const &precision)
                : mean(mean) {
            precision_ = precision;
            LLT<MatrixXd> choleskySolver(precision);
            LDLT<MatrixXd> choleskySolverLDLT(precision);

            assert(choleskySolverLDLT.vectorD().minCoeff() > 0);
            transform = choleskySolver.matrixL();
//            cout << "L*LT" << endl << transform* transform.transpose() << endl;

            transform = transform.inverse().transpose();
            covariance_ = precision.inverse();

        }

        VectorXd mean;
        MatrixXd transform;
        MatrixXd covariance_;
        MatrixXd precision_;

        MatrixXd operator()(const int &nn) const {
            static std::mt19937 gen{std::random_device{}()};
//        std::mt19937 gen(1);
            std::normal_distribution<> dist(0, 1);
//        return MatrixXd{transform * MatrixXd{MatrixXd::NullaryExpr(mean.size(), nn, [&](auto x) { return dist(gen); })}}; //test the fixed seed
            return (transform * MatrixXd::NullaryExpr(mean.size(), nn, [&](auto x) { return dist(gen); })).colwise() +
                   mean;
        }

        bool updatePrecisionMatrix(const Eigen::MatrixXd& newPrecision)
        {
//            // not psd, project back
            LDLT<MatrixXd> choleskySolverLDLT(newPrecision);
            Eigen::VectorXd vectorD{choleskySolverLDLT.vectorD()};

            assert(vectorD.minCoeff() > 0);

            precision_ = newPrecision;
            LLT<MatrixXd> choleskySolver(precision_);
            transform = choleskySolver.matrixL();
            transform = transform.inverse().transpose();
            covariance_ = precision_.inverse();

            return true;
        }

        bool updateMean(const VectorXd& new_mean)
        {
            mean = new_mean;
            return true;
        }


    };

    class GaussianSamplerSparsePrecision {
    public:
        GaussianSamplerSparsePrecision(SparseMatrix<double> const &precision)
                : GaussianSamplerSparsePrecision(VectorXd::Zero(precision.rows()), precision) {}

        GaussianSamplerSparsePrecision(VectorXd const &mean, SparseMatrix<double> const &precision)
                : mean(mean), precision_(precision) {
                updatePrecisionMatrix(precision);
        }

    protected:
        VectorXd mean;
        MatrixXd transform;
        SparseMatrix<double> precision_;
    public:
        bool updatePrecisionMatrix(const SparseMatrix<double>& newPrecision)
        {
            precision_ = newPrecision;
            SimplicialLLT<SparseMatrix<double>, Eigen::Lower, Eigen::NaturalOrdering<int>> sparsecholesky{precision_};
            SimplicialLDLT<SparseMatrix<double>, Eigen::Lower, Eigen::NaturalOrdering<int>> sparseLDLT{precision_};
            sparsecholesky.compute(precision_);
            cout << "spase cholesky LDLT, D matrix" << endl << sparseLDLT.vectorD() << endl;
            cout << "sparsecholesky.matrixL() " << endl << sparsecholesky.matrixL() << endl;

            transform = sparsecholesky.matrixL();

            // test dense eigen decomposition
            MatrixXd precision_dense(precision_);
            SelfAdjointEigenSolver<MatrixXd> eigenSolver(precision_dense);

            if (eigenSolver.eigenvalues().minCoeff() < 0)
            {
                cout << "eigen values " << endl << eigenSolver.eigenvalues() << endl;
            }

            transform = transform.inverse().transpose();
            return true;
        }

        bool updateMean(const VectorXd& new_mean)
        {
            mean = new_mean;
            return true;
        }

        MatrixXd operator()(const int &nn) const {
            static std::mt19937 gen{std::random_device{}()};
//        std::mt19937 gen(1);
            std::normal_distribution<> dist(0, 1);
//        return MatrixXd{transform * MatrixXd{MatrixXd::NullaryExpr(mean.size(), nn, [&](auto x) { return dist(gen); })}}; //test the fixed seed
            return (transform * MatrixXd::NullaryExpr(mean.size(), nn, [&](auto x) { return dist(gen); })).colwise() +
                   mean;
        }

    };

    class Gaussian_distribution{
    public:
        Gaussian_distribution(const Eigen::VectorXd& mean, const Eigen::MatrixXd& covariance):mean_(mean), Covariance_(covariance){
            dimension_ = mean.size();
            precision_ = Covariance_.inverse();
            SelfAdjointEigenSolver<MatrixXd> eigenSolver(precision_);
            assert(eigenSolver.eigenvalues().minCoeff() > 0);
        }
        double probability(Eigen::VectorXd x) const{
            double temp = (-(x-mean_).transpose()*precision_*(x-mean_))(0)/2.0;
            return exp(temp) / sqrt(pow(2 * PI_, dimension_) * Covariance_.determinant());
        }
        double log_prob(Eigen::VectorXd x) const{
            return log(Gaussian_distribution::probability(x));
//            return -((x-mean_).transpose()*precision_*(x-mean_))(0)/2.0 -
//                    dimension_ / 2.0 * log(2*PI_) - log(Covariance_.determinant()) / 2.0;
        }

        Eigen::MatrixXd get_precision(){
            return precision_;
        }

        Eigen::VectorXd get_mean(){
            return mean_;
        }

        Eigen::MatrixXd get_covariance(){
            return Covariance_;
        }

    protected:
        int dimension_;
        Eigen::VectorXd mean_;
        Eigen::MatrixXd Covariance_;
        Eigen::MatrixXd precision_;

    };
}
