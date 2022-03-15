//
// Created by hongzhe on 2/28/22.
//

#ifndef C_DRAFT_SPARSEMATRIXHELPER_H
#define C_DRAFT_SPARSEMATRIXHELPER_H

#endif //C_DRAFT_SPARSEMATRIXHELPER_H
#include <gtsam/3rdparty/Eigen/Eigen/Sparse>
#include <gtsam/3rdparty/Eigen/Eigen/SparseCore>

using namespace Eigen;
using namespace std;
typedef Triplet<double> T;
typedef SparseMatrix<double> SpMatrix;

namespace Sparsehelper{
    struct tridiagonalizer{
        tridiagonalizer(const int& n): size(n){}

        const int size;

        SpMatrix& operator()(const SpMatrix& input) const{
            SpMatrix* output;
            output = new SpMatrix(size, size);
            const int nnz = 3*size-2;
            output->reserve(nnz);
            vector<T> triplets;
            triplets.reserve(nnz);
            triplets.emplace_back(0, 0, input.coeff(0, 0));

            for (int i=1; i<size; i++)
            {
                triplets.emplace_back(i-1, i, input.coeff(i-1, i));
                triplets.emplace_back(i, i-1, input.coeff(i, i-1));
                triplets.emplace_back(i, i, input.coeff(i, i));
            }
            output->setFromTriplets(triplets.begin(), triplets.end());
            return *output;
        }

        SpMatrix& operator()(const Eigen::MatrixXd& input) const{
            SpMatrix* output;
            output = new SpMatrix(size, size);
            const int nnz = 3*size-2;
            output->reserve(nnz);
            vector<T> triplets;
            triplets.reserve(nnz);
            triplets.emplace_back(0, 0, input(0, 0));

            for (int i=1; i<size; i++)
            {
                triplets.emplace_back(i-1, i, input(i-1, i));
                triplets.emplace_back(i, i-1, input(i, i-1));
                triplets.emplace_back(i, i, input(i, i));
            }
            output->setFromTriplets(triplets.begin(), triplets.end());
            return *output;
        }

    };
}