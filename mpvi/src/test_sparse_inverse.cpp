//
// Created by hongzhe on 3/13/22.
//

/// test the inverse of a sparse matrix

#include "../include/SparseInverseMatrix.h"

typedef Triplet<double> T;
using namespace std;
using namespace SparseInverse;

int main(){
    int dimension = 6;
    SpMatrix sp_precision(6, 6);
    sp_precision.reserve(dimension*3-2);
    vector<T> vec_triplet;

    vec_triplet.emplace_back(0,0,1);
    vec_triplet.emplace_back(0,1,0.2);
    for (int i=1; i<dimension-1; i++){
        vec_triplet.emplace_back(i, i,1);
        vec_triplet.emplace_back(i, i-1, 0.2);
        vec_triplet.emplace_back(i, i+1, 0.2);
    }
    vec_triplet.emplace_back(dimension-1,dimension-1,1);
    vec_triplet.emplace_back(dimension-1,dimension-2,0.2);

    sp_precision.setFromTriplets(vec_triplet.begin(), vec_triplet.end());

    cout << "sparse precision " << endl << sp_precision;
    // Natural ordering is important here! Eigen Cholesky is up to a permutation.
    // https://stackoverflow.com/questions/53225371/simplicialllt-returns-wrong-cholesky-factors
    SimplicialLDLT<SpMatrix, Eigen::Lower, Eigen::NaturalOrdering<int>> sparse_LDLT_target{sp_precision};

    // test the inverse algorithm here
    sparse_inverser inverser(sp_precision);

    MatrixXd test_inverse = inverser.inverse();

    cout << "inverse matrix difference " << endl << test_inverse - sp_precision.toDense().inverse() << endl;

}