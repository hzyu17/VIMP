#include "../include/matplotlibcpp.h"

namespace plt = matplotlibcpp;


/**
* Plot the iteration results
* input: results: (num_iteration, num_states)
* output: figure
**/
void plot_result(const MatrixXd &results, int N, int dim_theta) {
    // plotting in the 2D case
    plt::figure();
    MatrixXd x(results.rows(), N), y(results.rows(), N);
    vector<double> vec_x, vec_y;
    vec_x.resize(x.cols());
    vec_y.resize(x.cols());

    for (int i = 0; i < N; i++) {
        x.col(i) = results.col(i * dim_theta);
        y.col(i) = results.col(i * dim_theta + 1);
    }
    for (int k = x.rows() - 1; k < x.rows(); k++) {
        VectorXd::Map(&vec_x[0], x.cols()) = VectorXd{x.row(k)};
        VectorXd::Map(&vec_y[0], y.cols()) = VectorXd{y.row(k)};
        plt::plot(vec_x, vec_y, "--");
    }


    cout << "x" << endl << x << endl;
    cout << "y" << endl << y << endl;

    plt::grid(true);
    plt::show();
}