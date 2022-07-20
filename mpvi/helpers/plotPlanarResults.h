/**
 * @file plotPlanarResults.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Plotting helper
 * @version 0.1
 * @date 2022-07-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;


/**
* Plot the iteration results
* @param results (num_iteration, num_states)
* @param nstates the total number of states
* @param dim_theta dimension of the configuration variable
* output: figure
**/
void plot_result(const MatrixXd &results, int nstates, int dim_theta) {
    /// plotting in the 2D case
    plt::figure();
    MatrixXd x(results.rows(), nstates), y(results.rows(), nstates);
    /// single point for one iteration
    vector<double> vec_x, vec_y;
    vec_x.resize(x.cols());
    vec_y.resize(x.cols());

    for (int i = 0; i < nstates; i++) {
        /// x coord and y coord
        x.col(i) = results.col(i * dim_theta);
        y.col(i) = results.col(i * dim_theta + 1);
    }
    for (int k = 0; k < x.rows(); k++) {
        VectorXd::Map(&vec_x[0], x.cols()) = VectorXd{x.row(k)};
        VectorXd::Map(&vec_y[0], y.cols()) = VectorXd{y.row(k)};
        plt::scatter(vec_x, vec_y, 3);
    }


    cout << "x" << endl << x << endl;
    cout << "y" << endl << y << endl;

    plt::grid(true);
    plt::show();
}