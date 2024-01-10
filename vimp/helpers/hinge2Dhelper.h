/**
 * @file hinge2Dhelper.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Helper functions for hing loss in a 2D sdf
 * @version 0.1
 * @date 2022-08-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <gpmp2/obstacle/ObstacleCost.h>
#include <gpmp2/obstacle/PlanarSDF.h>
#include "GaussianVI/helpers/MatrixHelper.h"
#include "GaussianVI/helpers/EigenWrapper.h"

using namespace Eigen;
using namespace gvi;

/**
 * @brief given a collection of states (length N*4), returns a vector of hingelosses (length N)
 * @param means 
 */
inline VectorXd planar_hingeloss(const VectorXd& means, const gpmp2::PlanarSDF& sdf, double eps){
    int dim_conf = 2;
    int dim_theta = 4;
    int N = means.size() / dim_theta;

    VectorXd hinge_loss = VectorXd::Zero(N);

    for (int i=0; i< N; i++){
        int s_indx = i*dim_theta;
        VectorXd point(2);
        point << means(s_indx*N), means(s_indx*N+1);
        hinge_loss(i) = gpmp2::hingeLossObstacleCost(point, sdf, eps);
    }

    return hinge_loss;

}


/**
 * @brief 
 * 
 * @param mesh_x: mesh points of x coordinate 
 * @param mesh_y: mesh points of y coordinate
 * @return MatrixXd: mesh points of z coordinate which is the hinge loss on (x, y)
 */
inline MatrixXd mesh_hingeloss(const MatrixXd& mesh_x, 
                               const MatrixXd& mesh_y, 
                               const gpmp2::PlanarSDF& sdf, 
                               double eps)
{
    MatrixXd mesh_hinge{MatrixXd::Zero(mesh_x.cols(), mesh_y.cols())};
    for (int i = 0; i<mesh_x.cols(); i++){
        for (int j=0; j<mesh_y.cols(); j++){
            VectorXd point(2);
            point << mesh_x(i), mesh_y(j);
            mesh_hinge(i, j) = gpmp2::hingeLossObstacleCost(point, sdf, eps);
        }
    }

    return mesh_hinge;
}


std::pair<double, VectorXd> hingeloss_gradient_point(double x, double y, 
                            const gpmp2::PlanarSDF& sdf, 
                            double eps, 
                            Eigen::MatrixXd& Jacobian)
{
    VectorXd point(2);
    point << x, y;
    Jacobian = MatrixXd::Zero(1, 2);
    double hinge_loss = gpmp2::hingeLossObstacleCost(point, sdf, eps, Jacobian);
    MatrixXd Jacobian_column = Jacobian.transpose();
    std::pair<double, VectorXd> res = std::make_pair(hinge_loss, Jacobian_column);
    return res;
}


/**
 * @brief 
 * 
 * @param mesh_x: mesh points of x coordinate 
 * @param mesh_y: mesh points of y coordinate
 * @return MatrixXd: mesh points of z coordinate which is the hinge loss on (x, y)
 */
using vec_1d = std::vector<double>;
using vec_2d = std::vector<vec_1d>;
std::tuple<MatrixXd, vec_2d, vec_2d> hingeloss_gradient_mesh(const MatrixXd& mesh_x, 
                                        const MatrixXd& mesh_y, 
                                        const gpmp2::PlanarSDF& sdf, 
                                        double eps, 
                                        MatrixXd Jacobians){
    
    MatrixXd mesh_hinge{MatrixXd::Zero(mesh_x.cols(), mesh_y.cols())};
    Jacobians = MatrixXd::Zero(2, mesh_x.cols()*mesh_y.cols());

    // return: mesh hinge loss, mesh grad_x, mesh grad_y
    std::tuple<MatrixXd, vec_2d, vec_2d> res;
    EigenWrapper ei;
    vec_2d grad_x(mesh_x.cols());
    vec_2d grad_y(mesh_x.cols());
    
    for (int i = 0; i<mesh_x.cols(); i++){
        vec_1d grad_xi(mesh_y.cols());
        vec_1d grad_yi(mesh_y.cols());
        for (int j=0; j<mesh_y.cols(); j++){
            MatrixXd Jacobian_ij = MatrixXd::Zero(1, 2);
            VectorXd point(2);
            point << mesh_x(i), mesh_y(j);
            mesh_hinge(i, j) = gpmp2::hingeLossObstacleCost(point, sdf, eps, Jacobian_ij);
            // MatrixXd Jacobian_ij_column = Jacobian_ij.transpose();
            // ei.compress3d(Jacobian_ij_column, Jacobians, i*j);
            grad_xi[j] = Jacobian_ij(0);
            grad_yi[j] = Jacobian_ij(1);

        }
        grad_x[i] = grad_xi;
        grad_y[i] = grad_yi;
    }

    res = std::make_tuple(mesh_hinge, grad_x, grad_y);
    return res;
}


/**
 * @brief compute hinge losses from a matrix read from csv file
 * @param filename 
 * The data is in shape (n_iters, N*dim)
 */
inline void hinge_loss_io(const std::string& filename){
    gvi::MatrixIO m_io;
    
}
