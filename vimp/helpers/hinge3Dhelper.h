/**
 * @file hinge3Dhelper.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Helper functions for hing loss in a 3D sdf
 * @version 0.1
 * @date 2022-08-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <gpmp2/obstacle/ObstacleCost.h>
#include <gpmp2/obstacle/SignedDistanceField.h>
#include "data_io.h"
#include "eigen_wrapper.h"


using namespace Eigen;
using namespace vimp;


/**
 * @brief the hinge loss and gradient of a 3D SDF.
 * 
 * @param mesh_x: mesh points of x coordinate 
 * @param mesh_y: mesh points of y coordinate
 * @param mesh_z: mesh points of z coordinate
 * @return tuple<VectorXd, MatrixXd>: the first return is the meshed hinge losses; 
 * the second is concatenation of the quiver directions [[u1,...,uN]; [v1,...,vN]; [w1,...,wN]].
 */
inline std::tuple<VectorXd, MatrixXd> mesh3D_hinge_gradient(const VectorXd& mesh_x, 
                                                            const VectorXd& mesh_y, 
                                                            const VectorXd& mesh_z,
                                                            const gpmp2::SignedDistanceField& sdf, 
                                                            double eps)
{
    MatrixXd mesh_Jacobians(3, mesh_x.rows());
    VectorXd mesh_hinge(mesh_x.rows());
    EigenWrapper ei;
    for (int i=0; i<mesh_x.rows(); i++){
        VectorXd point(3);
        MatrixXd Jacobian(1, 3);
        point << mesh_x(i), mesh_y(i), mesh_z(i);
        mesh_hinge(i) = gpmp2::hingeLossObstacleCost(point, sdf, eps, Jacobian);
        mesh_Jacobians.col(i) = Jacobian.transpose();
    }
    return std::make_tuple(mesh_hinge, mesh_Jacobians);
}


std::tuple<VectorXd, MatrixXd> hinge_gradient_point(const VectorXd & pose, 
                                                    const gpmp2::SignedDistanceField& sdf, 
                                                    double eps)
{
    VectorXd point(3);
    VectorXd hinge(1);
    MatrixXd Jacobian(1, 3);
    point << pose(0), pose(1), pose(2);
    hinge(0) = gpmp2::hingeLossObstacleCost(point, sdf, eps, Jacobian);
    return std::make_tuple(hinge, Jacobian);
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
    vimp::MatrixIO m_io;
    
}
