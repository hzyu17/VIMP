/**
 * @file hingeloss_helper.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief Helper functions for hing loss in a sdf
 * @version 0.1
 * @date 2022-08-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <gpmp2/obstacle/ObstacleCost.h>
#include <gpmp2/obstacle/PlanarSDF.h>
#include "data_io.h"
using namespace Eigen;


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
inline MatrixXd mesh_hingeloss(const MatrixXd& mesh_x, const MatrixXd& mesh_y, const gpmp2::PlanarSDF& sdf, double eps){
    
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


/**
 * @brief compute hinge losses from a matrix read from csv file
 * @param filename 
 * The data is in shape (n_iters, N*dim)
 */
inline void hinge_loss_io(const string& filename){
    vimp::MatrixIO m_io;
    
}
