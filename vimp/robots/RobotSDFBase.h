/**
 * @file RobotSDFBase.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief The base class defining a robot with a sdf class.
 * For linear dynamics, we use the RobotModel class defined in gpmp2; 
 * For nonlinear dynamics we define seperate robot model class under dynamics/ directory.  
 * 
 * Every derived class must have 2 functions: 
 * 1. update_sdf(); 
 * 2. default_sdf(); 
 * 
 * The main function is 
 * hinge_jacobian(pose) which returns the hinge loss and its gradients wrt the pose.
 * 
 * The class has 3 template parameters: 
 * 1. the robot model (including forward kinematics and the collision checking balls);
 * 2. the SDF;
 * 3. the collision checking factor which evaluates the hinge loss and its gradients.
 * 
 * @version 0.1
 * @date 2023-04-14
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include "../helpers/MatrixIO.h"
#include <Eigen/Dense>

using namespace Eigen;

namespace vimp{

template <typename ROBOT, typename SDF, typename SDFFACTOR>
class RobotSDFBase{

public:
    RobotSDFBase(){}
    RobotSDFBase(int ndof, int nlinks): _ndof(ndof), _nlinks(nlinks){}
    
    virtual ~RobotSDFBase(){}

    virtual void update_sdf(const SDF& sdf) = 0;
    virtual void default_sdf() = 0;

    /**
     * Obstacle Returns the Vector of h(x) and the Jacobian matrix.
     * */
    virtual std::tuple<VectorXd, MatrixXd> hinge_jacobian(const VectorXd& pose){
        MatrixXd Jacobian;
        VectorXd vec_err = _psdf_factor->evaluateError(pose, Jacobian);
        return std::make_tuple(vec_err, Jacobian);
    }

    /**
     * @brief Hinge loss function for nonlinear robot dynamics model.
     */
    virtual std::tuple<VectorXd, MatrixXd> hinge_jacobian_nonlinear_dyn(const VectorXd& pose){}

    inline ROBOT RobotModel() const { return _robot; }
    inline std::shared_ptr<SDF> psdf() const { return _psdf; }
    inline SDF sdf() const { return *_psdf; }
    inline int ndof() const {return _ndof;}
    inline int nlinks() const {return _nlinks;}

public:
    ROBOT _robot;
    SDF _sdf;
    std::shared_ptr<SDFFACTOR> _psdf_factor; 
    std::shared_ptr<SDF> _psdf;
    int _ndof, _nlinks;

    MatrixIO _m_io;
};

}