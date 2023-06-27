/**
 * @file RobotSDFBase.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief The base class defining a robot with a sdf class.
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