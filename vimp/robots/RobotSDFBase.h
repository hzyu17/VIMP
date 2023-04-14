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

#include "../helpers/data_io.h"
#include <Eigen/Dense>

using namespace Eigen;

namespace vimp{

template <typename ROBOT, typename SDF, typename SDFFACTOR>
class RobotSDFBase{

public:
    RobotSDFBase(){}
    virtual ~RobotSDFBase(){}

    virtual ROBOT RobotModel() const = 0;
    virtual std::shared_ptr<SDF> sdf() const = 0;
    virtual std::tuple<VectorXd, MatrixXd> hinge_jacobian(const VectorXd& pose) = 0;
    virtual void update_sdf(const SDF& sdf) = 0;
    virtual int ndof() const = 0;
    virtual int nlinks() const = 0;
    virtual void default_sdf() = 0;

protected:
    ROBOT _robot;
    SDF _sdf;
    std::shared_ptr<SDFFACTOR> _psdf_factor; 
    std::shared_ptr<SDF> _psdf;
    int _ndof, _nlinks;

    MatrixIO _m_io;
};

}