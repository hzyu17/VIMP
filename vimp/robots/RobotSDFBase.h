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

#define STRING(x) #x
#define XSTRING(x) STRING(x)

#include "GaussianVI/helpers/MatrixHelper.h"
#include <gpmp2/obstacle/ObstaclePlanarSDFFactor.h>
#include <Eigen/Dense>

using namespace Eigen;

namespace vimp{

template <typename ROBOT, typename SDF, typename SDFFACTOR>
class RobotSDFBase{

public:
    virtual ~RobotSDFBase(){}

    RobotSDFBase(){}

    RobotSDFBase(int ndof, int nlinks): 
    _ndof(ndof), _nlinks(nlinks){}

    RobotSDFBase(int ndof, int nlinks, int map_dim, const std::string& map_name): 
    _ndof(ndof), _nlinks(nlinks), _map_name(map_name), _origin(map_dim)
    {

        // std::cout << "map_name" << std::endl << map_name << std::endl;
        if (map_dim == 2){
                /// map and sdf
                std::string source_root{XSTRING(SOURCE_ROOT)};

                _origin.setZero();
                _origin << -20.0, -10.0;
                _cell_size = 0.1;

                if (strcmp(map_name.data(), "2dpr_map0") == 0){
                    _field_file = source_root+"/maps/2dpR/map0/field_multiobs_map0.csv";
                    // layout of SDF: Bottom-left is (0,0), length is +/- cell_size per grid.
                    _origin.setZero();
                    _origin << -1, -1;
                    _cell_size = 0.01;
                }
                else if(strcmp(map_name.data(), "2dpr_map1") == 0){
                    _field_file = source_root+"/maps/2dpR/map1/field_multiobs_map1.csv";

                }
                else if(strcmp(map_name.data(), "2dpr_map2") == 0){
                    _field_file = source_root+"/maps/2dpR/map2/field_multiobs_map2.csv";

                }
                else if(strcmp(map_name.data(), "2dpr_map3") == 0){
                    _field_file = source_root+"/maps/2dpR/map3/field_multiobs_map3.csv";
                }
                else if(strcmp(map_name.data(), "2darm_map1") == 0){
                    _cell_size=0.01;
                    _origin << -1.0, -1.0;
                    _field_file = source_root+"/maps/2dArm/field_one_obs.csv";
                }
                else if(strcmp(map_name.data(), "2darm_map2") == 0){
                    _cell_size=0.01;
                    _origin << -1.0, -1.0;
                    _field_file = source_root+"/maps/2dArm/field_two_obs.csv";
                }
                else if(strcmp(map_name.data(), "single_obs") == 0){
                    _cell_size=0.1;
                    _origin << -5.0, -5.0;
                    _field_file = source_root+"/maps/2dQuad/singleObstacleMap_field.csv";
                }

                else{
                    std::runtime_error("No such map for 2d point robot sdf!");
                }

                // if (!field_file.empty()){
                //     Base::update_field_file(field_file);
                // }

        }else if (map_dim==3){
            std::cout << "3-D workspace map" << std::endl;
        }
            
    }
    

    virtual void update_sdf(const SDF& sdf) = 0;
    virtual void default_sdf(){};

    /**
     * Obstacle Returns the Vector of h(x) and the Jacobian matrix.
     * */
    virtual std::tuple<VectorXd, MatrixXd> hinge_jacobian(const VectorXd& pose){
        MatrixXd Jacobian;

        VectorXd vec_err{_psdf_factor->evaluateError(pose, Jacobian)};
        return std::make_tuple(vec_err, Jacobian);
    }

    /**
     * @brief Hinge loss function for nonlinear robot dynamics model.
     */
    virtual std::tuple<VectorXd, MatrixXd> hinge_jacobian_nonlinear_dyn(const VectorXd& pose){}

    inline ROBOT RobotModel() const { return _robot; }
    inline std::shared_ptr<SDF> psdf() const { return _psdf; }
    inline SDF sdf() const { return _sdf; }
    virtual inline int ndof() const {return _ndof;}
    virtual inline int nlinks() const {return _nlinks;}

    Eigen::VectorXd map_origin() const { return _origin; }
    double cell_size() const { return _cell_size; }
    std::string field_file() const { return _field_file; }

public:
    ROBOT _robot;
    SDF _sdf;
    Eigen::VectorXd _origin;
    double _cell_size;

    std::shared_ptr<SDFFACTOR> _psdf_factor; 
    std::shared_ptr<SDF> _psdf;
    int _ndof, _nlinks;

    gvi::MatrixIO _m_io;

    std::string _map_name, _field_file;
};

}