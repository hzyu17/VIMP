/**
 * @file Dynamics.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief The base class definition of dynamics.
 * @version 0.1
 * @date 2023-03-30
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once

#include "helpers/EigenWrapper.h"

namespace vimp{

class Dynamics{

public:
    Dynamics(){}

    Dynamics(int nx, int nu, int nt):_nx(nx), _nu(nu), _nt(nt){}

protected:
    int _nx, _nu, _nt;
    EigenWrapper _ei;
};

}