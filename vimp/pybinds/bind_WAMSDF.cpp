// pybinds/bind_wam.cpp
#include <pybind11/pybind11.h>
#include "pybinds/bind_RobotSDF.h"
#include "robots/WamArmSDFExample.h"  

namespace py  = pybind11;

PYBIND11_MODULE(pybind_WamSDF, m)
{
    m.doc() = "GVIMP bindings for WAM arm";

    bind_RobotSDF<gpmp2::ArmModel, vimp::WamArmSDFExample>(m, "GVIMPWAMArm");
}
