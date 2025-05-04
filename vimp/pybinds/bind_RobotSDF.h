// Pybind of the RobotSDF class

#pragma once
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include "instances/gvimp/GVIMPRobotSDF.h"        // template
#include "helpers/ExperimentParams.h"     // GVIMPParams

namespace py  = pybind11;

template <class Robot, class RobotSDF>
void bind_RobotSDF(py::module &m, const char *py_name)
{
    using GVIMPRobotSDF  = vimp::GVIMPRobotSDF<Robot, RobotSDF>;
    using Params = vimp::GVIMPParams;

    py::class_<GVIMPRobotSDF, std::shared_ptr<GVIMPRobotSDF>>(m, py_name)
        .def(py::init<>())
        .def(py::init<Params &>(), py::arg("params"))
        .def("robot_sdf", &GVIMPRobotSDF::robot_sdf)
        .def("run_optimization_withtime",
             &GVIMPRobotSDF::run_optimization_withtime,
             py::arg("params"), py::arg("verbose") = true)
        .def("run_optimization_return",
             &GVIMPRobotSDF::run_optimization_return,
             py::arg("params"), py::arg("verbose") = true)
        .def("get_mu_precision", &GVIMPRobotSDF::get_mu_precision);
}
