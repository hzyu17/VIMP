#include <pybind11/pybind11.h>
#include <pybind11/stl.h> 

#include "helpers/ExperimentParams.h"

namespace py = pybind11;


void bindParams(py::module &m) {

    // -----------------------------------------------------------------
    // 1) Expose the *base* class so Python understands inheritance
    //    (we wrap only the bits needed by GVIMPParams' API).
    // -----------------------------------------------------------------
    py::class_<vimp::Params, std::shared_ptr<vimp::Params>>(m, "Params")
        .def("nx",      &vimp::Params::nx)
        .def("nu",      &vimp::Params::nu)
        .def("total_time", &vimp::Params::total_time)
        .def("nt",      &vimp::Params::nt)
        .def("radius",  &vimp::Params::radius);

    // -----------------------------------------------------------------
    // 2) GVIMPParams
    // -----------------------------------------------------------------
    py::class_<vimp::GVIMPParams,             // C++ type
               vimp::Params,                  // <-- base
               std::shared_ptr<vimp::GVIMPParams>>(m, "GVIMPParams")

        // ----- constructor with keyword arguments and defaults ----------
        .def(py::init< int,int,double,int,double,int,
                       double,double,double,double,int,double,double,double,
                       double,int,double,int,
                       const std::string&, const std::string&, double>(),
             py::arg("nx"),
             py::arg("nu"),
             py::arg("total_time"),
             py::arg("n_states"),
             py::arg("coeff_Qc"),
             py::arg("GH_degree"),
             py::arg("sig_obs"),
             py::arg("eps_sdf"),
             py::arg("radius"),
             py::arg("step_size"),
             py::arg("num_iter"),
             py::arg("initial_precision_factor"),
             py::arg("boundary_penalties"),
             py::arg("temperature"),
             py::arg("high_temperature"),
             py::arg("low_temp_iterations"),
             py::arg("stop_err"),
             py::arg("max_n_backtracking"),
             py::arg("map_name") = "map0",
             py::arg("sdf_file") = "",
             py::arg("alpha")    = 1.0)

        // ----- read-only getters ---------------------------------------
        .def_property_readonly("coeff_Qc",                 &vimp::GVIMPParams::coeff_Qc)
        .def_property_readonly("initial_precision_factor", &vimp::GVIMPParams::initial_precision_factor)
        .def_property_readonly("boundary_penalties",       &vimp::GVIMPParams::boundary_penalties)
        .def_property_readonly("temperature",              &vimp::GVIMPParams::temperature)
        .def_property_readonly("high_temperature",         &vimp::GVIMPParams::high_temperature)
        .def_property_readonly("alpha",                    &vimp::GVIMPParams::alpha)
        .def_property_readonly("max_iter_lowtemp",         &vimp::GVIMPParams::max_iter_lowtemp)
        .def_property_readonly("GH_degree",                &vimp::GVIMPParams::GH_degree)

        // ----- setters / mutators --------------------------------------
        .def("set_temperature",               &vimp::GVIMPParams::set_temperature)
        .def("set_high_temperature",          &vimp::GVIMPParams::set_high_temperature)
        .def("set_boundary_penalties",        &vimp::GVIMPParams::set_boundary_penalties)
        .def("update_initial_precision_factor",&vimp::GVIMPParams::update_initial_precision_factor)
        .def("update_boundary_penalties",     &vimp::GVIMPParams::update_boundary_penalties);
}


PYBIND11_MODULE(bind_Params, m) {
    auto core = m.def_submodule("core");
    bindParams(core);
}