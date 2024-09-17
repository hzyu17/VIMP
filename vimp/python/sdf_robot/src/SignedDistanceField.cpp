/**
 *  @file  SignedDistanceField.cpp
 *  @brief A wrapper class of signed distance function defined in
 *         https://github.com/gtrll/gpmp2/blob/main/gpmp2/obstacle/SignedDistanceField.h
 *  @date  Dec 19, 2023
 **/

// #pragma once

#include <pybind11/pybind11.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <cstring>
#include <tuple>
#include <memory>
#include <stdexcept>

#include <Eigen/Dense>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

namespace py = pybind11;

/**
 * Signed distance field use vector<Matrix> as data type
 * Matrix represent the X (col) & Y (row) dimension, and vector is used to Z
 */
class SignedDistanceField {

public:
  // index and float_index is <row, col, z>
  typedef std::tuple<size_t, size_t, size_t> index;
  typedef std::tuple<double, double, double> float_index;
  typedef std::shared_ptr<SignedDistanceField> shared_ptr;

private:
  Eigen::Vector3f origin_;
  // geometry setting of signed distance field
  size_t field_rows_, field_cols_, field_z_;
  double cell_size_;
  // sdf data
  std::vector<Eigen::MatrixXd> data_;

public:
  /// constructor
  SignedDistanceField() {}

  /// constructor with all data
  SignedDistanceField(const Eigen::Vector3f& origin, double cell_size, const std::vector<Eigen::MatrixXd>& data) :
      origin_(origin), field_rows_(data[0].rows()), field_cols_(data[0].cols()),
      field_z_(data.size()), cell_size_(cell_size), data_(data) {}

  /// constructor with no data, insert the data later
  /// used by matlab wrapper
  SignedDistanceField(const Eigen::Vector3f& origin, double cell_size, size_t field_rows,
      size_t field_cols, size_t field_z) :
      origin_(origin), field_rows_(field_rows), field_cols_(field_cols),
      field_z_(field_z), cell_size_(cell_size), data_(std::vector<Eigen::MatrixXd>(field_z)) {}

  ~SignedDistanceField() {}


  /// insert data matrix to each layer of sdf
  /// @param z_idx the z index of 3-D sdf
  /// @param field_layer matrix of each slice of 3-D sdf, Matrix represent the X (col) & Y (row)
  void initFieldData(size_t z_idx, const Eigen::MatrixXd& field_layer) {
    if (z_idx >= field_z_)
      throw std::runtime_error("[SignedDistanceField] matrix layer out of index");
    data_[z_idx] = field_layer;
  }

  /// give a point, search for signed distance field and (optional) gradient
  /// @param point query position
  /// @return signed distance
  inline double getSignedDistance(const Eigen::Vector3f& point) const {
    const float_index pidx = convertPoint3toCell(point);
    return signed_distance(pidx);
  }

  /// give a point, search for signed distance field and (optional) gradient
  /// @param point query position
  /// @param g returned gradient reference
  /// @return signed distance
  inline Eigen::Vector3f getGradient(const Eigen::Vector3f& point) const {
    const float_index pidx = convertPoint3toCell(point);
    const Eigen::Vector3f g_idx = gradient(pidx);
    // convert gradient of index to gradient of metric unit
    return Eigen::Vector3f(g_idx(1), g_idx(0), g_idx(2)) / cell_size_;
  }


  /// convert between point and cell corrdinate
  inline float_index convertPoint3toCell(const Eigen::Vector3f& point) const {
    // check point range
    if (point.x() < origin_.x() || point.x() > (origin_.x() + (field_cols_-1.0)*cell_size_) ||
        point.y() < origin_.y() || point.y() > (origin_.y() + (field_rows_-1.0)*cell_size_) ||
        point.z() < origin_.z() || point.z() > (origin_.z() + (field_z_-1.0)*cell_size_)) {
        
      throw std::out_of_range("Index out of range");
    }

    const double col = (point.x() - origin_.x()) / cell_size_;
    const double row = (point.y() - origin_.y()) / cell_size_;
    const double z   = (point.z() - origin_.z()) / cell_size_;
    return std::make_tuple(row, col, z);
  }

  inline Eigen::Vector3f convertCelltoPoint3(const float_index& cell) const {
    return origin_ + Eigen::Vector3f(
        std::get<1>(cell) * cell_size_,
        std::get<0>(cell) * cell_size_,
        std::get<2>(cell) * cell_size_);
  }


  /// tri-linear interpolation
  inline double signed_distance(const float_index& idx) const {
    const double lr = floor(std::get<0>(idx)), lc = floor(std::get<1>(idx)), lz = floor(std::get<2>(idx));
    const double hr = lr + 1.0, hc = lc + 1.0, hz = lz + 1.0;
    const size_t lri = static_cast<size_t>(lr), lci = static_cast<size_t>(lc), lzi = static_cast<size_t>(lz),
        hri = static_cast<size_t>(hr), hci = static_cast<size_t>(hc), hzi = static_cast<size_t>(hz);
    return
        (hr-std::get<0>(idx))*(hc-std::get<1>(idx))*(hz-std::get<2>(idx))*signed_distance(lri, lci, lzi) +
        (std::get<0>(idx)-lr)*(hc-std::get<1>(idx))*(hz-std::get<2>(idx))*signed_distance(hri, lci, lzi) +
        (hr-std::get<0>(idx))*(std::get<1>(idx)-lc)*(hz-std::get<2>(idx))*signed_distance(lri, hci, lzi) +
        (std::get<0>(idx)-lr)*(std::get<1>(idx)-lc)*(hz-std::get<2>(idx))*signed_distance(hri, hci, lzi) +
        (hr-std::get<0>(idx))*(hc-std::get<1>(idx))*(std::get<2>(idx)-lz)*signed_distance(lri, lci, hzi) +
        (std::get<0>(idx)-lr)*(hc-std::get<1>(idx))*(std::get<2>(idx)-lz)*signed_distance(hri, lci, hzi) +
        (hr-std::get<0>(idx))*(std::get<1>(idx)-lc)*(std::get<2>(idx)-lz)*signed_distance(lri, hci, hzi) +
        (std::get<0>(idx)-lr)*(std::get<1>(idx)-lc)*(std::get<2>(idx)-lz)*signed_distance(hri, hci, hzi);
  }

  // gradient operator for tri-linear interpolation
  // gradient regrads to float_index
  // not differentiable at index point
  inline Eigen::Vector3f gradient(const float_index& idx) const {
    const double lr = floor(std::get<0>(idx)), lc = floor(std::get<1>(idx)), lz = floor(std::get<2>(idx));
    const double hr = lr + 1.0, hc = lc + 1.0, hz = lz + 1.0;
    const size_t lri = static_cast<size_t>(lr), lci = static_cast<size_t>(lc), lzi = static_cast<size_t>(lz),
        hri = static_cast<size_t>(hr), hci = static_cast<size_t>(hc), hzi = static_cast<size_t>(hz);
    return Eigen::Vector3f(
        (hc-std::get<1>(idx))*(hz-std::get<2>(idx)) * (signed_distance(hri, lci, lzi)-signed_distance(lri, lci, lzi)) +
        (std::get<1>(idx)-lc)*(hz-std::get<2>(idx)) * (signed_distance(hri, hci, lzi)-signed_distance(lri, hci, lzi)) +
        (hc-std::get<1>(idx))*(std::get<2>(idx)-lz) * (signed_distance(hri, lci, hzi)-signed_distance(lri, lci, hzi)) +
        (std::get<1>(idx)-lc)*(std::get<2>(idx)-lz) * (signed_distance(hri, hci, hzi)-signed_distance(lri, hci, hzi)),

        (hr-std::get<0>(idx))*(hz-std::get<2>(idx)) * (signed_distance(lri, hci, lzi)-signed_distance(lri, lci, lzi)) +
        (std::get<0>(idx)-lr)*(hz-std::get<2>(idx)) * (signed_distance(hri, hci, lzi)-signed_distance(hri, lci, lzi)) +
        (hr-std::get<0>(idx))*(std::get<2>(idx)-lz) * (signed_distance(lri, hci, hzi)-signed_distance(lri, lci, hzi)) +
        (std::get<0>(idx)-lr)*(std::get<2>(idx)-lz) * (signed_distance(hri, hci, hzi)-signed_distance(hri, lci, hzi)),

        (hr-std::get<0>(idx))*(hc-std::get<1>(idx)) * (signed_distance(lri, lci, hzi)-signed_distance(lri, lci, lzi)) +
        (std::get<0>(idx)-lr)*(hc-std::get<1>(idx)) * (signed_distance(hri, lci, hzi)-signed_distance(hri, lci, lzi)) +
        (hr-std::get<0>(idx))*(std::get<1>(idx)-lc) * (signed_distance(lri, hci, hzi)-signed_distance(lri, hci, lzi)) +
        (std::get<0>(idx)-lr)*(std::get<1>(idx)-lc) * (signed_distance(hri, hci, hzi)-signed_distance(hri, hci, lzi))
    );
  }

  // access
  inline double signed_distance(size_t r, size_t c, size_t z) const {
    return data_[z](r, c);
  }

  const Eigen::Vector3f& origin() const { return origin_; }
  size_t x_count() const { return field_cols_; }
  size_t y_count() const { return field_rows_; }
  size_t z_count() const { return field_z_; }
  double cell_size() const { return cell_size_; }
  const std::vector<Eigen::MatrixXd>& raw_data() const { return data_; }

  /// print
  void print(const std::string& str = "") const {
    std::cout << str;
    // std::cout << "field origin:     "; origin_.print();
    std::cout << "field resolution: " << cell_size_ << std::endl;
    std::cout << "field size:       " << field_cols_ << " x "
        << field_rows_ << " x " << field_z_ << std::endl;
  }

//   /// save to file
//   void saveSDF(const std::string filename);

//   /// load from file
//   void loadSDF(const std::string filename);

};

PYBIND11_MODULE(libsdf, m) {
    py::class_<SignedDistanceField>(m, "SignedDistanceField")
        .def(py::init<const Eigen::Vector3f&, double, const std::vector<Eigen::MatrixXd>&>())
        .def("origin", &SignedDistanceField::origin)
        .def("getSignedDistance", &SignedDistanceField::getSignedDistance)
        .def("getGradient", &SignedDistanceField::getGradient);
}
