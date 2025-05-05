#include <Eigen/Dense>
#include <vector>
#include <string>
// #include <helpers/MatrixHelper.h>
#include <helpers/SerializeEigenMaps.h>

class SignedDistanceField {

public:
  // index and float_index is <row, col>
  typedef std::tuple<size_t, size_t, size_t> index;
  typedef Eigen::Vector3d float_index;
  typedef std::shared_ptr<SignedDistanceField> shared_ptr;

  // geometry setting of signed distance field
  Eigen::Vector3d origin_;
  int field_rows_, field_cols_, field_z_;
  double cell_size_;

  std::vector<Eigen::MatrixXd> data_;

public:
  /// constructor
  SignedDistanceField() {}

  /// constructor with data
  SignedDistanceField(const Eigen::Vector3d& origin, double cell_size, const std::vector<Eigen::MatrixXd>& data) :
      origin_(origin), field_rows_(data[0].rows()), field_cols_(data[0].cols()), 
      field_z_(data.size()), cell_size_(cell_size), data_(data){}

  SignedDistanceField(const Eigen::Vector3d& origin, double cell_size, int field_rows, int field_cols, int field_z): 
      origin_(origin), field_rows_(field_rows), field_cols_(field_cols),
      field_z_(field_z), cell_size_(cell_size), data_(std::vector<Eigen::MatrixXd>(field_z)) {}

  ~SignedDistanceField() {}


  void initFieldData(int z_idx, const Eigen::MatrixXd& field_layer) {
    if (z_idx >= field_z_)
      throw std::runtime_error("[SignedDistanceField] matrix layer out of index");
    data_[z_idx] = field_layer;
  }

  /// give a point, search for signed distance field and (optional) gradient
  /// return signed distance
  inline double getSignedDistance(const Eigen::MatrixXd& point) const {
    const float_index pidx = convertPoint3toCell(point);
    return signed_distance(pidx);
  }

  /// convert between point and cell corrdinate
  inline float_index convertPoint3toCell(const Eigen::Vector3d& point) const {
    if (point.x() < origin_.x() || point.x() > (origin_.x() + (field_cols_-1.0)*cell_size_) ||
        point.y() < origin_.y() || point.y() > (origin_.y() + (field_rows_-1.0)*cell_size_) ||
        point.z() < origin_.z() || point.z() > (origin_.z() + (field_z_-1.0)*cell_size_)) {
        
      throw std::runtime_error("Querying SDF out of range");
    }

    const double col = (point(0) - origin_(0)) / cell_size_;
    const double row = (point(1) - origin_(1)) / cell_size_;
    const double z   = (point(2) - origin_(2)) / cell_size_;
    return Vector3d{row, col, z};
  }

  inline Eigen::Vector3d convertCelltoPoint2(const float_index& cell) const {
    return origin_ + Eigen::Vector3d(
        cell(1) * cell_size_,
        cell(0) * cell_size_,
        cell(2) * cell_size_);
  }

  /// bilinear interpolation
  inline double signed_distance(const float_index& idx) const {
    const double lr = floor(idx(0)), lc = floor(idx(1)), lz = floor(idx(2));
    const double hr = lr + 1.0, hc = lc + 1.0, hz = lz + 1.0;
    const int lri = static_cast<int>(lr), lci = static_cast<int>(lc), lzi = static_cast<int>(lz), 
              hri = static_cast<int>(hr), hci = static_cast<int>(hc), hzi = static_cast<int>(hz);
    // printf("lri = %d, lci = %d, lzi = %d, hri = %d, hci = %d, hzi = %d\n\n", lri, lci, lzi, hri, hci, hzi);
    return
        (hr-idx(0))*(hc-idx(1))*(hz-idx(2))*signed_distance(lri, lci, lzi) +
        (idx(0)-lr)*(hc-idx(1))*(hz-idx(2))*signed_distance(hri, lci, lzi) +
        (hr-idx(0))*(idx(1)-lc)*(hz-idx(2))*signed_distance(lri, hci, lzi) +
        (idx(0)-lr)*(idx(1)-lc)*(hz-idx(2))*signed_distance(hri, hci, lzi) +
        (hr-idx(0))*(hc-idx(1))*(idx(2)-lz)*signed_distance(lri, lci, hzi) +
        (idx(0)-lr)*(hc-idx(1))*(idx(2)-lz)*signed_distance(hri, lci, hzi) +
        (hr-idx(0))*(idx(1)-lc)*(idx(2)-lz)*signed_distance(lri, hci, hzi) +
        (idx(0)-lr)*(idx(1)-lc)*(idx(2)-lz)*signed_distance(hri, hci, hzi);
  }

  void loadSDF(const std::string& filename) {
    std::ifstream ifs(filename, std::ios::binary);
    if (!ifs.is_open()) {
        std::cout << "File '" << filename << "' does not exist!" << std::endl;
        return;
    }

    auto pos = filename.rfind('.');
    if (pos == std::string::npos) {
      throw std::runtime_error("Cannot detect extension for SDF file: " + filename);
    }
    auto fext = filename.substr(pos + 1);
    std::transform(fext.begin(), fext.end(), fext.begin(), ::tolower);

    std::cout << "Loading SDF File: " << filename << "\n"
              << "  Extension detected: '" << fext << "'\n";

    if (fext == "xml") {
      cereal::XMLInputArchive ia(ifs);
      ia(*this);
    }
    else if (fext == "bin") {
      cereal::BinaryInputArchive ia(ifs);
      ia(*this);
    }
    else {
      throw std::runtime_error("Unsupported SDF extension '" + fext + "'");
    }

    std::cout << "Loading SDF file completed!" << std::endl;

  }

  void saveSDF(const std::string filename) {
    
    std::ofstream ofs(filename.c_str());
    assert(ofs.good());
    std::string fext = filename.substr(filename.find_last_of(".") + 1);

    if (fext == "bin") {
      cereal::BinaryOutputArchive archive(ofs);
      archive(*this);
    }
  }

  // access
  double signed_distance(int r, int c, int z) const {
    return data_[z](r, c);
  }

  const Eigen::Vector3d& origin() const { return origin_; }
  int x_count() const { return field_cols_; }
  int y_count() const { return field_rows_; }
  int z_count() const { return field_z_; }
  double cell_size() const { return cell_size_; }
  const std::vector<Eigen::MatrixXd>& raw_data() const { return data_; }

  /** Serialization function */
  template<class Archive>
  void serialize(Archive& ar) {
    ar(CEREAL_NVP(origin_));
    ar(CEREAL_NVP(field_rows_));
    ar(CEREAL_NVP(field_cols_));
    ar(CEREAL_NVP(field_z_));
    ar(CEREAL_NVP(cell_size_));
    ar(CEREAL_NVP(data_));
  }

};