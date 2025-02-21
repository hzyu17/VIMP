#include <Eigen/Dense>
#include <vector>
#include <string>
#include <helpers/SerializeEigenMaps.h>

class SignedDistanceField {

public:

  // geometry setting of signed distance field
  Eigen::Vector3d origin_;
  int field_rows_, field_cols_, field_z_;
  double cell_size_;

  std::vector<Eigen::MatrixXd> data_;

public:
  /// constructor
  SignedDistanceField() {}

  SignedDistanceField(const Eigen::Vector3d& origin, double cell_size, int field_rows, int field_cols, int field_z): 
      origin_(origin), field_rows_(field_rows), field_cols_(field_cols),
      field_z_(field_z), cell_size_(cell_size), data_(std::vector<Eigen::MatrixXd>(field_z)) {}

  ~SignedDistanceField() {}


  void initFieldData(int z_idx, const Eigen::MatrixXd& field_layer) {
    if (z_idx >= field_z_)
      throw std::runtime_error("[SignedDistanceField] matrix layer out of index");
    data_[z_idx] = field_layer;
  }

  void loadSDF(const std::string& filename) {
    std::ifstream ifs(filename, std::ios::binary);
    if (!ifs.is_open()) {
        std::cout << "File '" << filename << "' does not exist!" << std::endl;
        return;
    }

    std::string fext = filename.substr(filename.find_last_of(".") + 1);
    if (fext == "xml") {
        cereal::XMLInputArchive ia(ifs);
        ia(*this);
    }
    else if (fext == "bin") {
        cereal::BinaryInputArchive ia(ifs);
        ia(*this);
    }
    else {
        cereal::JSONInputArchive ia(ifs);
        ia(*this);
    }
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

  /** Serialization function */
  // When serialize and deserialize, the data is stored in a binary file
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