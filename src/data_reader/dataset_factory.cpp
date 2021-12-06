#include "calibration/data_reader/dataset_factory.h"

DataReader::unique_ptr getDataReader(const std::string& dataset_type,
    const std::string& dataset_path) {
  if (dataset_type == "euroc") {
    return std::make_unique<EurocReader>(dataset_path);
  }
  // else if {
  // 
  // }
  else {
    std::cerr << "Dataset type " << dataset_type << " not valid" << std::endl;
    std::abort();
  }
}