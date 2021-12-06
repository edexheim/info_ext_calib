#ifndef EUROCREADER_H
#define EUROCREADER_H

#include "calibration/data_reader/DataReader.h"

class EurocReader : public DataReader {

  public:
    EurocReader(std::string path);
    ~EurocReader();

    // inherited virtual functions
    bool initialize() override;
    std::vector<CameraModelType> getCameraModelType() const override;
    std::vector<Eigen::VectorXd> getCalibParams() const override;

  private:

};

#endif