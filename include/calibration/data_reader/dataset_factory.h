#pragma once

#include "calibration/data_reader/DataReader.h"
#include "calibration/data_reader/EurocReader.h"

DataReader::unique_ptr getDataReader(const std::string& dataset_type,
    const std::string& dataset_path);