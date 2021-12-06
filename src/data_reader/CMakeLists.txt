### Add source code
target_sources(multi_calib
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/dataset_factory.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/DataReader.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/EurocReader.cpp"
)