### Add source code
target_sources(multi_calib
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/info_metrics.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/lin_alg_utils.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/serialization.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/SimParams.cpp"
)