### Add source code
target_sources(multi_calib
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/stereo_matching.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/stereo_params.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/stereo_calib.cpp"
)