### Add source code
target_sources(multi_calib
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/camera_model_factory.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/Pinhole.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/PinholeModel.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/PinholeKB.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/PinholeRadtan.cpp"
)