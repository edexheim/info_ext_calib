### Add source code
target_sources(multi_calib
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/BackendParams.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/DatabaseThread.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/partition.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/Segment.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/SegmentAccumulator.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/SegmentConstruction.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/SegmentDatabase.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/BatchSegmentDatabase.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/IncrementalSegmentDatabase.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/SegmentId.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/triangulation.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/graph_utils.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/ISAM2Custom.cpp"
)