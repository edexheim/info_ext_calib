### Add source code
target_sources(multi_calib
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/Frontend.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/Camera.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/keypoints.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/keyframe_selection/KeyframeSelecter.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/keyframe_selection/EntropyKeyframeSelecter.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/keyframe_selection/FeatureKeyframeSelecter.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/keyframe_selection/MotionKeyframeSelecter.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/FeatureDatabase.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/FeatureTracker.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/Frame.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/FrontendParams.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/OutlierRejecter.cpp"
)