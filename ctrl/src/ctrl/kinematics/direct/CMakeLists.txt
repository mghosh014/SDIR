add_library(FWKin
            fw_kinematics.cpp
        )

target_include_directories(FWKin
        PUBLIC
        "${CMAKE_SOURCE_DIR}/src/ctrl/postypes"
        )