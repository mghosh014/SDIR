add_library(PTP
            ptp.cpp
        )

target_include_directories(FWKin
        PUBLIC
        "${CMAKE_SOURCE_DIR}/src/ctrl/postypes"
        )