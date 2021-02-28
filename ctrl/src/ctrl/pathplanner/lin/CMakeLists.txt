add_library(LIN
            lin.cpp
        )

target_include_directories(FWKin
        PUBLIC
        "${CMAKE_SOURCE_DIR}/src/ctrl/postypes"
        )