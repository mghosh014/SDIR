add_library(PosTypes
            Configuration.cpp
            SixDPos.cpp
            TMatrix.cpp
            Trajectory.cpp
        )

target_include_directories(FWKin
        PUBLIC
        "${CMAKE_SOURCE_DIR}/src/ctrl/postypes"
        )