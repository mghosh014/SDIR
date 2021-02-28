add_library(Ctrl
            sdir_ctrl.cpp
        )

add_subdirectory(com)
add_dependencies(Ctrl Com)

add_subdirectory(kinematics/direct)
add_subdirectory(kinematics/inverse)
add_dependencies(Ctrl FWKin IVKin)

add_subdirectory(pathplanner/lin)
add_subdirectory(pathplanner/ptp)
add_dependencies(Ctrl PTP LIN)

add_subdirectory(postypes)
add_dependencies(Ctrl PosTypes)

target_link_libraries(Ctrl Com PosTypes FWKin IVKin PTP LIN)
