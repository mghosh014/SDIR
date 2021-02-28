#define _USE_MATH_DEFINES

#include "sdir_ctrl.h"
#include "kinematics/direct/fw_kinematics.h"
#include "kinematics/inverse/inverse_kinematics.h"
#include "pathplanner/ptp/ptp.h"
#include "pathplanner/lin/lin.h"
#include <Trajectory.h>

SixDPos* SdirCtrl::get_pos_from_config(Configuration* _cfg)
{
    //ToDo: IMPLEMENT!
    FwKinematics fwKinematics;
    SixDPos* new_pos = fwKinematics.get_fw_kinematics(_cfg);
    return new_pos;
}


vector<Configuration*>* SdirCtrl::get_config_from_pos(SixDPos* pos)
{
    //ToDo: IMPLEMENT!
    InvKinematics invKinematics;
    vector<Configuration*>* new_cfg = invKinematics.get_inv_kinematics(pos);
    return new_cfg;
}


Trajectory* SdirCtrl::move_robot_ptp(SixDPos* start, SixDPos* end)
{
    //ToDo: IMPLEMENT!

    return NULL;
}

Trajectory* SdirCtrl::move_robot_ptp(Configuration* start, Configuration* end)
{
    //ToDo: IMPLEMENT!
    Ptp ptp;
    return ptp.get_ptp_trajectoy(start, end);
}


Trajectory* SdirCtrl::move_robot_lin(SixDPos* start, SixDPos* end)
{
    //ToDo: IMPLEMENT!
    return NULL;
}


Trajectory* SdirCtrl::move_robot_lin(Configuration* start, Configuration* end)
{
    //ToDo: IMPLEMENT!
    Lin lin;
    return lin.get_lin_trajectoy(start, end);
}