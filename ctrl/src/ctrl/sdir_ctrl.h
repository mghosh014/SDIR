#ifndef SDRI_CTRL2019_SDIR_CTRL_H
#define SDRI_CTRL2019_SDIR_CTRL_H

#include <Configuration.h>
#include <Trajectory.h>
#include "SixDPos.h"

/**
 * This is the main entry class for the sdir controller. It handles the api for computing robot specific tasks.
 */
class SdirCtrl {
public:
    /**
     * Computes a {@ref SixDPos} from a given {@ref Configuration}.
     *
     * @param config {@ref Configuration} to compute the position
     * @return {@ref SixDPos} for the passed configuration
     */
    SixDPos* get_pos_from_config(Configuration* config);

    /**
     * Computes a set of {@ref Configuration} from a given {@ref SixDPos}.
     *
     * @param pos {@ref SixDPos} to compute the position
     * @return vector containing all possible configurations for a given positions
     */
    vector<Configuration*>* get_config_from_pos(SixDPos* pos);

    /**
     * Computes a trajectory for a ptp movement from a start and target position
     *
     * @param start
     * @param end
     * @return reference of a {@ref Trajectory} for the movement
     */
    Trajectory* move_robot_ptp(SixDPos* start, SixDPos* end);

    /**
     * Computes a trajectory for a ptp movement from a start and target configuration
     *
     * @param start start {@ref Configuration}
     * @param end   target {@ref Configuration}
     * @return reference of a {@ref Trajectory} for the movement
     */
    Trajectory* move_robot_ptp(Configuration* start, Configuration* end);

    /**
     * Computes a trajectory for a lin movement from a start and target position
     *
     * @param start start {@ref SixDPos}
     * @param end   target {@ref SixDPos}
     * @return reference of a {@ref Trajectory} for the movement
     */
    Trajectory* move_robot_lin(SixDPos* start, SixDPos* end);

    /**
     * Computes a trajectory for a lin movement from a start and target configuration
     *
     * @param start start {@ref Configuration}
     * @param end   target {@ref Configuration}
     * @return reference of a {@ref Trajectory} for the movement
     */
    Trajectory* move_robot_lin(Configuration* start, Configuration* end);
};


#endif //SDRI_CTRL2019_SDIR_CTRL_H
