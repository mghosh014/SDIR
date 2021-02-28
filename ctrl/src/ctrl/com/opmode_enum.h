#ifndef SDRI_CTRL2019_OPMODE_ENUM_H
#define SDRI_CTRL2019_OPMODE_ENUM_H

/**
 * Operation modes used to define the action initiated from the gui
 * - CFG_2_POS: computes a position from a given configuration
 * - POS_2_CFG: computes a configuration from a given position
 * - PTP: initiate a ptp movement
 * - LIN: initiate a lin movement
 */
enum OpMode{ CFG_2_POS, POS_2_CFG, PTP, PTPSYNC, LIN };


#endif //SDRI_CTRL2019_OPMODE_ENUM_H
