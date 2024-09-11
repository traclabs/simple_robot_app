/************************************************************************
**
**
** File: simple_robot_app_msgids.h
**
** Purpose:
**  Define Simple Robot App Message IDs
**
** Notes:
**
**
*************************************************************************/
#ifndef _simple_robot_app_msgids_h_
#define _simple_robot_app_msgids_h_

#include "cfe_msgids.h"

// (GROUND SIDE) User is sending commands -- ground->cFS
#define SIMPLE_ROBOT_APP_CMD_MID     (CFE_PLATFORM_CMD_MID_BASE + 0x37)
// Housekeeping is requesting data back
#define SIMPLE_ROBOT_APP_SEND_HK_MID (CFE_PLATFORM_CMD_MID_BASE + 0x38)

// (FLIGHT SIDE) Actuator state  robot->cFS
#define SIMPLE_ROBOT_APP_FLIGHT_TLM_MID      (CFE_PLATFORM_CMD_MID_BASE + 0x39)


// (GROUND SIDE) Telemetry data being sent back -- cFS->ground
#define SIMPLE_ROBOT_APP_HK_TLM_MID      (CFE_PLATFORM_TLM_MID_BASE + 0x36)

// (FLIGHT SIDE) Command being sent cFS-> robot
#define SIMPLE_ROBOT_APP_FLIGHT_CMD_MID      (CFE_PLATFORM_TLM_MID_BASE + 0x37)

// High-rate control signal to run control loop
#define SIMPLE_ROBOT_APP_HR_CONTROL_MID  (CFE_PLATFORM_TLM_MID_BASE + 0x39)


#endif /* _simple_robot_app_msgids_h_ */

/*********************************/
/* End of File Comment           */
/*********************************/
