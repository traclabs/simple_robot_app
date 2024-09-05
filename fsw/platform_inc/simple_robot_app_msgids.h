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

// Ground is sending commands
#define SIMPLE_ROBOT_APP_CMD_MID     (CFE_PLATFORM_CMD_MID_BASE + 0x37)
// Housekeeping is requesting data back
#define SIMPLE_ROBOT_APP_SEND_HK_MID (CFE_PLATFORM_CMD_MID_BASE + 0x38)

// Telemetry data being sent back
#define SIMPLE_ROBOT_APP_HK_TLM_MID      (CFE_PLATFORM_TLM_MID_BASE + 0x36)

// High-rate control signal to run control loop
#define SIMPLE_ROBOT_APP_HR_CONTROL_MID  (CFE_PLATFORM_TLM_MID_BASE + 0x39)


#endif /* _simple_robot_app_msgids_h_ */

/*********************************/
/* End of File Comment           */
/*********************************/
