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

#define SIMPLE_ROBOT_APP_CMD_MID     (CFE_PLATFORM_CMD_MID_BASE + 0x37)
#define SIMPLE_ROBOT_APP_SEND_HK_MID (CFE_PLATFORM_CMD_MID_BASE + 0x38)
#define SIMPLE_ROBOT_APP_ROBOT_STATE_MID (CFE_PLATFORM_CMD_MID_BASE + 0x39)

#define SIMPLE_ROBOT_APP_HK_TLM_MID      (CFE_PLATFORM_TLM_MID_BASE + 0x36)
#define SIMPLE_ROBOT_APP_ROBOT_CONTROL_MID   (CFE_PLATFORM_TLM_MID_BASE + 0x37)
#define SIMPLE_ROBOT_APP_HR_CONTROL_MID  (CFE_PLATFORM_TLM_MID_BASE + 0x38)
#endif /* _simple_robot_app_msgids_h_ */

/*********************************/
/* End of File Comment           */
/*********************************/
