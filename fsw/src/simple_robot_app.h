/*******************************************************************************
**
**      GSC-18128-1, "Core Flight Executive Version 6.7"
**
**      Copyright (c) 2006-2019 United States Government as represented by
**      the Administrator of the National Aeronautics and Space Administration.
**      All Rights Reserved.
**
**      Licensed under the Apache License, Version 2.0 (the "License");
**      you may not use this file except in compliance with the License.
**      You may obtain a copy of the License at
**
**        http://www.apache.org/licenses/LICENSE-2.0
**
**      Unless required by applicable law or agreed to in writing, software
**      distributed under the License is distributed on an "AS IS" BASIS,
**      WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
**      See the License for the specific language governing permissions and
**      limitations under the License.
**
** File: simple_robot_app.h
**
** Purpose:
**   This file is main hdr file for the application.
**
**
*******************************************************************************/

#ifndef _simple_robot_app_h_
#define _simple_robot_app_h_

/*
** Required header files.
*/
#include "cfe.h"
#include "cfe_error.h"
#include "cfe_evs.h"
#include "cfe_sb.h"
#include "cfe_es.h"

#include "simple_robot_app_perfids.h"
#include "simple_robot_app_msgids.h"
#include "simple_robot_app_msg.h"

// #include "simple_robot_app_msgids.h"

/***********************************************************************/
#define SIMPLE_ROBOT_APP_PIPE_DEPTH 32 /* Depth of the Command Pipe for Application */
/************************************************************************
** Type Definitions
*************************************************************************/

/*
** Global Data
*/

typedef struct
{
    /*
    ** Command interface counters...
    */
    uint8 CmdCounter;

    uint32 square_counter;
    uint32 hk_counter;

    //Housekeeping telemetry packet sent back to ground
    SimpleRobotAppTlm_t JointTlm;
    // Command joint goal received from ground
    SimpleRobotAppCmd_t JointCmd;
    
    // Run Status variable used in the main processing loop
    uint32 RunStatus;

    /*
    ** Operational data (not reported in housekeeping)...
    */
    CFE_SB_PipeId_t CommandPipe;

    /*
    ** Initialization data (not reported in housekeeping)...
    */
    char   PipeName[CFE_MISSION_MAX_API_LEN];
    uint16 PipeDepth;

    CFE_EVS_BinFilter_t EventFilters[SIMPLE_ROBOT_APP_EVENT_COUNTS];

} SimpleRobotAppData_t;

/****************************************************************************/
/*
** Local function prototypes.
**
** Note: Except for the entry point (SimpleRobotAppMain), these
**       functions are not called from any other source module.
*/
void  SimpleRobotAppMain(void);

int32 SimpleRobotAppInit(void);

void  SimpleRobotAppProcessCommandPacket(CFE_SB_Buffer_t *SBBufPtr);
void  SimpleRobotAppProcessGroundCommand(CFE_SB_Buffer_t *SBBufPtr);

int32 SimpleRobotAppReportHousekeeping(const CFE_MSG_CommandHeader_t *Msg);
void SimpleRobotAppProcessRobotState(CFE_SB_Buffer_t *SBBufPtr);

int32 SimpleRobotAppNoop(const SimpleRobotAppNoopCmd_t *Msg);
int32 updateRobotCommand(const SimpleRobotAppCmd_t *Msg);

bool SimpleRobotAppVerifyCmdLength(CFE_MSG_Message_t *MsgPtr, size_t ExpectedLength);
void fillJoints(SimpleRobotAppJointConfig_t *_joints, float j0, float j1, float j2, float j3, float j4, float j5);

#endif /* _SIMPLE_ROBOT_APP_h_ */
