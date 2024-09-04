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
** File: simple_robot_app.c
**
** Purpose:
**   This file contains the source code for the ros App.
**
*******************************************************************************/

/*
** Include Files:
*/
#include "simple_robot_app_events.h"
#include "simple_robot_app_version.h"
#include "simple_robot_app.h"
#include "simple_robot_app_table.h"

#include <string.h>

#include <math.h>

/*
** global data
*/
SimpleRobotAppData_t SimpleRobotAppData;
struct robot_state_st{
  SimpleRobotAppJointState_t state; /**< Twist the robot is currently using **/
  bool is_robot_moving;
} lastRobotState;
bool updated_command;

void HighRateControLoop(void);

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  * *  * * * * **/
/* SimpleRobotAppMain() -- Application entry point and main process loop         */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  * *  * * * * **/
void SimpleRobotAppMain(void)
{
    int32            status;
    CFE_SB_Buffer_t *SBBufPtr;

    /*
    ** Create the first Performance Log entry
    */
    CFE_ES_PerfLogEntry(SIMPLE_ROBOT_APP_PERF_ID);

    /*
    ** Perform application specific initialization
    ** If the Initialization fails, set the RunStatus to
    ** CFE_ES_RunStatus_APP_ERROR and the App will not enter the RunLoop
    */
    status = SimpleRobotAppInit();

    if (status != CFE_SUCCESS)
    {
        SimpleRobotAppData.RunStatus = CFE_ES_RunStatus_APP_ERROR;
    }

    /*
    ** Runloop
    */
    while (CFE_ES_RunLoop(&SimpleRobotAppData.RunStatus) == true)
    {
        /*
        ** Performance Log Exit Stamp
        */
        CFE_ES_PerfLogExit(SIMPLE_ROBOT_APP_PERF_ID);

        /* Pend on receipt of command packet */
        status = CFE_SB_ReceiveBuffer(&SBBufPtr, SimpleRobotAppData.CommandPipe, CFE_SB_PEND_FOREVER);

        if (status == CFE_SUCCESS)
        {
            SimpleRobotAppProcessCommandPacket(SBBufPtr);
        }
        else
        {
            CFE_EVS_SendEvent(SIMPLE_ROBOT_APP_PIPE_ERR_EID, CFE_EVS_EventType_ERROR,
                              "SimpleRobotApp: SB Pipe Read Error, App Will Exit");

            SimpleRobotAppData.RunStatus = CFE_ES_RunStatus_APP_ERROR;
        }

        /*
        ** Performance Log Entry Stamp
        */
        CFE_ES_PerfLogEntry(SIMPLE_ROBOT_APP_PERF_ID);
    }

    /*
    ** Performance Log Exit Stamp
    */
    CFE_ES_PerfLogExit(SIMPLE_ROBOT_APP_PERF_ID);

    CFE_ES_ExitApp(SimpleRobotAppData.RunStatus);

} /* End of SimpleRobotAppMain() */

void fillJoints(SimpleRobotAppJointState_t *_joints, float j0, float j1, float j2, float j3, float j4, float j5, float j6)
{
 _joints->joint_0 = j0;
 _joints->joint_1 = j1;
 _joints->joint_2 = j2;
 _joints->joint_3 = j3;
 _joints->joint_4 = j4;
 _joints->joint_5 = j5;
 _joints->joint_6 = j6;      
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  */
/*                                                                            */
/* SimpleRobotAppInit() --  initialization                                       */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
int32 SimpleRobotAppInit(void)
{
    int32 status;

    SimpleRobotAppData.RunStatus = CFE_ES_RunStatus_APP_RUN;

    /*
    ** Initialize app command execution counters
    */
    SimpleRobotAppData.CmdCounter = 0;
    SimpleRobotAppData.ErrCounter = 0;
    SimpleRobotAppData.square_counter = 0;
    SimpleRobotAppData.hk_counter = 0;

    // updated
    updated_command = false;

    // Initialize telemetry data back to ground
    fillJoints(&SimpleRobotAppData.HkTlm.Payload.state, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      
    SimpleRobotAppData.HkTlm.Payload.is_robot_moving = false;

    /*
    ** Initialize app configuration data
    */
    SimpleRobotAppData.PipeDepth = SIMPLE_ROBOT_APP_PIPE_DEPTH;

    strncpy(SimpleRobotAppData.PipeName, "SIMPLE_ROBOT_APP_PIPE", sizeof(SimpleRobotAppData.PipeName));
    SimpleRobotAppData.PipeName[sizeof(SimpleRobotAppData.PipeName) - 1] = 0;

    /*
    ** Initialize event filter table...
    */
    SimpleRobotAppData.EventFilters[0].EventID = SIMPLE_ROBOT_APP_STARTUP_INF_EID;
    SimpleRobotAppData.EventFilters[0].Mask    = 0x0000;
    SimpleRobotAppData.EventFilters[1].EventID = SIMPLE_ROBOT_APP_COMMAND_ERR_EID;
    SimpleRobotAppData.EventFilters[1].Mask    = 0x0000;
    SimpleRobotAppData.EventFilters[2].EventID = SIMPLE_ROBOT_APP_COMMANDNOP_INF_EID;
    SimpleRobotAppData.EventFilters[2].Mask    = 0x0000;
    SimpleRobotAppData.EventFilters[3].EventID = SIMPLE_ROBOT_APP_COMMANDMODE_INF_EID;
    SimpleRobotAppData.EventFilters[3].Mask    = 0x0000;
    SimpleRobotAppData.EventFilters[4].EventID = SIMPLE_ROBOT_APP_INVALID_MSGID_ERR_EID;
    SimpleRobotAppData.EventFilters[4].Mask    = 0x0000;
    SimpleRobotAppData.EventFilters[5].EventID = SIMPLE_ROBOT_APP_LEN_ERR_EID;
    SimpleRobotAppData.EventFilters[5].Mask    = 0x0000;
    SimpleRobotAppData.EventFilters[6].EventID = SIMPLE_ROBOT_APP_PIPE_ERR_EID;
    SimpleRobotAppData.EventFilters[6].Mask    = 0x0000;

    status = CFE_EVS_Register(SimpleRobotAppData.EventFilters, SIMPLE_ROBOT_APP_EVENT_COUNTS, CFE_EVS_EventFilter_BINARY);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("SimpleRobotApp: Error Registering Events, RC = 0x%08lX\n", (unsigned long)status);
        return (status);
    }

    /*
    ** Initialize housekeeping packet (clear user data area).
    */
    CFE_MSG_Init(&SimpleRobotAppData.HkTlm.TlmHeader.Msg, CFE_SB_ValueToMsgId(SIMPLE_ROBOT_APP_HK_TLM_MID), sizeof(SimpleRobotAppData.HkTlm));
    CFE_MSG_Init(&SimpleRobotAppData.FlightGoal.TlmHeader.Msg, CFE_SB_ValueToMsgId(SIMPLE_ROBOT_APP_ROBOT_CONTROL_MID), sizeof(SimpleRobotAppData.FlightGoal));

    /*
    ** Create Software Bus message pipe.
    */
    status = CFE_SB_CreatePipe(&SimpleRobotAppData.CommandPipe, SimpleRobotAppData.PipeDepth, SimpleRobotAppData.PipeName);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("SimpleRobotApp: Error creating pipe, RC = 0x%08lX\n", (unsigned long)status);
        return (status);
    }

    /*
    ** Subscribe to Housekeeping request commands
    */
    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(SIMPLE_ROBOT_APP_SEND_HK_MID), SimpleRobotAppData.CommandPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("SimpleRobotApp: Error Subscribing to HK request, RC = 0x%08lX\n", (unsigned long)status);
        return (status);
    }

    /*
    ** Subscribe to ground command packets
    */
    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(SIMPLE_ROBOT_APP_CMD_MID), SimpleRobotAppData.CommandPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("SimpleRobotApp: Error Subscribing to Command, RC = 0x%08lX\n", (unsigned long)status);

        return (status);
    }


    /*
    ** Subscribe to flight's robot state data
    */
    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(SIMPLE_ROBOT_APP_ROBOT_STATE_MID), SimpleRobotAppData.CommandPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("SimpleRobotApp: Error Subscribing to Odom data, RC = 0x%08lX\n", (unsigned long)status);

        return (status);
    }

    
    /*
    ** Subscribe to High Rate wakeup for sending robot commands on the flight side
    */
    status = CFE_SB_Subscribe(CFE_SB_ValueToMsgId(SIMPLE_ROBOT_APP_HR_CONTROL_MID), SimpleRobotAppData.CommandPipe);
    if (status != CFE_SUCCESS)
    {
        CFE_ES_WriteToSysLog("SimpleRobotApp: Error Subscribing to HR Wakeup Command, RC = 0x%08lX\n", (unsigned long)status);

        return (status);
    }

    CFE_EVS_SendEvent(SIMPLE_ROBOT_APP_STARTUP_INF_EID, CFE_EVS_EventType_INFORMATION, "SimpleRobotApp Initialized.%s",
                      SIMPLE_ROBOT_APP_VERSION_STRING);

    return (CFE_SUCCESS);

} /* End of SimpleRobotAppInit() */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*  Name:  SimpleRobotAppProcessCommandPacket                                    */
/*                                                                            */
/*  Purpose:                                                                  */
/*     This routine will process any packet that is received on the ros    */
/*     command pipe.                                                          */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
void SimpleRobotAppProcessCommandPacket(CFE_SB_Buffer_t *SBBufPtr)
{
    CFE_SB_MsgId_t MsgId = CFE_SB_INVALID_MSG_ID;

    CFE_MSG_GetMsgId(&SBBufPtr->Msg, &MsgId);
    switch (CFE_SB_MsgIdToValue(MsgId))
    {
        case SIMPLE_ROBOT_APP_CMD_MID:
            SimpleRobotAppProcessGroundCommand(SBBufPtr);
            break;

        case SIMPLE_ROBOT_APP_SEND_HK_MID:
            SimpleRobotAppReportHousekeeping((CFE_MSG_CommandHeader_t *)SBBufPtr);
            break;

        case SIMPLE_ROBOT_APP_ROBOT_STATE_MID:
            SimpleRobotAppProcessRobotState(SBBufPtr);
            break;

        case SIMPLE_ROBOT_APP_HR_CONTROL_MID:
            HighRateControLoop();
            break;
            
        default:
            CFE_EVS_SendEvent(SIMPLE_ROBOT_APP_INVALID_MSGID_ERR_EID, CFE_EVS_EventType_ERROR,
                              "SimpleRobotApp: invalid command packet,MID = 0x%x", (unsigned int)CFE_SB_MsgIdToValue(MsgId));
            break;
    }

    return;

} /* End SimpleRobotAppProcessCommandPacket */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* SimpleRobotAppProcessGroundCommand() -- SimpleRobotApp ground commands          */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
void SimpleRobotAppProcessGroundCommand(CFE_SB_Buffer_t *SBBufPtr)
{
    CFE_MSG_FcnCode_t CommandCode = 0;

    CFE_MSG_GetFcnCode(&SBBufPtr->Msg, &CommandCode);

    OS_printf("SimpleRobotAppProcessGroundCommand() -- we're getting a ground command...%d\n", CommandCode);

    /*
    ** Process "known" SimpleRobotApp ground commands
    */
    switch (CommandCode)
    {
        case SIMPLE_ROBOT_APP_NOOP_CC:
            if (SimpleRobotAppVerifyCmdLength(&SBBufPtr->Msg, sizeof(SimpleRobotAppNoopCmd_t)))
            {
                SimpleRobotAppNoop((SimpleRobotAppNoopCmd_t *)SBBufPtr);
            }

            break;

        case SIMPLE_ROBOT_APP_MOVE_CC:
            if (SimpleRobotAppVerifyCmdLength(&SBBufPtr->Msg, sizeof(SimpleRobotAppCmd_t)))
            {
                OS_printf("Updating robot command!!!!!!!!!!!!!!!");
                updateRobotCommand((SimpleRobotAppCmd_t *)SBBufPtr);
            }

            break;

        /* default case already found during FC vs length test */
        default:
            CFE_EVS_SendEvent(SIMPLE_ROBOT_APP_COMMAND_ERR_EID, CFE_EVS_EventType_ERROR,
                              "Invalid ground command code: CC = %d", CommandCode);
            break;
    }


    return;

} /* End of SimpleRobotAppProcessGroundCommand() */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* SimpleRobotAppProcessRobotState() -- SimpleRobotApp arm state                   */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
void SimpleRobotAppProcessRobotState(CFE_SB_Buffer_t *SBBufPtr)
{
    CFE_MSG_FcnCode_t CommandCode = 0;

    CFE_MSG_GetFcnCode(&SBBufPtr->Msg, &CommandCode);

    // Read
    if (SimpleRobotAppVerifyCmdLength(&SBBufPtr->Msg, sizeof(SimpleRobotAppRobotState_t)))
    {
       SimpleRobotAppRobotState_t* state = (SimpleRobotAppRobotState_t *)SBBufPtr;
       
       // Fill the lastState
       lastRobotState.state = state->state;
       lastRobotState.is_robot_moving = state->is_robot_moving;                     
    }


    return;

} /* End of SimpleRobotAppProcessRobotState() */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*  Name:  SimpleRobotAppReportHousekeeping                                          */
/*                                                                            */
/*  Purpose:                                                                  */
/*         This function is triggered in response to a task telemetry request */
/*         from the housekeeping task. This function will gather the Apps     */
/*         telemetry, packetize it and send it to the housekeeping task via   */
/*         the software bus                                                   */
/* * * * * * * * * * * * * * * * * * * * * * * *  * * * * * * *  * *  * * * * */
int32 SimpleRobotAppReportHousekeeping(const CFE_MSG_CommandHeader_t *Msg)
{    
    /*
    ** Get command execution counters...
    */
    SimpleRobotAppData.HkTlm.Payload.CommandErrorCounter = SimpleRobotAppData.ErrCounter*2;
    SimpleRobotAppData.ErrCounter++;
    SimpleRobotAppData.HkTlm.Payload.CommandCounter      = SimpleRobotAppData.CmdCounter++;

    OS_printf("SimpleRobotAppReportHousekeeping reporting: %d\n", SimpleRobotAppData.HkTlm.Payload.CommandCounter);

 
    CFE_SB_TimeStampMsg(&SimpleRobotAppData.HkTlm.TlmHeader.Msg);
    CFE_SB_TransmitMsg(&SimpleRobotAppData.HkTlm.TlmHeader.Msg, true);

    return CFE_SUCCESS;

} /* End of SimpleRobotAppReportHousekeeping() */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* SimpleRobotAppNoop -- ROS NOOP commands                                          */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
int32 SimpleRobotAppNoop(const SimpleRobotAppNoopCmd_t *Msg)
{
    CFE_EVS_SendEvent(SIMPLE_ROBOT_APP_COMMANDNOP_INF_EID, CFE_EVS_EventType_INFORMATION, "SimpleRobotApp: NOOP command %s",
                      SIMPLE_ROBOT_APP_VERSION);

    return CFE_SUCCESS;
} /* End of SimpleRobotAppNoop */


int32 updateRobotCommand(const SimpleRobotAppCmd_t *Msg)
{
                OS_printf("Updating robot command inside......");

    fillJoints(&SimpleRobotAppData.FlightGoal.goal, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
   
   switch(Msg->pose_id)
   {
     // Open
     case 0:
     {
        fillJoints(&SimpleRobotAppData.FlightGoal.goal, 0.0, 0.0, 0.0, -3.1416, 0.0, 0.0, 0.0);
     } break;
     // All zeros
     case 1:
     {} break;
     // Random
     case 2:
     {
        fillJoints(&SimpleRobotAppData.FlightGoal.goal, 1.0, -1.5, 3.0, -3.2, 0.8, 0.5, -1.0);

     } break;       
   }

    updated_command = true;

    CFE_EVS_SendEvent(SIMPLE_ROBOT_APP_COMMANDMODE_INF_EID, CFE_EVS_EventType_INFORMATION, "SimpleRobotApp: Received command %s",
                      SIMPLE_ROBOT_APP_VERSION);

    return CFE_SUCCESS;
    
}

void HighRateControLoop(void) {
    
    // 1. Publish the twist to State in rosfsw (it is like sending a command to the robot)
    // (we should use another name, telemetry is not supposed to command anything)

    if (updated_command)    
    {
    CFE_SB_TimeStampMsg(&SimpleRobotAppData.FlightGoal.TlmHeader.Msg);
    CFE_SB_TransmitMsg(&SimpleRobotAppData.FlightGoal.TlmHeader.Msg, true);
    updated_command = false;    
    }

 
    
    // 2. Update the telemetry information to be sent back to the ground        
    struct robot_state_st *st = &lastRobotState;

    SimpleRobotAppData.HkTlm.Payload.state = st->state;
    SimpleRobotAppData.HkTlm.Payload.is_robot_moving  = st->is_robot_moving;

    // This data is sent when a Housekeeping request is received, 
    // (usually, at a low rate) so nothing sent here
    //memcpy(&st->joints, &SimpleRobotAppData.HkTlm.Payload.state, sizeof(SimpleRobotAppSSRMS_t) );
    
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* SimpleRobotAppVerifyCmdLength() -- Verify command packet length                   */
/*                                                                            */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
bool SimpleRobotAppVerifyCmdLength(CFE_MSG_Message_t *MsgPtr, size_t ExpectedLength)
{
    bool              result       = true;
    size_t            ActualLength = 0;
    CFE_SB_MsgId_t    MsgId        = CFE_SB_INVALID_MSG_ID;
    CFE_MSG_FcnCode_t FcnCode      = 0;

    CFE_MSG_GetSize(MsgPtr, &ActualLength);

    /*
    ** Verify the command packet length.
    */
    if (ExpectedLength != ActualLength)
    {
        CFE_MSG_GetMsgId(MsgPtr, &MsgId);
        CFE_MSG_GetFcnCode(MsgPtr, &FcnCode);

        CFE_EVS_SendEvent(SIMPLE_ROBOT_APP_LEN_ERR_EID, CFE_EVS_EventType_ERROR,
                          "Invalid Msg length: ID = 0x%X,  CC = %u, Len = %u, Expected = %u",
                          (unsigned int)CFE_SB_MsgIdToValue(MsgId), (unsigned int)FcnCode, (unsigned int)ActualLength,
                          (unsigned int)ExpectedLength);

        result = false;

        SimpleRobotAppData.ErrCounter++;
    }

    return (result);

} /* End of SimpleRobotAppVerifyCmdLength() */
