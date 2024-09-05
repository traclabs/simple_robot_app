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
  SimpleRobotAppJointConfig_t state; /**< Twist the robot is currently using **/
  bool is_robot_moving;
} lastRobotState;

void HighRateControLoop(void);

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  * *  * * * * **/
/* SimpleRobotAppMain() -- Application entry point and main process loop      */
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

void fillJoints(SimpleRobotAppJointConfig_t *_joints, float j0, float j1, float j2, float j3, float j4, float j5)
{
 _joints->shoulder_pan_joint = j0;
 _joints->shoulder_lift_joint = j1;
 _joints->elbow_joint = j2;
 _joints->wrist_1_joint = j3;
 _joints->wrist_2_joint = j4;
 _joints->wrist_3_joint = j5;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  */
/*                                                                            */
/* SimpleRobotAppInit() --  initialization                                    */
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

    // Initialize telemetry data back to ground
    fillJoints(&SimpleRobotAppData.HkTlm.Payload.joint_state, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      
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
        // Command is being received from ground!
        case SIMPLE_ROBOT_APP_CMD_MID:
            SimpleRobotAppProcessGroundCommand(SBBufPtr);
            break;

        // Our app is being asked to send back telemetry data!
        case SIMPLE_ROBOT_APP_SEND_HK_MID:
            SimpleRobotAppReportHousekeeping((CFE_MSG_CommandHeader_t *)SBBufPtr);
            break;

        // Our app receives a pretty fast clock (1000Hz) to perform a control loop
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

        case SIMPLE_ROBOT_APP_CMD_CC:
            if (SimpleRobotAppVerifyCmdLength(&SBBufPtr->Msg, sizeof(SimpleRobotAppCmd_t)))
            {
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
/*  Name:  SimpleRobotAppReportHousekeeping                                   */
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
   SimpleRobotAppData.JointCmd.joint_goal.shoulder_pan_joint = Msg->joint_goal.shoulder_pan_joint;
   SimpleRobotAppData.JointCmd.joint_goal.shoulder_lift_joint = Msg->joint_goal.shoulder_lift_joint;
   SimpleRobotAppData.JointCmd.joint_goal.elbow_joint = Msg->joint_goal.elbow_joint;
   SimpleRobotAppData.JointCmd.joint_goal.wrist_1_joint = Msg->joint_goal.wrist_1_joint;
   SimpleRobotAppData.JointCmd.joint_goal.wrist_2_joint = Msg->joint_goal.wrist_2_joint;
   SimpleRobotAppData.JointCmd.joint_goal.wrist_3_joint = Msg->joint_goal.wrist_3_joint;
            
   CFE_EVS_SendEvent(SIMPLE_ROBOT_APP_COMMANDMODE_INF_EID, CFE_EVS_EventType_INFORMATION, "SimpleRobotApp: Received command %s",
                     SIMPLE_ROBOT_APP_VERSION);

    return CFE_SUCCESS;
    
}

void HighRateControLoop(void) {
    
    float errors[6];
    errors[0] = (SimpleRobotAppData.JointCmd.joint_goal.shoulder_pan_joint - SimpleRobotAppData.HkTlm.Payload.joint_state.shoulder_pan_joint);
    errors[1] = (SimpleRobotAppData.JointCmd.joint_goal.shoulder_pan_joint - SimpleRobotAppData.HkTlm.Payload.joint_state.shoulder_lift_joint);
    errors[2] = (SimpleRobotAppData.JointCmd.joint_goal.elbow_joint - SimpleRobotAppData.HkTlm.Payload.joint_state.elbow_joint);
    errors[3] = (SimpleRobotAppData.JointCmd.joint_goal.wrist_1_joint - SimpleRobotAppData.HkTlm.Payload.joint_state.wrist_1_joint);
    errors[4] = (SimpleRobotAppData.JointCmd.joint_goal.wrist_2_joint - SimpleRobotAppData.HkTlm.Payload.joint_state.wrist_2_joint);
    errors[5] = (SimpleRobotAppData.JointCmd.joint_goal.wrist_3_joint - SimpleRobotAppData.HkTlm.Payload.joint_state.wrist_3_joint);

    // Update state (telemetry) stored. It will be sent back to a lower rate
    // (when a Housekeeping request is received)
    float Kp = 0.01;
    SimpleRobotAppData.HkTlm.Payload.joint_state.shoulder_pan_joint += + Kp * errors[0];
    SimpleRobotAppData.HkTlm.Payload.joint_state.shoulder_lift_joint += + Kp * errors[1];    
    SimpleRobotAppData.HkTlm.Payload.joint_state.elbow_joint += Kp * errors[2];    
    SimpleRobotAppData.HkTlm.Payload.joint_state.wrist_1_joint += Kp * errors[4];        
    SimpleRobotAppData.HkTlm.Payload.joint_state.wrist_2_joint += Kp * errors[5];
    SimpleRobotAppData.HkTlm.Payload.joint_state.wrist_3_joint += Kp * errors[6];
              
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * **/
/*                                                                            */
/* SimpleRobotAppVerifyCmdLength() -- Verify command packet length            */
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
