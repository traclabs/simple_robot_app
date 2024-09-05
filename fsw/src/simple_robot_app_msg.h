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
** File: simple_robot_app_msg.h
**
** Purpose:
**  Define SimpleRobotApp Messages and info.
**  These are the messages that will be translated to their ROS2 equivalents with the BRASH tools
**
** Notes:
**
**
*******************************************************************************/
#ifndef _simple_robot_app_msg_h_
#define _simple_robot_app_msg_h_

/**
 * SimpleRobotApp command codes
 */
#define SIMPLE_ROBOT_APP_NOOP_CC        0
#define SIMPLE_ROBOT_APP_CMD_CC   1

/*************************************************************************/

/*
** Type definition (generic "no arguments" command)
*/
typedef struct 
{
   CFE_MSG_CommandHeader_t CmdHeader;
} SimpleRobotAppNoArgsCmd_t;


typedef struct
{
  float shoulder_pan_joint;
  float shoulder_lift_joint;
  float elbow_joint;
  float wrist_1_joint;
  float wrist_2_joint;
  float wrist_3_joint;
} SimpleRobotAppJointConfig_t;

typedef struct
{
   CFE_MSG_CommandHeader_t CmdHeader;
   SimpleRobotAppJointConfig_t joint_goal;
} SimpleRobotAppCmd_t;

/*
** The following commands all share the "NoArgs" format
**
** They are each given their own type name matching the command name, which
** allows them to change independently in the future without changing the prototype
** of the handler function
*/
typedef SimpleRobotAppNoArgsCmd_t SimpleRobotAppNoopCmd_t;

/*************************************************************************/

typedef struct
{
    uint8 CommandErrorCounter;
    uint8 CommandCounter;
    SimpleRobotAppJointConfig_t joint_state;
} SimpleRobotAppHkTlmPayload_t;

typedef struct
{
    CFE_MSG_TelemetryHeader_t  TlmHeader; /**< \brief Telemetry header */
    SimpleRobotAppHkTlmPayload_t Payload;   /**< \brief Telemetry payload */
} SimpleRobotAppHkTlm_t;


#endif /* _simple_robot_app_msg_h_ */

/************************/
/*  End of File Comment */
/************************/



















