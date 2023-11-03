/**
 ********************************************************************
 * @file    dji_config_manager.c
 * @brief
 *
 * @copyright (c) 2018 DJI. All rights reserved.
 *
 * All information contained herein is, and remains, the property of DJI.
 * The intellectual and technical concepts contained herein are proprietary
 * to DJI and may be covered by U.S. and foreign patents, patents in process,
 * and protected by trade secret or copyright law.  Dissemination of this
 * information, including but not limited to data and other proprietary
 * material(s) incorporated within the information, in any form, is strictly
 * prohibited without the express written consent of DJI.
 *
 * If you receive this source code without DJI’s authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify DJI of its removal. DJI reserves the right to pursue
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
 *
 *********************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <dji_logger.h>
#include <dji_aircraft_info.h>
#include "dji_config_manager.h"

/* Private constants ---------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private values -------------------------------------------------------------*/
static T_DjiUserInfo s_configManagerUserInfo = {0};
static T_DjiUserLinkConfig s_configManagerLinkInfo = {0};
static bool s_configManagerIsEnable = false;

void DjiUserConfigManager_GetAppInfo(T_DjiUserInfo *userInfo)
{
    memcpy(userInfo, &s_configManagerUserInfo, sizeof(T_DjiUserInfo));
}

void DjiUserConfigManager_GetLinkConfig(T_DjiUserLinkConfig *linkConfig)
{
    memcpy(linkConfig, &s_configManagerLinkInfo, sizeof(T_DjiUserLinkConfig));
}

bool DjiUserConfigManager_IsEnable(void)
{
    return s_configManagerIsEnable;
}

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
