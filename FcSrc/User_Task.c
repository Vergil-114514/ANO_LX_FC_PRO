#include "User_Task.h"
#include "Drv_BSP.h"
#include "Drv_AnoOf.h"
#include "Drv_CarLink.h"
#include "Drv_UwbMini5.h"
#include "Drv_PayloadServo.h"
#include "Ano_Math.h"
#include "AnoPTv8.h"
#include "AnoPTv8FrameFactory.h"
#include "LX_FcFunc.h"
#include "LX_FcState.h"
#include "LX_LowLevelFunc.h"

static _user_task_state userTaskState = USER_TASK_STATE_WAIT_START;
static uint32_t userTaskStateTimeMs;
static uint32_t userTaskMissionTimeMs;
static uint16_t userTaskStableMs;
static uint16_t userTaskActionRetryMs;
static uint8_t userTaskCommandSent;
static uint8_t userTaskStartConsumed;
static uint8_t userTaskStartPending;
static uint8_t userTaskMissionActive;
static uint8_t userTaskType = USER_TASK_TYPE_DROP;
static uint8_t userTaskHomeValid;
static int32_t userTaskHomeUwbX;
static int32_t userTaskHomeUwbY;
static float userTaskVelocityX;
static float userTaskVelocityY;
static uint8_t userTaskStateReported;
static uint8_t userTaskLastOfUpdateCnt;
static uint8_t userTaskLastAltUpdateCnt;
static uint32_t userTaskLastVelUpdateCnt;
static uint16_t userTaskOfAgeMs;
static uint16_t userTaskAltAgeMs;
static uint16_t userTaskVelAgeMs;
static uint8_t userTaskVelReceived;

static void UserTask_SetZeroTarget(void)
{
    rt_tar.st_data.rol = 0;
    rt_tar.st_data.pit = 0;
    rt_tar.st_data.thr = 0;
    rt_tar.st_data.yaw_dps = 0;
    rt_tar.st_data.vel_x = 0;
    rt_tar.st_data.vel_y = 0;
    rt_tar.st_data.vel_z = 0;
}

static void UserTask_ReportState(void)
{
    const char *stateText;

    switch (userTaskState)
    {
    case USER_TASK_STATE_WAIT_START:
        stateText = "AUTO: WAIT_START";
        break;
    case USER_TASK_STATE_PREPARE:
        stateText = "AUTO: PREPARE";
        break;
    case USER_TASK_STATE_UNLOCK:
        stateText = "AUTO: UNLOCK";
        break;
    case USER_TASK_STATE_TAKEOFF:
        stateText = "AUTO: TAKEOFF";
        break;
    case USER_TASK_STATE_HOVER:
        stateText = "AUTO: HOVER_3S";
        break;
    case USER_TASK_STATE_TRACK_MODE:
        stateText = "AUTO: TRACK_MODE";
        break;
    case USER_TASK_STATE_ACQUIRE_CAR:
        stateText = "AUTO: ACQUIRE_CAR";
        break;
    case USER_TASK_STATE_DROP_DESCEND:
        stateText = "AUTO: DROP_DESCEND";
        break;
    case USER_TASK_STATE_DROP_DELAY:
        stateText = "AUTO: DROP_DELAY";
        break;
    case USER_TASK_STATE_DROP_RELEASE:
        stateText = "AUTO: DROP_RELEASE";
        break;
    case USER_TASK_STATE_CLIMB_CRUISE:
        stateText = "AUTO: CLIMB_CRUISE";
        break;
    case USER_TASK_STATE_CAR_LAND_DESCEND:
        stateText = "AUTO: CAR_LAND_DESCEND";
        break;
    case USER_TASK_STATE_CAR_LOCK:
        stateText = "AUTO: CAR_LOCK";
        break;
    case USER_TASK_STATE_CAR_DWELL:
        stateText = "AUTO: CAR_DWELL_5S";
        break;
    case USER_TASK_STATE_RETAKEOFF_MODE:
        stateText = "AUTO: RETAKEOFF_MODE";
        break;
    case USER_TASK_STATE_RETAKEOFF_UNLOCK:
        stateText = "AUTO: RETAKEOFF_UNLOCK";
        break;
    case USER_TASK_STATE_RETAKEOFF_TAKEOFF:
        stateText = "AUTO: RETAKEOFF_TAKEOFF";
        break;
    case USER_TASK_STATE_RETAKEOFF_TRACK_MODE:
        stateText = "AUTO: RETAKEOFF_TRACK";
        break;
    case USER_TASK_STATE_WAIT_CAR_A:
        stateText = "AUTO: WAIT_CAR_A";
        break;
    case USER_TASK_STATE_RETURN_H:
        stateText = "AUTO: RETURN_H";
        break;
    case USER_TASK_STATE_LAND_H:
        stateText = "AUTO: LAND_H";
        break;
    case USER_TASK_STATE_LAND_FAILSAFE:
        stateText = "AUTO: LAND_FAILSAFE";
        break;
    case USER_TASK_STATE_COMPLETE:
        stateText = "AUTO: COMPLETE";
        break;
    default:
        stateText = "AUTO: UNKNOWN";
        break;
    }

    AnoPTv8SendStr(LT_D_SWJ, ANOPTV8DEVID_SWJ, ANOLOGCOLOR_GREEN, stateText);
}

static void UserTask_SetState(const _user_task_state state)
{
    if ((userTaskState == state) && (userTaskStateReported != 0U))
    {
        return;
    }

    userTaskState = state;
    userTaskStateTimeMs = 0U;
    userTaskStableMs = 0U;
    userTaskActionRetryMs = 0U;
    userTaskCommandSent = 0U;
    UserTask_ReportState();
    userTaskStateReported = 1U;
}

static int32_t UserTask_Abs32(const int32_t value)
{
    return (value >= 0) ? value : -value;
}

static void UserTask_UpdateSensorAges(void)
{
    if (ano_of.of_update_cnt != userTaskLastOfUpdateCnt)
    {
        userTaskLastOfUpdateCnt = ano_of.of_update_cnt;
        userTaskOfAgeMs = 0U;
    }
    else if (userTaskOfAgeMs <= (0xFFFFU - 20U))
    {
        userTaskOfAgeMs += 20U;
    }
    else
    {
        userTaskOfAgeMs = 0xFFFFU;
    }

    if (ano_of.alt_update_cnt != userTaskLastAltUpdateCnt)
    {
        userTaskLastAltUpdateCnt = ano_of.alt_update_cnt;
        userTaskAltAgeMs = 0U;
    }
    else if (userTaskAltAgeMs <= (0xFFFFU - 20U))
    {
        userTaskAltAgeMs += 20U;
    }
    else
    {
        userTaskAltAgeMs = 0xFFFFU;
    }

    if (fc_vel_update_count != userTaskLastVelUpdateCnt)
    {
        userTaskLastVelUpdateCnt = fc_vel_update_count;
        userTaskVelAgeMs = 0U;
        userTaskVelReceived = 1U;
    }
    else if (userTaskVelAgeMs <= (0xFFFFU - 20U))
    {
        userTaskVelAgeMs += 20U;
    }
    else
    {
        userTaskVelAgeMs = 0xFFFFU;
    }
}

static uint8_t UserTask_UwbFresh(void)
{
    return (UwbMini5.filter_valid != 0U) &&
           (UwbMini5.filter_age_ms <= UWB_DATA_TIMEOUT_MS);
}

static uint8_t UserTask_VelocityFresh(void)
{
    return (userTaskVelReceived != 0U) &&
           (userTaskVelAgeMs <= AUTO_IMU_VEL_DATA_TIMEOUT_MS);
}

static uint8_t UserTask_SensorsReady(void)
{
    return (UserTask_UwbFresh() != 0U) &&
           (CarLink.valid != 0U) &&
           (CarLink.age_ms <= CAR_LINK_TIMEOUT_MS) &&
           (ano_of.work_sta != 0U) &&
           (ano_of.of1_sta != 0U) &&
           (ano_of.alt_update_cnt != 0U) &&
           (userTaskOfAgeMs <= AUTO_TOF_DATA_TIMEOUT_MS) &&
           (userTaskAltAgeMs <= AUTO_TOF_DATA_TIMEOUT_MS);
}

static uint8_t UserTask_StartRequested(void)
{
#if CAR_LINK_PORT == CAR_LINK_PORT_NONE
    return 0U;
#else
    return (CarLink.valid != 0U) &&
           ((CarLink.flags & CAR_LINK_FLAG_START) != 0U) &&
           ((CarLink.flags & CAR_LINK_FLAG_ABORT) == 0U);
#endif
}

static uint8_t UserTask_Aborted(void)
{
#if CAR_LINK_PORT == CAR_LINK_PORT_NONE
    return 0U;
#else
    return CarLink.abort_pending;
#endif
}

static uint8_t UserTask_AltitudeValid(void)
{
    return (ano_of.work_sta != 0U) &&
           (ano_of.alt_update_cnt != 0U) &&
           (userTaskAltAgeMs <= AUTO_TOF_DATA_TIMEOUT_MS);
}

static uint8_t UserTask_TakeoffStable(void)
{
    const int32_t altitudeError = (int32_t)AUTO_CRUISE_ALT_CM - (int32_t)ano_of.of_alt_cm;

    return (UserTask_AltitudeValid() != 0U) &&
           (UserTask_VelocityFresh() != 0U) &&
           (UserTask_Abs32(altitudeError) <= (int32_t)AUTO_ALTITUDE_TOL_CM) &&
           (UserTask_Abs32((int32_t)fc_vel.st_data.vel_z) <= 20);
}

static uint8_t UserTask_TargetAligned(const int32_t targetX,
                                      const int32_t targetY,
                                      const uint16_t targetAltitudeCm)
{
    const int32_t errorX = UwbMini5.filtered_position_mm[0] - targetX;
    const int32_t errorY = UwbMini5.filtered_position_mm[1] - targetY;
    const int32_t altitudeError = (int32_t)ano_of.of_alt_cm - (int32_t)targetAltitudeCm;

    if (UserTask_SensorsReady() == 0U)
    {
        return 0U;
    }

    return (UserTask_VelocityFresh() != 0U) &&
           (UserTask_Abs32(errorX) <= (int32_t)AUTO_ALIGN_TOL_MM) &&
           (UserTask_Abs32(errorY) <= (int32_t)AUTO_ALIGN_TOL_MM) &&
           (UserTask_Abs32(altitudeError) <= (int32_t)AUTO_ALTITUDE_TOL_CM) &&
           (UserTask_Abs32((int32_t)fc_vel.st_data.vel_z) <= 20);
}

static uint8_t UserTask_CarContactAligned(void)
{
    const int32_t errorX = UwbMini5.filtered_position_mm[0] - UWB_TARGET_OFFSET_X_MM;
    const int32_t errorY = UwbMini5.filtered_position_mm[1] - UWB_TARGET_OFFSET_Y_MM;

    if ((UserTask_SensorsReady() == 0U) ||
        (UserTask_VelocityFresh() == 0U))
    {
        return 0U;
    }

    return (UserTask_Abs32(errorX) <= (int32_t)AUTO_ALIGN_TOL_MM) &&
           (UserTask_Abs32(errorY) <= (int32_t)AUTO_ALIGN_TOL_MM) &&
           (ano_of.of_alt_cm <= AUTO_LAND_CONTACT_ALT_CM) &&
           (UserTask_Abs32((int32_t)fc_vel.st_data.vel_z) <= 20);
}

static uint8_t UserTask_CarContactStable(void)
{
    if (UserTask_CarContactAligned() != 0U)
    {
        if (userTaskStableMs < AUTO_ALIGN_STABLE_MS)
        {
            userTaskStableMs += 20U;
        }
        return (userTaskStableMs >= AUTO_ALIGN_STABLE_MS);
    }

    userTaskStableMs = 0U;
    return 0U;
}

static uint8_t UserTask_TargetStable(const int32_t targetX,
                                     const int32_t targetY,
                                     const uint16_t targetAltitudeCm)
{
    if (UserTask_TargetAligned(targetX, targetY, targetAltitudeCm) != 0U)
    {
        if (userTaskStableMs < AUTO_ALIGN_STABLE_MS)
        {
            userTaskStableMs += 20U;
        }
        return (userTaskStableMs >= AUTO_ALIGN_STABLE_MS);
    }

    userTaskStableMs = 0U;
    return 0U;
}

static void UserTask_LimitHorizontalSpeed(float *velocityX, float *velocityY)
{
    const float maxSpeed = 30.0f;
    const float speed = my_2_norm(*velocityX, *velocityY);

    if (speed > maxSpeed)
    {
        const float scale = maxSpeed / speed;
        *velocityX *= scale;
        *velocityY *= scale;
    }
}

static void UserTask_LimitVelocity(const float desired, float *current)
{
    const float maxStep = 50.0f * 0.02f;

    *current += LIMIT(desired - *current, -maxStep, maxStep);
}

static void UserTask_UpdateTarget(const int32_t targetX,
                                  const int32_t targetY,
                                  const uint16_t targetAltitudeCm)
{
    const float positionX = (float)UwbMini5.filtered_position_mm[0] * 0.1f;
    const float positionY = (float)UwbMini5.filtered_position_mm[1] * 0.1f;
    const float errorCarX = ((float)targetY * 0.1f) - positionY;
    const float errorCarY = positionX - ((float)targetX * 0.1f);
    int32_t yawX100 = CarLink.yaw_x100 % 36000L;
    float errorFcX;
    float errorFcY;
    float desiredX;
    float desiredY;
    float desiredZ;
    float yawRad;
    float cosYaw;
    float sinYaw;

    if (yawX100 < 0L)
    {
        yawX100 += 36000L;
    }

    yawRad = (float)yawX100 * 0.01f * RAD_PER_DEG;
    cosYaw = my_cos(yawRad);
    sinYaw = (float)my_sin(yawRad);
    errorFcX = cosYaw * errorCarX - sinYaw * errorCarY;
    errorFcY = sinYaw * errorCarX + cosYaw * errorCarY;

    if (errorFcX > 3.0f || errorFcX < -3.0f)
    {
        desiredX = errorFcX * 0.5f;
    }
    else
    {
        desiredX = 0.0f;
    }
    if (errorFcY > 3.0f || errorFcY < -3.0f)
    {
        desiredY = errorFcY * 0.5f;
    }
    else
    {
        desiredY = 0.0f;
    }

    UserTask_LimitHorizontalSpeed(&desiredX, &desiredY);
    UserTask_LimitVelocity(desiredX, &userTaskVelocityX);
    UserTask_LimitVelocity(desiredY, &userTaskVelocityY);
    UserTask_LimitHorizontalSpeed(&userTaskVelocityX, &userTaskVelocityY);

    if (UserTask_AltitudeValid() != 0U)
    {
        desiredZ = ((float)targetAltitudeCm - (float)ano_of.of_alt_cm) * AUTO_VERTICAL_SPEED_KP;
        if (desiredZ > AUTO_MAX_VERTICAL_SPEED_CMPS)
        {
            desiredZ = AUTO_MAX_VERTICAL_SPEED_CMPS;
        }
        else if (desiredZ < -AUTO_MAX_VERTICAL_SPEED_CMPS)
        {
            desiredZ = -AUTO_MAX_VERTICAL_SPEED_CMPS;
        }
    }
    else
    {
        desiredZ = 0.0f;
    }

    rt_tar.st_data.rol = 0;
    rt_tar.st_data.pit = 0;
    rt_tar.st_data.thr = 0;
    rt_tar.st_data.yaw_dps = 0;
    rt_tar.st_data.vel_x = (int16_t)userTaskVelocityX;
    rt_tar.st_data.vel_y = (int16_t)userTaskVelocityY;
    rt_tar.st_data.vel_z = (int16_t)desiredZ;
}

static void UserTask_ResetTargetVelocity(void)
{
    userTaskVelocityX = 0.0f;
    userTaskVelocityY = 0.0f;
}

static void UserTask_EnterLandFailsafe(void)
{
    if ((userTaskState == USER_TASK_STATE_LAND_FAILSAFE) ||
        (userTaskState == USER_TASK_STATE_COMPLETE))
    {
        return;
    }

    userTaskMissionActive = 0U;
    UserTask_ResetTargetVelocity();
    UserTask_SetState(USER_TASK_STATE_LAND_FAILSAFE);
}

static void UserTask_EnterComplete(void)
{
    userTaskMissionActive = 0U;
    userTaskStartPending = 0U;
    UserTask_ResetTargetVelocity();
    UserTask_SetState(USER_TASK_STATE_COMPLETE);
}

uint8_t UserTask_IsAutoControlActive(void)
{
    return 1U;
}

uint8_t UserTask_GetState(void)
{
    return (uint8_t)userTaskState;
}

uint8_t UserTask_GetType(void)
{
    return userTaskType;
}

void UserTask_OneKeyCmd(void)
{
    UserTask_UpdateSensorAges();

    if (userTaskStateReported == 0U)
    {
        UserTask_SetState(USER_TASK_STATE_WAIT_START);
    }

    if (userTaskStateTimeMs < 0xFFFFFFFFUL - 20UL)
    {
        userTaskStateTimeMs += 20U;
    }
    if ((userTaskMissionActive != 0U) &&
        (userTaskState != USER_TASK_STATE_COMPLETE) &&
        (userTaskState != USER_TASK_STATE_LAND_FAILSAFE) &&
        (userTaskMissionTimeMs < 0xFFFFFFFFUL - 20UL))
    {
        userTaskMissionTimeMs += 20U;
    }

    if ((userTaskMissionActive != 0U) &&
        (userTaskMissionTimeMs >= AUTO_TASK_TIMEOUT_MS))
    {
        UserTask_EnterLandFailsafe();
    }

    if ((UserTask_Aborted() != 0U) || (EmergencyStopESC != 0U))
    {
        if ((userTaskState != USER_TASK_STATE_WAIT_START) &&
            (userTaskState != USER_TASK_STATE_LAND_FAILSAFE) &&
            (userTaskState != USER_TASK_STATE_COMPLETE))
        {
            UserTask_EnterLandFailsafe();
        }
    }

    switch (userTaskState)
    {
    case USER_TASK_STATE_WAIT_START:
        UserTask_SetZeroTarget();
        LX_Change_Mode(3U);

        if (fc_sta.unlock_sta != 0U)
        {
            FC_Lock();
        }

        if (UserTask_Aborted() != 0U)
        {
            userTaskStartPending = 0U;
            userTaskMissionActive = 0U;
            userTaskStartConsumed = 1U;
            userTaskHomeValid = 0U;
            CarLink.abort_pending = 0U;
        }
        else if (UserTask_StartRequested() == 0U)
        {
            userTaskStartConsumed = 0U;
        }
        else if ((userTaskStartConsumed == 0U) &&
                 (userTaskStartPending == 0U) &&
                 (UserTask_UwbFresh() != 0U))
        {
            userTaskStartConsumed = 1U;
            userTaskStartPending = 1U;
            userTaskMissionActive = 1U;
            userTaskMissionTimeMs = 0U;
            userTaskType = ((CarLink.flags & CAR_LINK_FLAG_TASK_DYNAMIC) != 0U) ?
                           USER_TASK_TYPE_DYNAMIC : USER_TASK_TYPE_DROP;
            userTaskHomeValid = 1U;
            userTaskHomeUwbX = UwbMini5.filtered_position_mm[0];
            userTaskHomeUwbY = UwbMini5.filtered_position_mm[1];
            CarLink.car_at_a_pending = 0U;
        }

        if ((userTaskStartPending != 0U) &&
            (userTaskHomeValid != 0U) &&
            (fc_sta.fc_mode_sta == 3U) &&
            (fc_sta.unlock_sta == 0U) &&
            (EmergencyStopESC == 0U) &&
            (UserTask_SensorsReady() != 0U))
        {
            UserTask_SetState(USER_TASK_STATE_PREPARE);
        }
        break;

    case USER_TASK_STATE_PREPARE:
        UserTask_SetZeroTarget();
        if ((fc_sta.fc_mode_sta != 3U) ||
            (fc_sta.unlock_sta != 0U) ||
            (EmergencyStopESC != 0U) ||
            (UserTask_SensorsReady() == 0U) ||
            (userTaskHomeValid == 0U))
        {
            UserTask_SetState(USER_TASK_STATE_WAIT_START);
        }
        else
        {
            UserTask_SetState(USER_TASK_STATE_UNLOCK);
            userTaskActionRetryMs = 1000U;
        }
        break;

    case USER_TASK_STATE_UNLOCK:
        UserTask_SetZeroTarget();
        if (fc_sta.unlock_sta != 0U)
        {
            UserTask_SetState(USER_TASK_STATE_TAKEOFF);
        }
        else if (userTaskActionRetryMs < 1000U)
        {
            userTaskActionRetryMs += 20U;
        }
        else
        {
            userTaskActionRetryMs = 0U;
            FC_Unlock();
        }
        break;

    case USER_TASK_STATE_TAKEOFF:
        UserTask_SetZeroTarget();
        if (userTaskCommandSent == 0U)
        {
            if (OneKey_Takeoff(AUTO_CRUISE_ALT_CM) != 0U)
            {
                userTaskCommandSent = 1U;
                userTaskStateTimeMs = 0U;
            }
        }
        if (UserTask_TakeoffStable() != 0U)
        {
            if (userTaskStableMs < 500U)
            {
                userTaskStableMs += 20U;
            }
            if (userTaskStableMs >= 500U)
            {
                UserTask_SetState(USER_TASK_STATE_HOVER);
            }
        }
        else
        {
            userTaskStableMs = 0U;
        }
        if ((userTaskCommandSent != 0U) && (userTaskStateTimeMs > 15000U))
        {
            UserTask_EnterLandFailsafe();
        }
        break;

    case USER_TASK_STATE_HOVER:
        UserTask_SetZeroTarget();
        if (userTaskStateTimeMs >= 3000U)
        {
            UserTask_SetState(USER_TASK_STATE_TRACK_MODE);
        }
        break;

    case USER_TASK_STATE_TRACK_MODE:
        UserTask_SetZeroTarget();
        if (fc_sta.fc_mode_sta == 2U)
        {
            UserTask_SetState(USER_TASK_STATE_ACQUIRE_CAR);
        }
        else
        {
            LX_Change_Mode(2U);
        }
        break;

    case USER_TASK_STATE_ACQUIRE_CAR:
        UserTask_UpdateTarget(UWB_TARGET_OFFSET_X_MM,
                              UWB_TARGET_OFFSET_Y_MM,
                              AUTO_CRUISE_ALT_CM);
        if (UserTask_TargetStable(UWB_TARGET_OFFSET_X_MM,
                                  UWB_TARGET_OFFSET_Y_MM,
                                  AUTO_CRUISE_ALT_CM) != 0U)
        {
            if (userTaskType == USER_TASK_TYPE_DYNAMIC)
            {
                UserTask_SetState(USER_TASK_STATE_CAR_LAND_DESCEND);
            }
            else
            {
                UserTask_SetState(USER_TASK_STATE_DROP_DESCEND);
            }
        }
        break;

    case USER_TASK_STATE_DROP_DESCEND:
        UserTask_UpdateTarget(UWB_TARGET_OFFSET_X_MM,
                              UWB_TARGET_OFFSET_Y_MM,
                              DROP_ALT_CM);
        if (UserTask_TargetStable(UWB_TARGET_OFFSET_X_MM,
                                  UWB_TARGET_OFFSET_Y_MM,
                                  DROP_ALT_CM) != 0U)
        {
            UserTask_SetState(USER_TASK_STATE_DROP_DELAY);
        }
        break;

    case USER_TASK_STATE_DROP_DELAY:
        UserTask_UpdateTarget(UWB_TARGET_OFFSET_X_MM,
                              UWB_TARGET_OFFSET_Y_MM,
                              DROP_ALT_CM);
        if (UserTask_TargetAligned(UWB_TARGET_OFFSET_X_MM,
                                   UWB_TARGET_OFFSET_Y_MM,
                                   DROP_ALT_CM) == 0U)
        {
            UserTask_SetState(USER_TASK_STATE_DROP_DESCEND);
        }
        else if (userTaskStateTimeMs >= PAYLOAD_RELEASE_DELAY_MS)
        {
            UserTask_SetState(USER_TASK_STATE_DROP_RELEASE);
        }
        break;

    case USER_TASK_STATE_DROP_RELEASE:
        UserTask_UpdateTarget(UWB_TARGET_OFFSET_X_MM,
                              UWB_TARGET_OFFSET_Y_MM,
                              DROP_ALT_CM);
        if (UserTask_TargetAligned(UWB_TARGET_OFFSET_X_MM,
                                   UWB_TARGET_OFFSET_Y_MM,
                                   DROP_ALT_CM) == 0U)
        {
            UserTask_SetState(USER_TASK_STATE_DROP_DESCEND);
        }
        else
        {
            if (DrvPayloadServoIsReleased() == 0U)
            {
                DrvPayloadServoRelease();
            }
            UserTask_SetState(USER_TASK_STATE_CLIMB_CRUISE);
        }
        break;

    case USER_TASK_STATE_CLIMB_CRUISE:
        UserTask_UpdateTarget(UWB_TARGET_OFFSET_X_MM,
                              UWB_TARGET_OFFSET_Y_MM,
                              AUTO_CRUISE_ALT_CM);
        if (UserTask_TargetStable(UWB_TARGET_OFFSET_X_MM,
                                  UWB_TARGET_OFFSET_Y_MM,
                                  AUTO_CRUISE_ALT_CM) != 0U)
        {
            UserTask_SetState(USER_TASK_STATE_WAIT_CAR_A);
        }
        break;

    case USER_TASK_STATE_CAR_LAND_DESCEND:
        UserTask_UpdateTarget(UWB_TARGET_OFFSET_X_MM,
                              UWB_TARGET_OFFSET_Y_MM,
                              AUTO_LAND_CONTACT_ALT_CM);
        if (UserTask_CarContactStable() != 0U)
        {
            UserTask_SetState(USER_TASK_STATE_CAR_LOCK);
        }
        break;

    case USER_TASK_STATE_CAR_LOCK:
        UserTask_UpdateTarget(UWB_TARGET_OFFSET_X_MM,
                              UWB_TARGET_OFFSET_Y_MM,
                              AUTO_LAND_CONTACT_ALT_CM);
        if (UserTask_CarContactAligned() == 0U)
        {
            userTaskActionRetryMs = 0U;
            UserTask_SetState(USER_TASK_STATE_CAR_LAND_DESCEND);
        }
        else if (fc_sta.unlock_sta == 0U)
        {
            UserTask_SetState(USER_TASK_STATE_CAR_DWELL);
        }
        else if (userTaskActionRetryMs >= 500U)
        {
            userTaskActionRetryMs = 0U;
            FC_Lock();
        }
        else
        {
            userTaskActionRetryMs += 20U;
        }
        break;

    case USER_TASK_STATE_CAR_DWELL:
        UserTask_SetZeroTarget();
        if (userTaskStateTimeMs >= AUTO_CAR_DWELL_MS)
        {
            UserTask_SetState(USER_TASK_STATE_RETAKEOFF_MODE);
        }
        break;

    case USER_TASK_STATE_RETAKEOFF_MODE:
        UserTask_SetZeroTarget();
        LX_Change_Mode(3U);
        if (fc_sta.fc_mode_sta == 3U)
        {
            UserTask_SetState(USER_TASK_STATE_RETAKEOFF_UNLOCK);
            userTaskActionRetryMs = 1000U;
        }
        break;

    case USER_TASK_STATE_RETAKEOFF_UNLOCK:
        UserTask_SetZeroTarget();
        if (fc_sta.unlock_sta != 0U)
        {
            UserTask_SetState(USER_TASK_STATE_RETAKEOFF_TAKEOFF);
        }
        else if (userTaskActionRetryMs < 1000U)
        {
            userTaskActionRetryMs += 20U;
        }
        else
        {
            userTaskActionRetryMs = 0U;
            FC_Unlock();
        }
        break;

    case USER_TASK_STATE_RETAKEOFF_TAKEOFF:
        UserTask_SetZeroTarget();
        if (userTaskCommandSent == 0U)
        {
            if (OneKey_Takeoff(AUTO_CRUISE_ALT_CM) != 0U)
            {
                userTaskCommandSent = 1U;
                userTaskStateTimeMs = 0U;
            }
        }
        if (UserTask_TakeoffStable() != 0U)
        {
            if (userTaskStableMs < 500U)
            {
                userTaskStableMs += 20U;
            }
            if (userTaskStableMs >= 500U)
            {
                UserTask_SetState(USER_TASK_STATE_RETAKEOFF_TRACK_MODE);
            }
        }
        else
        {
            userTaskStableMs = 0U;
        }
        if ((userTaskCommandSent != 0U) && (userTaskStateTimeMs > 15000U))
        {
            UserTask_EnterLandFailsafe();
        }
        break;

    case USER_TASK_STATE_RETAKEOFF_TRACK_MODE:
        UserTask_SetZeroTarget();
        if (fc_sta.fc_mode_sta == 2U)
        {
            UserTask_SetState(USER_TASK_STATE_WAIT_CAR_A);
        }
        else
        {
            LX_Change_Mode(2U);
        }
        break;

    case USER_TASK_STATE_WAIT_CAR_A:
        UserTask_SetZeroTarget();
        UserTask_ResetTargetVelocity();
        if (fc_sta.fc_mode_sta != 2U)
        {
            LX_Change_Mode(2U);
        }
        if ((fc_sta.fc_mode_sta == 2U) &&
            (CarLink.car_at_a_pending != 0U) &&
            (UserTask_SensorsReady() != 0U))
        {
            CarLink.car_at_a_pending = 0U;
            UserTask_SetState(USER_TASK_STATE_RETURN_H);
        }
        break;

    case USER_TASK_STATE_RETURN_H:
        UserTask_UpdateTarget(userTaskHomeUwbX,
                              userTaskHomeUwbY,
                              AUTO_CRUISE_ALT_CM);
        if (UserTask_TargetStable(userTaskHomeUwbX,
                                  userTaskHomeUwbY,
                                  AUTO_CRUISE_ALT_CM) != 0U)
        {
            UserTask_SetState(USER_TASK_STATE_LAND_H);
        }
        break;

    case USER_TASK_STATE_LAND_H:
        UserTask_SetZeroTarget();
        if (fc_sta.unlock_sta == 0U)
        {
            UserTask_EnterComplete();
        }
        else if (userTaskActionRetryMs >= 500U)
        {
            userTaskActionRetryMs = 0U;
            OneKey_Land();
        }
        else
        {
            userTaskActionRetryMs += 20U;
        }
        break;

    case USER_TASK_STATE_LAND_FAILSAFE:
        UserTask_SetZeroTarget();
        if (fc_sta.unlock_sta == 0U)
        {
            UserTask_EnterComplete();
        }
        else if (userTaskActionRetryMs >= 500U)
        {
            userTaskActionRetryMs = 0U;
            if (UserTask_AltitudeValid() != 0U && (ano_of.of_alt_cm <= 30U))
            {
                FC_Lock();
            }
            else
            {
                OneKey_Land();
            }
        }
        else
        {
            userTaskActionRetryMs += 20U;
        }
        break;

    case USER_TASK_STATE_COMPLETE:
        UserTask_SetZeroTarget();
        if (fc_sta.unlock_sta != 0U)
        {
            FC_Lock();
        }
        break;

    default:
        UserTask_SetState(USER_TASK_STATE_WAIT_START);
        break;
    }
}
