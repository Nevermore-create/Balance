#include"ctrl_balancetest.h"
/*
 * ctrl_balance.c
 *
 *  Created on: 2020年11月14日
 *      Author: lx
 */
float accel[3]={0.0f,0.0f,0.0f}; //采集到的三个加速度数据
float angul[3]={0.0f,0.0f,0.0f}; //采集到的三个角速度数据
float filter_angle=0.0f;//滤波之后的角度
float accel_angle=0.0f;//加速度算出的角度
float angul_angle=0.0f;//角速度算出的角度
float ave_angle=0.0f;//初始化时求出的平均角度



/*void CTRL_Init(void)
{
    pitMgr_t::insert(5U, 4U, CTRL_AngCtrl, pitMgr_t::enable);
}*/
void FilterInit(void)//初始化函数
{
    const uint32_t sampleTime = 1024;
    float sum_angle=0.0f;
    for(uint32_t i = 0; i < sampleTime; ++i)
    {
         imu_6050.ReadSensorBlocking();
         imu_6050.Convert(&accel[0], &accel[1], &accel[2],&angul[0],&angul[1],&angul[2]);
         float accelAx=accel[2];
         accelAx = (accelAx> accel_grv - 0.001) ? (accel_grv - 0.001) : accelAx;
         accelAx = (accelAx < - (accel_grv - 0.001)) ? (- (accel_grv - 0.001)) : accelAx;
         accel_angle = ang2rad(- asin(accelAx/ accel_grv));
         sum_angle += accel_angle;
         SDK_DelayAtLeastUs(1000,CLOCK_GetFreq(kCLOCK_CoreSysClk)); // MPU6050更新速度没那么快，连续读取只会得到相同的值。
     }
    ave_angle=sum_angle/((float)sampleTime);
    filter_angle=ave_angle;
    angul_angle=ave_angle;
}
void FilterUpdate(uint32_t updateTime_ms)
{
    float dT = ((float)updateTime_ms) * 0.001f;
    float accelAx = accel[2];
    float angulAx=angul[1];
    //以下为限幅
    accelAx = (accelAx> accel_grv - 0.001) ? (accel_grv - 0.001) : accelAx;
    accelAx = (accelAx < - (accel_grv - 0.001)) ? (- (accel_grv - 0.001)) : accelAx;
    accel_angle = ang2rad(- asin(accelAx/ accel_grv));  //根据加速度所得角度
    angul_angle += angulAx*dT;   //角速度积分所得角度
    filter_angle += (angulAx + ((accel_angle - filter_angle) * Tg)) * dT;  //滤波之后得出的角度
}
void SendAngle(void)
{
    float angle[3]={accel_angle,angul_angle,filter_angle};
    SCHOST_VarUpload(angle,3);   //加速度计算出的、角速度计算出的、滤波之后的角度传到上位机
}

/* ******************** 直立环 ******************** */
int32_t ctrl_angCtrlEn[3] = {1, 0, 1}; ///< 直立环使能

float ctrl_angSet = 20.0f; ///< 机械零点

pidCtrl_t ctrl_angPid =
{
    .kp = 400.0f, .ki = 0.0f, .kd = 1.0f,
    .errCurr = 0.0f, .errIntg = 0.0f, .errDiff = 0.0f, .errPrev = 0.0f,
};
void MENU_DataSetUpTEST(void)
{
    static menu_list_t *control;
    control = MENU_ListConstruct("Control", 20, menu_menuRoot);
    assert(control);
    MENU_ListInsert(menu_menuRoot, MENU_ItemConstruct(menuType, control, "Control", 0, 0));
    {
        MENU_ListInsert(control, MENU_ItemConstruct(varfType, &ctrl_angPid.kp, ".kp", 11, menuItem_data_global));
        MENU_ListInsert(control, MENU_ItemConstruct(varfType, &ctrl_angPid.ki, ".ki", 11, menuItem_data_global));
        MENU_ListInsert(control, MENU_ItemConstruct(varfType, &ctrl_angPid.kd, ".kd", 11, menuItem_data_global));
    }
    //TODO: 在这里添加子菜单和菜单项
}

float ctrl_angPidOutput = 0.0f; ///< 直立环输出

void CTRL_AngCtrl(void *userData)
{
    if(kStatus_Success == imu_6050.ReadSensorBlocking())
    {
        imu_6050.Convert(&accel[0], &accel[1], &accel[2],&angul[0],&angul[1],&angul[2]);
        FilterUpdate(5U);
        //PRINTF("ang: %d,%d,%d\n", accel_angle,angul_angle,filter_angle);
    }
    else
    {
        ctrl_angCtrlEn[0] = 0;
        //PRINTF("\n[W] IMU Fail!\n");
    }
    if(1 == ctrl_angCtrlEn[0])
    {
        PIDCTRL_ErrUpdate(&ctrl_angPid, -(filter_angle - ctrl_angSet));
        ctrl_angPidOutput = PIDCTRL_CalcPIDGain(&ctrl_angPid);
        ctrl_angPidOutput = ctrl_angPidOutput < 30.0f ? ctrl_angPidOutput : 30.0f;  ///< 限幅
    }
    else
    {
        ctrl_angPidOutput = 0.0f;
    }

    MotorUpdate(ctrl_angPidOutput, ctrl_angPidOutput);
}

/* *********************************************** */

float ctrl_motorLQ = 0.0f, ctrl_motorRQ = 0.0f;

void MotorUpdate(float motorL, float motorR)
{
    /// 平滑
    ctrl_motorLQ = ctrl_motorLQ * 0.75f + motorL * 0.25f;
    ctrl_motorRQ = ctrl_motorRQ * 0.75f + motorR * 0.25f;

    if(ctrl_motorLQ > 0)
    {
        SCFTM_PWM_ChangeHiRes(MOTOR_PERIPHERAL, kFTM_Chnl_0, 20000U, ctrl_motorLQ);
        SCFTM_PWM_ChangeHiRes(MOTOR_PERIPHERAL, kFTM_Chnl_1, 20000U, 0.0f);
    }
    else
    {
        SCFTM_PWM_ChangeHiRes(MOTOR_PERIPHERAL, kFTM_Chnl_0, 20000U, 0.0f);
        SCFTM_PWM_ChangeHiRes(MOTOR_PERIPHERAL, kFTM_Chnl_1, 20000U, -ctrl_motorLQ);
    }

    if(ctrl_motorRQ > 0)
    {
        SCFTM_PWM_ChangeHiRes(MOTOR_PERIPHERAL, kFTM_Chnl_2, 20000U, ctrl_motorRQ);
        SCFTM_PWM_ChangeHiRes(MOTOR_PERIPHERAL, kFTM_Chnl_3, 20000U, 0.0f);
    }
    else
    {
        SCFTM_PWM_ChangeHiRes(MOTOR_PERIPHERAL, kFTM_Chnl_2, 20000U, 0.0f);
        SCFTM_PWM_ChangeHiRes(MOTOR_PERIPHERAL, kFTM_Chnl_3, 20000U, -ctrl_motorRQ);
    }
}
void testtest(void){
    float ctrl_motorLQ = 10.0f;
    SCFTM_PWM_ChangeHiRes(MOTOR_PERIPHERAL, kFTM_Chnl_0, 20000U, ctrl_motorLQ);
    SCFTM_PWM_ChangeHiRes(MOTOR_PERIPHERAL, kFTM_Chnl_1, 20000U, 0.0f);
}
