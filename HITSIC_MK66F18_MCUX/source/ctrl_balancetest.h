/*
 * ctrl_balance.h
 *
 *  Created on: 2020年11月14日
 *      Author: lx
 */

#ifndef CTRL_BALANCE_H_
#define CTRL_BALANCE_H_

#include "hitsic_common.h"
#include "inc_stdlib.hpp"

#include "sys_pitmgr.hpp"
#include "sys_extint.hpp"
#include "drv_imu_invensense.hpp"
#include "lib_pidctrl.h"

#include "app_menu.hpp"
#include "sc_ftm.h"
#include "sc_host.h"

#define pi (3.1415926f)
#define accel_grv (9.8f)
#define Tg (1.0f/0.8f)
#define rad2ang(x) (x*(pi/180.0f))
#define ang2rad(x) (x*(180.0f/pi))

extern float accel[3];
extern float angul[3];
extern float filter_angle;
extern float accel_angle;
extern float angul_angle;
extern float ave_angle;//初始化时求出的平均角度


extern inv::mpu6050_t imu_6050;
extern int32_t ctrl_angCtrlEn[3];
extern float ctrl_angSet;
extern pidCtrl_t ctrl_angPid;
extern float ctrl_angPidOutput;

void FilterInit(void);
void FilterUpdate(uint32_t updateTime_ms);
void Balance_CTR(void);
void Interrupt(void);
void SendAngle(void);
void CTRL_Init(void);
void CTRL_AngCtrl(void *userData);
void MotorUpdate(float motorL, float motorR);
void testtest(void);
void MENU_DataSetUpTEST(void);
#endif /* CTRL_BALANCE_H_ */
