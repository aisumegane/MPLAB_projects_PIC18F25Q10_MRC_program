/*
 * File:   speedsens.c
 * Author: ICE_MEGANE
 *
 * Created on 2026/4/2, 22:26
 */

#include <xc.h>
#include "./mcufunc/pic18f25q10.h"
#include "userdefine.h"


#include "./mcufunc/gpio.h"
#include "./mcufunc/mcu_setup.h"
#include "./mcufunc/timer_driver.h"

#include "speedsens.h"


/* パラメータ */
#define SPEEDSENS_SPEED_AT_1_CAPTURE    ((u16)232500)

#define SPEEDSENS_CAPTURE_MTR_NUM         ((u8)4)
#define SPEEDSENS_CAPTURE_1STGEAR_NUM     ((u8)4)


u8 u8_speedsens_s_capture_ary_mtr_index;
u16 u16_speedsens_s_capture_ary_mtr[ SPEEDSENS_CAPTURE_MTR_NUM ] =
{
    (u8)0,(u8)0,(u8)0,(u8)0
};

u8 u8_speedsens_s_capture_ary_1stgear_index;
u16 u16_speedsens_s_capture_ary_1stgear[ SPEEDSENS_CAPTURE_1STGEAR_NUM ] =
{
    (u8)0,(u8)0,(u8)0,(u8)0
};

static u16 u16_speedsens_s_mtr_capture_ave;
static u16 u16_speedsens_s_1stgear_capture_ave;

u16 u16_speedsens_g_mtr_speed_ave;
u16 u16_speedsens_g_1stgear_speed_ave;

/**************************************************************/
/*  Function:                                                 */
/*  ループタスク                                               */
/**************************************************************/
void func_speedsens_g_main( void )
{
    u16_speedsens_g_mtr_speed_ave = func_speedsens_g_calc_speed( u16_speedsens_s_mtr_capture_ave );
    u16_speedsens_g_1stgear_speed_ave = func_speedsens_g_calc_speed( u16_speedsens_s_1stgear_capture_ave );
}


/**************************************************************/
/*  Function:                                                 */
/*  初期化処理                                                 */
/**************************************************************/
void func_speedsens_g_init( void )
{
    u16_speedsens_s_mtr_capture_ave = (u16)0;
    u16_speedsens_g_1stgear_speed_ave = (u16)0;
    
    u8_speedsens_s_capture_ary_mtr_index = (u8)0;
    u8_speedsens_s_capture_ary_1stgear_index = (u8)0;
}

/**************************************************************/
/*  Function:                                                 */
/*  キャプチャ値を回収する                                      */
/**************************************************************/
void func_speedsens_g_collect_mtr_capture( u16 u16_capture )
{
    u16 u16_loopcnt;
    
    u16_speedsens_s_capture_ary_mtr[ u8_speedsens_s_capture_ary_mtr_index ] = u16_capture;
    
    if( u8_speedsens_s_capture_ary_mtr_index < SPEEDSENS_CAPTURE_MTR_NUM )
    {
        u8_speedsens_s_capture_ary_mtr_index++;
        
        if( u8_speedsens_s_capture_ary_mtr_index == SPEEDSENS_CAPTURE_MTR_NUM )
        {
            u8_speedsens_s_capture_ary_mtr_index = (u16)0;
        }
    }
    
    u16_speedsens_s_mtr_capture_ave = (u16)0;
    
    for( u16_loopcnt = (u16)0; u16_loopcnt < SPEEDSENS_CAPTURE_MTR_NUM; u16_loopcnt++ )
    {
        u16_speedsens_s_mtr_capture_ave += u16_speedsens_s_capture_ary_mtr[ u16_loopcnt ];
    }
    
    u16_speedsens_s_mtr_capture_ave = u16_speedsens_s_mtr_capture_ave >> 2U;
}

/**************************************************************/
/*  Function:                                                 */
/*  キャプチャ値を回収する                                      */
/**************************************************************/
void func_speedsens_g_collect_1stgear_capture( u16 u16_capture )
{
    u16 u16_loopcnt;
    
    u16_speedsens_s_capture_ary_1stgear[ u8_speedsens_s_capture_ary_1stgear_index ] = u16_capture;
    
    if( u8_speedsens_s_capture_ary_1stgear_index < SPEEDSENS_CAPTURE_1STGEAR_NUM )
    {
        u8_speedsens_s_capture_ary_1stgear_index++;
        
        if( u8_speedsens_s_capture_ary_1stgear_index == SPEEDSENS_CAPTURE_1STGEAR_NUM )
        {
            u8_speedsens_s_capture_ary_1stgear_index = (u16)0;
        }
    }
    
    u16_speedsens_s_1stgear_capture_ave = (u16)0;
    
    for( u16_loopcnt = (u16)0; u16_loopcnt < SPEEDSENS_CAPTURE_1STGEAR_NUM; u16_loopcnt++ )
    {
        u16_speedsens_s_1stgear_capture_ave += u16_speedsens_s_capture_ary_1stgear[ u16_loopcnt ];
    }
    
    u16_speedsens_s_1stgear_capture_ave = u16_speedsens_s_1stgear_capture_ave >> 2U;
}

/**************************************************************/
/*  Function:                                                 */
/*  回転数を算出する                                           */
/*  最大キャプチャに対して現在のキャプチャがいくつかを計算し、     */
/*  最大キャプチャの時の */
/**************************************************************/
u16 func_speedsens_g_calc_speed( u16 u16_capture_ave )
{
    u32 u32_speed;
    u16 u16_result;
    
    u32_speed = (u32)SPEEDSENS_SPEED_AT_1_CAPTURE * (u32)u16_capture_ave;
    u32_speed = u32_speed / (u32)U16_MAX;
    
    u16_result = (u16)u32_speed;
    
    return u16_result;
}
