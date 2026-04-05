/*
 * File:   speedcontrol.c
 * Author: ICE_MEGANE
 *
 * Created on 2020/03/5, 22:26
 */

#include <xc.h>
#include "userdefine.h"

#include "shift.h"

#include "./mcufunc/gpio.h"
#include "./mcufunc/adc.h"
#include "./tools/servo.h"
#include "radio_control.h"
#include "./tools/speedsens.h"

#include "speedcontrol.h"


#define SC_THROTTLE_DEFINE_0P           RC_CH_DUTY_50P          /* スロットル中立時の入力 */
#define SC_THROTTLE_HYSTERESIS          RC_CH_DUTY_5P           /* スロットル操作のヒステリシス */

u8 u8_sc_s_throttle_dir;
u8 u8_sc_g_throttle_duty_recalc;


/* プロトタイプ宣言 */
static void func_sc_s_throttle_per_dir_decide( void );
static void func_sc_s_throttle_per_dir_update( u8 *u8_per, u8 *u8_dir );
static void func_sc_s_throttle_threshold_update( u8 *u8_forward_th, u8 *u8_backward_th );

/**************************************************************/
/*  Function:                                                 */
/*                                                            */
/**************************************************************/
void func_speedcontrol_g_init( void )
{
    u8_sc_s_throttle_dir = SC_THROTTLE_DIR_NONE;
    u8_sc_g_throttle_duty_recalc = RC_CH_DUTY_0P;
}


/**************************************************************/
/*  Function:                                                 */
/*                                                            */
/**************************************************************/
void func_speedcontrol_g_main( void )
{
    func_sc_s_throttle_per_dir_update( &u8_sc_g_throttle_duty_recalc ,&u8_sc_s_throttle_dir );      /* dutyと方向更新処理 */
}


/**************************************************************/
/*  Function:                                                 */
/*  スロットル入力量 判断処理                                   */
/*  レバー操作に対して0~100の入力としているため、これを前後に振り分ける */
/*  分解能的に成立していないところがあるが、実害なのでOKとした     */
/**************************************************************/
static void func_sc_s_throttle_per_dir_update( u8 *u8_per, u8 *u8_dir )
{
    u8 u8_duty_input;
    u8 u8_duty_forward_th;
    u8 u8_duty_backward_th;
    u8 u8_duty_recalc_base;
    u8 u8_duty_recalc_gap;

    u32 u32_calc_buff;

    /* 初期化 */ 
    u8_duty_input = (u8)0;
    u8_duty_forward_th = (u8)0;
    u8_duty_backward_th = (u8)0;
    u8_duty_recalc_base = (u8)0;
    u8_duty_recalc_gap = (u8)0;

    /*-----------------------------------------------------------------------*/

    u8_duty_input = u8_rc_g_ch_duty_tbl[RC_DUTY_CH_THROTTLE];

    /* スロットル閾値更新 */
    func_sc_s_throttle_threshold_update( &u8_duty_forward_th, &u8_duty_backward_th );


    /* 入力判定 */
    if( u8_duty_input > u8_duty_forward_th )
    { /* 前進方向のduty入力 */
        u8_duty_recalc_base = RC_CH_DUTY_100P - u8_duty_forward_th;
        u8_duty_recalc_gap = u8_duty_input - u8_duty_forward_th;

        u32_calc_buff = func_ud_g_calcmul_2x2_byte( (u16)u8_duty_recalc_gap, (u16)RC_CH_DUTY_100P );
        u32_calc_buff = func_ud_g_calcdiv_4x4_byte( u32_calc_buff, (u32)u8_duty_recalc_base );

        *u8_per = (u8)u32_calc_buff;            /* 前進方向のduty(再計算後)を設定 */
        *u8_dir = SC_THROTTLE_DIR_FORWARD;      /* 前進 */
    }
    else if( u8_duty_input < u8_duty_backward_th )
    { /* 後退方向のduty入力 */
        u8_duty_recalc_base = u8_duty_backward_th;
        u8_duty_recalc_gap = u8_duty_backward_th - u8_duty_input;

        u32_calc_buff = func_ud_g_calcmul_2x2_byte( (u16)u8_duty_recalc_gap, (u16)RC_CH_DUTY_100P );
        u32_calc_buff = func_ud_g_calcdiv_4x4_byte( u32_calc_buff, (u32)u8_duty_recalc_base );

        *u8_per = (u8)u32_calc_buff;            /* 後進方向のduty(再計算後)を設定 */
        *u8_dir = SC_THROTTLE_DIR_BACKWARD;     /* 後進 */
    }
    else
    { /* 中立状態 */
        *u8_per = RC_CH_DUTY_0P;
        *u8_dir = SC_THROTTLE_DIR_NONE;
    }
}

/**************************************************************/
/*  Function:                                                 */
/*  スロットル開閉閾値 更新処理                                 */
/**************************************************************/
static void func_sc_s_throttle_threshold_update( u8 *u8_forward_th, u8 *u8_backward_th )
{
    /* 前進 入力閾値レベルの設定 */
    *u8_forward_th = SC_THROTTLE_DEFINE_0P + SC_THROTTLE_HYSTERESIS;
    if( *u8_forward_th > RC_CH_DUTY_100P )
    { /* ヒステリシスを加味すると100%を超える */
        *u8_forward_th = RC_CH_DUTY_100P;
    }

    /* 更新 入力閾値レベルの設定 */
    if( SC_THROTTLE_DEFINE_0P > SC_THROTTLE_HYSTERESIS )
    { /* ヒステリシスを加味すると0%を下回る */
        *u8_backward_th = SC_THROTTLE_DEFINE_0P - SC_THROTTLE_HYSTERESIS;
    }
    else
    {
        *u8_backward_th = RC_CH_DUTY_0P;
    }
}


