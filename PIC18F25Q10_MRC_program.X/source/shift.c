/*
 * File:   shift.c
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

#define SHIFT_CHK_TIME                ((u8)500)     /* *10ms */

#define SERVO_POSI_LOWER_DEC          SERVO_DEG_IDX__15     /* 中心位置に対して下方向に何度ずらすか */
#define SERVO_POSI_UPPER_ADD          SERVO_DEG_IDX__15     /* 中心位置に対して上方向に何度ずらすか */

/* シフトチェンジの制御を担当 */

/* 関数プロトタイプ宣言 */
static void func_shift_s_shift_mode_decide( void );
static void func_shift_s_shift_degree_calc( void );
static void func_shift_s_shift_position_decide( void );
static void func_shit_s_shift_position_output( void );

u8 u8_shift_g_shift_mode;
u8 u8_shift_g_shift_position;

static u8 u8_shift_s_deg_newtral_idx;
static u8 u8_shift_s_deg_upper_idx;
static u8 u8_shift_s_deg_lower_idx;


static u8 u8_shift_s_shift_chk_cnt;


/* グローバル変数 */
/**************************************************************/
/*  Function:                                                 */
/*  main関数                                                   */
/*                                                            */
/*                                                            */
/**************************************************************/
void func_shift_g_main( void )
{
    /* 入力判定 */
    func_shift_s_shift_mode_decide();           /* 変則モード確定処理 */
    func_shift_s_shift_position_decide();       /* シフトチェンジ処理 */

    /* 出力制御 */
    func_shit_s_shift_position_output();        /* シフト位置出力処理 */
}


/**************************************************************/
/*  Function:                                                 */
/*  初期化関数                                                 */
/*                                                            */
/*                                                            */
/**************************************************************/
void func_shift_g_init( void )
{
    u8_shift_g_shift_mode = SHIFT_MODE_MANUAL;
    u8_shift_g_shift_position = SHIFT_POSI_0;

    u8_shift_s_deg_newtral_idx = (u8)0;
    u8_shift_s_deg_upper_idx   = (u8)0;
    u8_shift_s_deg_lower_idx   = (u8)0;
}



/**************************************************************/
/*  Function:                                                 */
/*  関数                                                 */
/*                                                            */
/*                                                            */
/**************************************************************/
static void func_shift_s_shift_position_decide( void )
{
    if( u8_shift_g_shift_mode == SHIFT_MODE_MANUAL )
    { /* マニュアルシフト */
        if( ( gpio_g_paddle_shift_sw.u8_state == LOW ) &&
            ( gpio_g_paddle_shift_sw.u8_state_bf == MID ))
        { /* シフトアップ */
            if( u8_shift_g_shift_position < SHIFT_POSI_7 )
            {
                u8_shift_g_shift_position++;
            }
            else
            {
                u8_shift_g_shift_position = SHIFT_POSI_7;
            }
        }
        else if( ( gpio_g_paddle_shift_sw.u8_state == HI ) &&
                 ( gpio_g_paddle_shift_sw.u8_state_bf == MID ))
        { /* シフトダウン */
            if( u8_shift_g_shift_position > SHIFT_POSI_1 )
            {
                u8_shift_g_shift_position--;
            }
            else
            {
                u8_shift_g_shift_position = SHIFT_POSI_0;
            }
        }
        else
        {
            ;               /* 現在のシフト位置を維持 */
        }
    }
    else if( u8_shift_g_shift_mode == SHIFT_MODE_AUTOMATIC )
    {

    }
    else
    {
        ;
    }
}


/**************************************************************/
/*  Function:                                                 */
/*  シフト位置出力処理                                          */
/*                                                            */
/**************************************************************/
static void func_shit_s_shift_position_output( void )
{
    /* 0bit目 */
    if( ( u8_shift_g_shift_position & ((u8)0x01) ) != (u8)0 )
    { /* 0bit目が立っている */
        U8_GPIO_G_OUT_SHIFT_0 = SET;
    }
    else
    {
        U8_GPIO_G_OUT_SHIFT_0 = CLEAR;
    }

    /* 1bit目 */
    if( ( u8_shift_g_shift_position & ((u8)0x02) ) != (u8)0 )
    { /* 0bit目が立っている */
        U8_GPIO_G_OUT_SHIFT_1 = SET;
    }
    else
    {
        U8_GPIO_G_OUT_SHIFT_1 = CLEAR;
    }

    /* 2bit目 */
    if( ( u8_shift_g_shift_position & ((u8)0x04) ) != (u8)0 )
    { /* 0bit目が立っている */
        U8_GPIO_G_OUT_SHIFT_2 = SET;
    }
    else
    {
        U8_GPIO_G_OUT_SHIFT_2 = CLEAR;
    }
}


/**************************************************************/
/*  Function:                                                 */
/*  変速モード確定関数                                          */
/*                                                            */
/**************************************************************/
static void func_shift_s_shift_mode_decide( void )
{
    if( gpio_g_shift_mode_sw.u8_state == HI )
    {
        u8_shift_g_shift_mode = SHIFT_MODE_MANUAL;
    }
    else
    {
        u8_shift_g_shift_mode = SHIFT_MODE_AUTOMATIC;
    }

}
