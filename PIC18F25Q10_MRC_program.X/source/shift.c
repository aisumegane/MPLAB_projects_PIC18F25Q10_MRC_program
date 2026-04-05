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
#include "./tools/speedsens.h"

#define SHIFT_DRIVE_STOP_CNT                ((u8)10)     /* 10ms */
#define SHIFT_DRIVE_STOP_SPEED              ((u16)0)     /* 0rpm */


/* シフトチェンジの制御を担当 */

/* 関数プロトタイプ宣言 */
static void func_shift_s_shift_mode_decide( void );
static void func_shift_s_shift_degree_calc( void );
static void func_shift_s_shift_position_decide( void );
static void func_shit_s_shift_position_output( void );

static u8 u8_shift_s_shift_chg_enable_wait_cnt;
static u8 u8_shift_s_shift_mode_req;

u8 u8_shift_g_shift_mode;
u8 u8_shift_g_shift_position;


/* グローバル変数 */
/**************************************************************/
/*  Function:                                                 */
/*  初期化関数                                                 */
/*                                                            */
/*                                                            */
/**************************************************************/
void func_shift_g_init( void )
{
    u8_shift_s_shift_mode_req = SHIFT_MODE_MANUAL;
    u8_shift_g_shift_mode = SHIFT_MODE_MANUAL;
    u8_shift_g_shift_position = SHIFT_POSI_0;
    u8_shift_s_shift_chg_enable_wait_cnt = (u8)0;
}

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
    u8 u8_mode_before;

    u8_mode_before = u8_shift_g_shift_mode;

    if( u8_shift_s_shift_chg_enable_wait_cnt < U8_MAX )
    {
        u8_shift_s_shift_chg_enable_wait_cnt++;
    }
    
    /* 変速モード変更要求 */
    if( gpio_g_shift_mode_sw.u8_state == HI )
    {
        u8_shift_s_shift_mode_req = SHIFT_MODE_MANUAL;
    }
    else
    {
        u8_shift_s_shift_mode_req = SHIFT_MODE_AUTOMATIC;
    }

    /* 停止状態でのみ、シフト操作モードの変更を許可する */
    if( ( u8_shift_s_shift_chg_enable_wait_cnt >= SHIFT_DRIVE_STOP_CNT ) &&
        ( u16_speedsens_g_speed_ave_1stgear == SHIFT_DRIVE_STOP_SPEED ) )               /* 1次ギヤで止まってる判定する場合、0,1,2のシフトレバーが接続状態でないとダメなので、条件としては微妙かも */
    { /* 現在車は停止している */
        u8_shift_g_shift_mode = u8_shift_s_shift_mode_req;          /* 現在の変速モード要求を反映する */
    }

    if( u8_mode_before != u8_shift_s_shift_chg_enable_wait_cnt )
    { /* 変速モードに変化があった */
        u8_shift_s_shift_chg_enable_wait_cnt = (u8)0;       /* クリア */
    }
}
