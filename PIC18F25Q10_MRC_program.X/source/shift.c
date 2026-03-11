/*
 * File:   main.c
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

#define SHIFT_CHK_TIME                ((u8)500)     /* *10ms */

#define SERVO_POSI_LOWER_DEC          SERVO_DEG_IDX__15     /* 中心位置に対して下方向に何度ずらすか */
#define SERVO_POSI_UPPER_ADD          SERVO_DEG_IDX__15     /* 中心位置に対して上方向に何度ずらすか */

/* シフトチェンジの制御を担当 */

/* 関数プロトタイプ宣言 */
static void func_shift_s_shift_degree_calc( void );
static void func_shift_s_shift_change( void );


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
    func_shift_s_shift_degree_calc();           /* サーボの角度計算処理 */
    func_shift_s_shift_change();                /* シフトチェンジ処理 */
}


/**************************************************************/
/*  Function:                                                 */
/*  初期化関数                                                 */
/*                                                            */
/*                                                            */
/**************************************************************/
void func_shift_g_init( void )
{
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
static void func_shift_s_shift_degree_calc( void )
{
    u8 u8_loopidx;
    u16 u16_adc_compare_cnt;
    
    /* 中立時の角度指定 */
    /* 割り算ができないので、全範囲からどの位置に割り出すためにfor文を使う */
    /* サーボの角度は36分割。AD値はフルスケールで最大1024なので、28を引き続けてマッチしたところを、おおよその位置指定IDXとする */


    /* ローカル変数初期化 */
    u8_loopidx = (u8)0;
    u16_adc_compare_cnt = (u16)0;

    while( u8_loopidx < SERVO_ANGLE_NUM )
    {
        if( u16_adc_compare_cnt > u16_adc_g_ad_result_ave____servo_posi_adj )
        {
            break;      /* 2点のうち、上側の角度idxを返すことにした */
        }

        u16_adc_compare_cnt += (u16)28;     /* 小数点以下はずれるので、あまり精度はない */
        u8_loopidx++;
    }

    if( u8_loopidx < SERVO_DEG_IDX__180 )
    {
        u8_shift_s_deg_newtral_idx = u8_loopidx;
    }
    else
    {
        u8_shift_s_deg_newtral_idx = SERVO_DEG_IDX__180;
    }
    
    
    /* 下側時の角度指定  */
    if( u8_shift_s_deg_newtral_idx > SERVO_POSI_LOWER_DEC )
    {
        u8_shift_s_deg_lower_idx = u8_shift_s_deg_newtral_idx - SERVO_POSI_LOWER_DEC;
    }
    else
    { /* 下側に寄せると0度になってしまう場合は、0度に固定する */
        u8_shift_s_deg_lower_idx = SERVO_DEG_IDX__0;
    }

    /* 上側時の角度指定  */
    u8_shift_s_deg_upper_idx = u8_shift_s_deg_newtral_idx + SERVO_POSI_UPPER_ADD;
    
    if( u8_shift_s_deg_upper_idx > SERVO_DEG_IDX__180 )
    {
        u8_shift_s_deg_upper_idx = SERVO_DEG_IDX__180;
    }
}


/**************************************************************/
/*  Function:                                                 */
/*  関数                                                 */
/*                                                            */
/*                                                            */
/**************************************************************/
static void func_shift_s_shift_change( void )
{
    /* シフトポジション指定 */
    if( ts_gpio_g_in_neutral.u8_state == SET )
    {
        /* サーボ上下に固定 */
        if( ts_gpio_g_in_shift_0.u8_state == SET )
        {
            servo_s_angle_set( u8_shift_s_deg_upper_idx, SERVO_SHIFT_0 );
        }
        else
        {
            servo_s_angle_set( u8_shift_s_deg_lower_idx, SERVO_SHIFT_0 );
        }


        if( ts_gpio_g_in_shift_1.u8_state == SET )
        {
            servo_s_angle_set( u8_shift_s_deg_upper_idx, SERVO_SHIFT_1 );
        }
        else
        {
            servo_s_angle_set( u8_shift_s_deg_lower_idx, SERVO_SHIFT_1 );
        }


        if( ts_gpio_g_in_shift_2.u8_state == SET )
        {
            servo_s_angle_set( u8_shift_s_deg_upper_idx, SERVO_SHIFT_2 );
        }
        else
        {
            servo_s_angle_set( u8_shift_s_deg_lower_idx, SERVO_SHIFT_2 );
        }
    }
    else
    {
        /* サーボ 中立 */
        servo_s_angle_set( u8_shift_s_deg_newtral_idx, SERVO_SHIFT_0 );
        servo_s_angle_set( u8_shift_s_deg_newtral_idx, SERVO_SHIFT_1 );
        servo_s_angle_set( u8_shift_s_deg_newtral_idx, SERVO_SHIFT_2 );
    }
}

