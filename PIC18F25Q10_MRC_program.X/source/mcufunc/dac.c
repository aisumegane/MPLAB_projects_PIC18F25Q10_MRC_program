/*
 * File:   dac.c
 * Author: ICE_MEGANE
 *
 * Created on 2026/04/4, 20:48
 */

#include <xc.h>
#include ".././userdefine.h"
#include "../mcufunc/pic18f25q10.h"


#include "../mcufunc/gpio.h"
#include ".././radio_control.h"
#include ".././speedsens.h"

#include "dac.h"


/* 定義 */
#define DAC_MAX_OUTPUT_VAL              ((u8)31)        /* 5bit最大値 0~31の計32段階出力 */

/* プロトタイプ宣言 */
static void func_dac_s_debug_out( u32 u32_val, u32 u32_val_max );

/* グローバル変数 */
/**************************************************************/
/*  Function:                                                 */
/*  main関数                                                   */
/*                                                            */
/*                                                            */
/**************************************************************/
void func_dac_g_main( void )
{
    //func_dac_s_debug_out( u8_rc_g_ch_duty_tbl[ RC_DUTY_CH_THROTTLE ] );
    func_dac_s_debug_out( (u32)u16_speedsens_g_speed_ave_mtr, (u32)15000 );             /* 0~15000rpm の範囲を0~5VのDAC出力で表現 */
}


/**************************************************************/
/*  Function:                                                 */
/*  初期化関数                                                 */
/*                                                            */
/*                                                            */
/**************************************************************/
void func_dac_g_init( void )
{
    ;
}


/* static関数 */
/**************************************************************/
/*  Function:                                                 */
/*  DACデバッグ用関数                                          */
/**************************************************************/
static void func_dac_s_debug_out( u32 u32_val, u32 u32_val_max )
{
    u32 u32_calc_buff;

    u32_calc_buff = u32_val * (u32)DAC_MAX_OUTPUT_VAL;
    u32_calc_buff = u32_calc_buff / u32_val_max;            /* 32分割で規格化 */

    if( u32_calc_buff > (u32)DAC_MAX_OUTPUT_VAL )
    {
        u32_calc_buff = (u32)DAC_MAX_OUTPUT_VAL;
    }

    /* DAC出力設定 */
    DAC1CON1 = (u8)u32_calc_buff;
}


