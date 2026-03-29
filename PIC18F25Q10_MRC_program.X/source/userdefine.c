/*
 * File:   shift.c
 * Author: ICE_MEGANE
 *
 * Created on 2020/03/5, 22:26
 */

#include <xc.h>
#include "userdefine.h"

#include "./mcufunc/gpio.h"
#include "./mcufunc/adc.h"
#include "./tools/servo.h"



/* 全ファイル共通で使いたい関数のみ定義する */

/* 2byte * 2byte 計算 */
u32  func_ud_g_calcmul_2x2_byte( u16 u16_arg1, u16 u16_arg2 )
{
    u16 u16_arg1_upper_8bit;
    u16 u16_arg1_lower_8bit;
    
    u16 u16_arg2_upper_8bit;
    u16 u16_arg2_lower_8bit;
    
    u32 u32_result;
    
    u16 u16_calc_buff;
    
    /* ローカル変数初期化 */
    u16_arg1_upper_8bit = (u8)0;
    u16_arg1_lower_8bit = (u8)0;
    u16_arg2_upper_8bit = (u8)0;
    u16_arg2_lower_8bit = (u8)0;
    u16_calc_buff = (u16)0;
    u32_result = (u32)0;
    
    /*  */
    u16_arg1_upper_8bit = u16_arg1 >> 8U;       /* 8bit*8bit の計算をするので、箱は16bitで足りる */
    u16_arg1_lower_8bit = u16_arg1;
    
    u16_arg2_upper_8bit = u16_arg2 >> 8U;
    u16_arg2_lower_8bit = u16_arg2;
    
    /* 掛け算：1/4 */
    u16_calc_buff = u16_arg1_upper_8bit * u16_arg2_upper_8bit;        /* ハードウェア演算に移動する */
    u32_result += ((u32)u16_calc_buff) << 16U;
    
    /* 掛け算：2/4 */
    u16_calc_buff = u16_arg1_upper_8bit * u16_arg2_lower_8bit;
    u32_result += ((u32)u16_calc_buff) << 8U;
    
    /* 掛け算：3/4 */
    u16_calc_buff = u16_arg1_lower_8bit * u16_arg2_upper_8bit;
    u32_result += ((u32)u16_calc_buff) << 8U;
    
    /* 掛け算：4/4 */
    u16_calc_buff = u16_arg1_lower_8bit * u16_arg2_lower_8bit;
    u32_result += ((u32)u16_calc_buff);

    return u32_result;
}

/* 4byte / 4byte 割り算 */
/* 処理重いので成立しないならアセンブラコード使う */
u32 func_ud_g_calcdiv_4x4_byte( u32 u32_arg1, u32 u32_arg2 )
{
    u32 u32_calc_buff;

    u32_calc_buff = u32_arg1 / u32_arg2;

    return u32_calc_buff;
}