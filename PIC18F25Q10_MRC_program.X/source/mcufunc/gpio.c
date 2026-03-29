/*
 * File:   main.c
 * Author: ICE_MEGANE
 *
 * Created on 2020/03/5, 22:26
 */

#include <xc.h>
#include "../userdefine.h"

#include "gpio.h"


#define GPIO_PORT_IN_JUDGE_CNT      ((u8)2)

/* 入力ポート割り当て設定 */



/* 出力ポート割り当て設定 (レジスタ・bit指定) */

u8 U8_GPIO_G_OUT_NEUTRAL;        /* クラッチ制御はメインマイコン側だが、各段のギヤを強制ニュートラル状態にする信号として受け取っておく */
u8 U8_GPIO_G_OUT_SHIFT_0;
u8 U8_GPIO_G_OUT_SHIFT_1;
u8 U8_GPIO_G_OUT_SHIFT_2;

u8 U8_GPIO_G_OUT_DEBUG;

static const ts_gpio_in_def ts_gpio_s_in_init =
{
    (u8)0,
    (u8)0,
    (u8)0
};

/* 関数プロトタイプ宣言 */
static void func_gpio_s_in_judge( void );
static void func_gpio_s_out_update( void );
static void func_gpio_s_port_judge( ts_gpio_in_def ts_port, u8 port_in );


/* グローバル変数 */
/**************************************************************/
/*  Function:                                                 */
/*  main関数                                                   */
/*                                                            */
/*                                                            */
/**************************************************************/
void func_gpio_g_main( void )
{
    func_gpio_s_in_judge();         /* 入力ポート判定更新 */
    func_gpio_s_out_update();       /* 出力ポート更新 */
}


/**************************************************************/
/*  Function:                                                 */
/*  gpio初期化関数                                             */
/*                                                            */
/*                                                            */
/**************************************************************/
void func_gpio_g_init( void )
{
    U8_GPIO_G_OUT_NEUTRAL = CLEAR;
    U8_GPIO_G_OUT_SHIFT_0 = CLEAR;
    U8_GPIO_G_OUT_SHIFT_1 = CLEAR;
    U8_GPIO_G_OUT_SHIFT_2 = CLEAR;

    U8_GPIO_G_OUT_DEBUG = CLEAR;
}


/**************************************************************/
/*  Function:                                                 */
/*  ポート入力判定関数                                          */
/*                                                            */
/**************************************************************/
static void func_gpio_s_in_judge( void )
{
    //func_gpio_s_port_judge( ts_gpio_g_in_neutral, GPIO_IN_NERUTRAL_PORT );
}



/**************************************************************/
/*  Function:                                                 */
/*  ポート出力設定関数                                          */
/*  出力切り替えはすべてこの処理呼び出し時に実行する               */
/**************************************************************/
static void func_gpio_s_out_update( void )
{
    /* 一応 0bit目 以外はマスクしておく */
    GPIO_OUT_SHIFT_NEUTRAL = U8_GPIO_G_OUT_NEUTRAL & (u8)0x01;

    GPIO_OUT_SHIFT_0 = U8_GPIO_G_OUT_SHIFT_0 & (u8)0x01;
    GPIO_OUT_SHIFT_1 = U8_GPIO_G_OUT_SHIFT_1 & (u8)0x01;
    GPIO_OUT_SHIFT_2 = U8_GPIO_G_OUT_SHIFT_2 & (u8)0x01;
}



/**************************************************************/
/*  Function:                                                 */
/*  ポート入力関数                                             */
/*  最初のポート状態を基準として連続して同じ入力があったら         */
/*  判定を切り替える。                                         */
/*  入力がふらふらしている場合は最初の状態固定になるので注意。     */
/**************************************************************/
static void func_gpio_s_port_judge( ts_gpio_in_def ts_port, u8 port_in )
{   
    if( ts_port.u8_buff != port_in )
    { /* 前回とポート状態が違う */
        ts_port.u8_judge_cnt = (u8)0;
    }
    
    if( ts_port.u8_judge_cnt > GPIO_PORT_IN_JUDGE_CNT )
    { /* n回連続でポート状態が同じ */
        ts_port.u8_state = port_in;
    }
}





