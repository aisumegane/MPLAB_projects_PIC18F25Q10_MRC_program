/*
 * File:   main.c
 * Author: ICE_MEGANE
 *
 * Created on 2020/03/5, 22:26
 */

#include <xc.h>
#include "config_bits.h"
#include "userdefine.h"     /* ※ヘッダファイル内でも定義を使用しているので、エラーが出ないよう一番最初に呼び出す※ */


#include "./mcufunc/adc.h"
#include "./mcufunc/gpio.h"
#include "./mcufunc/int.h"
#include "./mcufunc/mcu_setup.h"

#include "./tools/segment.h"
#include "./tools/servo.h"

#include "indicate.h"
#include "shift.h"
#include "radio_control.h"

#include "main.h"


/* ファイル内定義 */
#define MAIN_TASK_DIVIDER       ((u8)10)


/* 関数プロトタイプ宣言 */
static void func_main_s_init( void );
static void func_main_s_loop( void );


/* 変数宣言 */
static u8 u8_main_s_loop_go;
static u8 u8_main_s_1ms_task_cnt;


/**************************************************************/
/*  Function:                                                 */
/*  main task                                                 */
/*                                                            */
/**************************************************************/
void main(void)
{   
    func_mset_g_init();                     /* マイコン初期化 */
    func_main_s_init();                     /* 変数初期化 */
    func_mset_g_mcu_start_condition();      /* プログラム開始状態 */

    while( 1 )
    { /* 10ms 周期タスク */  /* a */
        if( u8_main_s_loop_go == SET )
        {
            func_main_s_loop();
            u8_main_s_loop_go = CLEAR;
        }
    }
}


/**************************************************************/
/*  Function:                                                 */
/*  mainループ動作許可フラグ 設定関数                            */
/*                                                            */
/**************************************************************/
void  func_main_s_main_loop_judge( void )
{
    /* このマイコン主な制御はサーボモータの角度切り替え */
    /* ラジコン用プロポの角度指令が50Hz程度の入力なので、それより早い1ms(100Hz)の更新周期をもっていれば十分 */
    /* 最大の応答速度を出してると言える */
    u8_main_s_1ms_task_cnt++;

    if( u8_main_s_1ms_task_cnt > MAIN_TASK_DIVIDER )
    { /* 割り込み発生から分周完了 */
        u8_main_s_loop_go = SET;
    }
}


/*******************/
/* static function */
/*******************/
/**************************************************************/
/*  Function:                                                 */
/*  メインループ処理                                           */
/*                                                            */
/**************************************************************/
static void func_main_s_loop( void )
{
    /* テスト出力 */
    /* 10ms感覚で出力反転 */
    if( U8_GPIO_G_OUT_TASK_CHK == CLEAR )
    {
        U8_GPIO_G_OUT_TASK_CHK = SET;
    }
    else
    {
        U8_GPIO_G_OUT_TASK_CHK = CLEAR;
    }


    /* 関数コール */
    func_adc_g_main();          /* AD変換処理　 */
    func_int_g_main();          /* 割り込み処理 */
    func_gpio_g_main();         /* GPIOポート更新処理 */
    func_rc_g_main();           /* ラジコンプロポ duty取得処理 */

    func_shift_g_main();        /* シフトチェンジ処理 */
    func_indicate_g_main();     /* 表示処理 */
}


/**************************************************************/
/*  Function:                                                 */
/*  変数初期設定                                               */
/*                                                            */
/**************************************************************/
static void func_main_s_init( void )
{
    u8_main_s_loop_go = CLEAR;                  /* 初期化 */
    u8_main_s_1ms_task_cnt = (u8)0;            /* 初期化 */
    
    func_adc_g_init();
    func_int_g_init();
    func_gpio_g_init();
    func_rc_g_init();
    
    func_segment_g_init();
    func_servo_g_init();
    

    func_shift_g_init();
    func_indicate_g_init();
}





