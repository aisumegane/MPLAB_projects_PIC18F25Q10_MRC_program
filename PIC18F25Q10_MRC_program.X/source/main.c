/*
 * File:   main.c
 * Author: ICE_MEGANE
 *
 * Created on 2020/03/5, 22:26
 */


// PIC18F25Q10 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1L
#pragma config FEXTOSC = OFF    // External Oscillator mode Selection bits (Oscillator not enabled)
#pragma config RSTOSC = HFINTOSC_64MHZ// Power-up default value for COSC bits (HFINTOSC with HFFRQ = 64 MHz and CDIV = 1:1)

// CONFIG1H
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)

// CONFIG2L
#pragma config MCLRE = EXTMCLR  // Master Clear Enable bit (MCLR pin (RE3) is MCLR)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (Power up timer disabled)
#pragma config LPBOREN = OFF    // Low-power BOR enable bit (Low power BOR is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bits (Brown-out Reset enabled according to SBOREN)

// CONFIG2H
#pragma config BORV = VBOR_190  // Brown Out Reset Voltage selection bits (Brown-out Reset Voltage (VBOR) set to 1.90V)
#pragma config ZCD = OFF        // ZCD Disable bit (ZCD disabled. ZCD can be enabled by setting the ZCDSEN bit of ZCDCON)
#pragma config PPS1WAY = ON     // PPSLOCK bit One-Way Set Enable bit (PPSLOCK bit can be cleared and set only once; PPS registers remain locked after one clear/set cycle)
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Extended Instruction Set and Indexed Addressing Mode disabled)

// CONFIG3L
#pragma config WDTCPS = WDTCPS_31// WDT Period Select bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = OFF       // WDT operating mode (WDT Disabled)

// CONFIG3H
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC      // WDT input clock selector (Software Control)

// CONFIG4L
#pragma config WRT0 = OFF       // Write Protection Block 0 (Block 0 (000800-001FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection Block 1 (Block 1 (002000-003FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection Block 2 (Block 2 (004000-005FFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection Block 3 (Block 3 (006000-007FFFh) not write-protected)

// CONFIG4H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-30000Bh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)
#pragma config SCANE = ON       // Scanner Enable bit (Scanner module is available for use, SCANMD bit can control the module)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low voltage programming enabled. MCLR/VPP pin function is MCLR. MCLRE configuration bit is ignored)

// CONFIG5L
#pragma config CP = OFF         // UserNVM Program Memory Code Protection bit (UserNVM code protection disabled)
#pragma config CPD = OFF        // DataNVM Memory Code Protection bit (DataNVM code protection disabled)

// CONFIG5H

// CONFIG6L
#pragma config EBTR0 = OFF      // Table Read Protection Block 0 (Block 0 (000800-001FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection Block 1 (Block 1 (002000-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection Block 2 (Block 2 (004000-005FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection Block 3 (Block 3 (006000-007FFFh) not protected from table reads executed in other blocks)

// CONFIG6H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

/* ↑↑↑上記は別ファイルに移したいのだが、うまくいかないっぽい */


#include <xc.h>
#include "./mcufunc/config_bits.h"
#include "userdefine.h"     /* ※ヘッダファイル内でも定義を使用しているので、エラーが出ないよう一番最初に呼び出す※ */


#include "./mcufunc/adc.h"
#include "./mcufunc/dac.h"
#include "./mcufunc/gpio.h"
#include "./mcufunc/int.h"
#include "./mcufunc/mcu_setup.h"

#include "./tools/segment.h"
#include "./tools/servo.h"

#include "indicate.h"
#include "shift.h"
#include "radio_control.h"
#include "./tools/speedsens.h"
#include "./speedcontrol.h"
#include "./tools/hbridge.h"

#include "main.h"


/* ファイル内定義 */
#define MAIN_TASK_DIVIDER       ((u8)1)


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
    func_mset_g_mcu_stop_condition();       /* 前割り込み停止 */
    func_mset_g_init();                     /* マイコン初期化 */
    func_main_s_init();                     /* 変数初期化 */
    func_mset_g_mcu_start_condition();      /* 割り込み許可 */

    while( 1 )
    { /* ループタスク */
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
void  func_main_g_main_loop_judge( void )
{
    u8_main_s_1ms_task_cnt++;

    if( u8_main_s_1ms_task_cnt >= MAIN_TASK_DIVIDER )       /* @@割り込み発生頻度高いとデバッグしずらいので、分周なくした */
    { /* 割り込み発生から分周完了 */
        u8_main_s_loop_go = SET;
        u8_main_s_1ms_task_cnt = (u8)0;
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
    //GPIO_OUT_DEBUG = SET;



    /* 関数コール */
    /* 入力処理 */
    func_adc_g_main();          /* AD変換処理　 */
    func_int_g_main();          /* 割り込み処理 */
    func_gpio_g_main();         /* GPIOポート更新処理 */
    func_rc_g_main();           /* ラジコンプロポ duty取得処理 */
    func_speedsens_g_main();    /* 回転数検出 */

    /* 計算処理 */
    func_shift_g_main();        /* シフトチェンジ処理 */
    func_speedcontrol_g_main(); /* 速度調整処理 */
    
    /* 出力処理 */
    func_dac_g_main();          /* DAC出力処理 */
    func_indicate_g_main();     /* 表示処理 */
    func_hbridge_g_main();     /* インバータ制御処理 */

    //GPIO_OUT_DEBUG = CLEAR;
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
    
    func_ud_g_init();

    /* 入力処理 */
    func_adc_g_init();
    func_int_g_init();
    func_gpio_g_init();
    func_rc_g_init();
    func_speedsens_g_init();
    
    /* 計算処理 */
    func_speedcontrol_g_init();
    func_shift_g_init();

    /* 出力処理 */
    func_dac_g_init();
    func_indicate_g_init();
    func_speedsens_g_init();
    func_inverter_g_init();

    
    /* その他 */
    func_segment_g_init();
    func_servo_g_init();
}





