
/*
 * File:   main.c
 * Author: ICE_MEGANE
 *
 * Created on 2020/03/5, 22:26
 */

#include <xc.h>
#include "../userdefine.h"

#include "adc.h"
#include "../main.h"

#include "int.h"


/* 割り込みフラグ bit位置定義 define */
#define INT_FLAG_TMR0_OVF           ((u8)0x04)
#define INT_FLAG_TMR4_MATCH         ((u8)0x02)
#define INT_FLAG_AD_COMPLITE        ((u8)0x40)


/* 関数プロトタイプ宣言 */
static void func_int_s_timer0_ovf( void );
static void func_int_s_timer4_match( void );



/**************************************************************/
/*  Function:                                                 */
/*  割り込み処理                                               */
/*  割り込み発生時にここに来る                                  */
/*  ベクタは1コだけなので割り込みフラグを処理内で判別             */
/**************************************************************/
void __interrupt() isr( void )
{
    /* 割り込みベクタ1コですべてここに来るため、実質記述順序が割り込み優先度になる */
    /* 割り込みが同時に入った場合はこの処理の中で長い時間待機してしまう点忘れない! */

    if( ( INTCON & INT_FLAG_TMR0_OVF ) != 0U )
    { /* TMR0 オーバーフロー割り込み発生 */
        func_int_s_timer0_ovf();
        INTCON &= (u8)~INT_FLAG_TMR0_OVF;
    }

    if( ( PIR3 & INT_FLAG_TMR4_MATCH ) != 0U )
    { /* TMR4 一致割り込み発生 */
        func_int_s_timer4_match();
        PIR3 &= (u8)~INT_FLAG_TMR4_MATCH;                         /* 割り込みフラグクリア */
    }

    if( ( PIR1 & INT_FLAG_AD_COMPLITE ) != 0U )
    { /* AD変換 完了割り込み発生 */
        func_adc_g_adc_data_get();                 /* AD変換結果格納 & 残りの変換要求を処理する */
        PIR1 &= (u8)~INT_FLAG_AD_COMPLITE;                         /* 割り込みフラグクリア */
    }
}


/**************************************************************/
/*  Function:                                                 */
/*  割り込み関連のmain-loop処理                                 */
/*                                                            */
/**************************************************************/
void func_int_g_main( void )
{
    /* 特に使う予定はない */
    /* 割り込み関連のフラグをループ側でクリアしたい場合はここで処理する */
}


/**************************************************************/
/*  Function:                                                 */
/*  割り込み関連初期化処理                                      */
/*                                                            */
/**************************************************************/
void func_int_g_init( void )
{
    ;
}


/**************************************************************/
/*  Function:                                                 */
/*  タイマ0 OVF割り込みタスク                                   */
/**************************************************************/
static void func_int_s_timer0_ovf( void )
{
    func_main_s_main_loop_judge();
}


/**************************************************************/
/*  Function:                                                 */
/*  タイマ4 一致割り込みタスク                                  */
/**************************************************************/
static void func_int_s_timer4_match( void )
{
    
}
