
/*
 * File:   main.c
 * Author: ICE_MEGANE
 *
 * Created on 2020/03/5, 22:26
 */

#include <xc.h>
#include "pic18f25q10.h"
#include "../userdefine.h"

#include "adc.h"
#include "gpio.h"
#include "../main.h"
#include "../radio_control.h"
#include "../tools/speedsens.h"

#include "int.h"


/* 割り込みフラグ bit位置定義 define */
#define INT_FLAG_TMR0_MATCH         TMR0IF
#define INT_FLAG_TMR4_MATCH         ((u8)0x02)
#define INT_FLAG_AD_COMPLITE        ((u8)0x40)
#define INT_FLAG_IOC                IOCIF
#define INT_FLAG_TMR5_OVF           TMR5IF
#define INT_FLAG_TMR1_GATE_CLOSE    TMR1GIF
#define INT_FLAG_TMR1_OVERFLOW      TMR1IF
#define INT_FLAG_TMR3_GATE_CLOSE    TMR3GIF
#define INT_FLAG_TMR3_OVERFLOW      TMR3IF

/* 関数プロトタイプ宣言 */
static void func_int_s_timer1_gate_close( void );
static void func_int_s_timer3_gate_close( void );
static void func_int_s_timer1_overflow( void );
static void func_int_s_timer3_overflow( void );



/**************************************************************/
/*  Function:                                                 */
/*  高優先度 割り込み処理                                       */
/*                                                            */
/*  割り込みベクタはHigh/Lowで2コ                               */
/**************************************************************/
void __interrupt (high_priority) high_isr(void)
{
#if 1
    if( INT_FLAG_IOC == SET )
    {
        IOCIE = CLEAR;                      /* IOCAF読み取り中に再度割り込み入ると読み取り失敗するので、処理開始前に禁止する */
        func_rc_g_duty_detection();         /* ラジコンプロポ duty取得処理 */
        /*INT_FLAG_IOC = CLEAR;*/           /* IOCFはIOC割り込みのステータスビット IOCの割り込みフラグは、上記関数内で個別にクリア */
        IOCIE = SET;
    }
#endif
}

/**************************************************************/
/*  Function:                                                 */
/*  低優先度 割り込み処理                                       */
/*                                                            */
/*  割り込みベクタはHigh/Lowで2コ                               */
/**************************************************************/
void __interrupt(low_priority) low_isr(void)
{
    /* 割り込みベクタ1コですべてここに来るため、実質記述順序が割り込み優先度になる */
    /* 割り込みが同時に入った場合はこの処理の中で長い時間待機してしまう点忘れない! */

    if( INT_FLAG_TMR0_MATCH == SET )
    { /* TMR0 オーバーフロー割り込み発生 */
        func_main_g_main_loop_judge();
        INT_FLAG_TMR0_MATCH = CLEAR;
    }

    if( ( PIR3 & INT_FLAG_TMR4_MATCH ) != 0U )
    { /* TMR4 一致割り込み発生 */
        PIR3 &= (u8)~INT_FLAG_TMR4_MATCH;                         /* 割り込みフラグクリア */
    }

    if( ( PIR1 & INT_FLAG_AD_COMPLITE ) != 0U )
    { /* AD変換 完了割り込み発生 */
        func_adc_g_adc_data_get();                 /* AD変換結果格納 & 残りの変換要求を処理する */
        PIR1 &= (u8)~INT_FLAG_AD_COMPLITE;                         /* 割り込みフラグクリア */
    }

#if 0
    /* IOCによるduty取得は割り込み優先度を高くして、値が変動しないようにする */
    if( INT_FLAG_IOC == SET )
    {
        IOCIE = CLEAR;                      /* IOCAF読み取り中に再度割り込み入ると読み取り失敗するので、処理開始前に禁止する */
        func_rc_g_duty_detection();         /* ラジコンプロポ duty取得処理 */
        /*INT_FLAG_IOC = CLEAR;*/           /* IOCFはIOC割り込みのステータスビット IOCの割り込みフラグは、上記関数内で個別にクリア */
        IOCIE = SET;
    }
#endif

    if( INT_FLAG_TMR5_OVF == SET )
    { /* IOCでduty検知できなかった場合のみここに来る */
        func_rc_g_duty_detect_timeout();    /*ラジコンプロポ duty取得処理 タイムアウト */
        INT_FLAG_TMR5_OVF = CLEAR;
    }
    
    if( INT_FLAG_TMR1_GATE_CLOSE == SET )
    { /* モータ回転数 取得用割り込み */
        func_int_s_timer1_gate_close();
        INT_FLAG_TMR1_GATE_CLOSE = CLEAR;
    }

    if( INT_FLAG_TMR1_OVERFLOW == SET )
    { /* モータ回転数 未検出割り込み */
        /* TMR1のゲートオフ中はカウントが進まないので、TMRxGIF発生後直後にここにきて取得したキャプチャが無効な値として扱われることはない。 */
        /* ただし回転検出そのものが１周期おきなので、回転検出の精度自体が下がる。 */
        func_int_s_timer1_overflow();
        INT_FLAG_TMR1_OVERFLOW = CLEAR;
    }
    
    if( INT_FLAG_TMR3_GATE_CLOSE == SET )
    { /* 1次側ギヤ回転数 取得用割り込み */
        func_int_s_timer3_gate_close();
        INT_FLAG_TMR3_GATE_CLOSE = CLEAR;
    }

    if( INT_FLAG_TMR3_OVERFLOW == SET )
    { /* 1次側ギヤ回転数 未検出割り込み */
        func_int_s_timer3_overflow();
        INT_FLAG_TMR3_OVERFLOW = CLEAR;
    }
}


/**************************************************************/
/*  Function:                                                 */
/*  割り込み関連のmain-loop処理                                 */
/*                                                            */
/**************************************************************/
void func_int_g_main( void )
{
    ;
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

/* タイマ1ゲート 閉鎖割り込み */
static void func_int_s_timer1_gate_close( void )
{
    u16 u16_data_buff;
    u16_data_buff = (u16)0;

    u16_data_buff = TMR1;
    func_speedsens_g_collect_capture( u16_data_buff, &speedsens_status[ SPEEDSENS_CH_MTR ] );
    TMR1 = (u16)0;              /* タイマクリア ※タイマゲートオフ区間のはずなので、ここで１回クリアするだけでOK */
}

/* タイマ1ゲート 閉鎖割り込み */
static void func_int_s_timer3_gate_close( void )
{
    u16 u16_data_buff;
    u16_data_buff = (u16)0;

    u16_data_buff = TMR3;
    func_speedsens_g_collect_capture( u16_data_buff, &speedsens_status[ SPEEDSENS_CH_1STGEAR ] );
    TMR3 = (u16)0;              /* タイマクリア ※タイマゲートオフ区間のはずなので、ここで１回クリアするだけでOK */
}


/* @@オーバーフロー側はデバッグでのデバッグが厳しい */
/* 評価する方法考えてみる。　動き的には多分大丈夫そうだが。 */
/* タイマ1 オーバーフロー割り込み */
static void func_int_s_timer1_overflow( void )
{
    func_speedsens_g_reset_capture_sts( &speedsens_status[ SPEEDSENS_CH_MTR ] );
    speedsens_status[ SPEEDSENS_CH_MTR ] .u8_cap_timer_reload = SET;                /* リロードは上記初期化関数内でSETしてないので、ここで設定 */
}


/* タイマ3 オーバーフロー割り込み */
static void func_int_s_timer3_overflow( void )
{
    func_speedsens_g_reset_capture_sts( &speedsens_status[ SPEEDSENS_CH_1STGEAR ] );
    speedsens_status[ SPEEDSENS_CH_1STGEAR ] .u8_cap_timer_reload = SET;                /* リロードは上記初期化関数内でSETしてないので、ここで設定 */
}
