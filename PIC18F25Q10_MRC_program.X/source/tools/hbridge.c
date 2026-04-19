/*
 * File:   inverter.c
 * Author: ICE_MEGANE
 *
 * Created on 2026/04/12, 14:38
 */

/* toolとして簡単に呼び出して使える形にしたいので、関数内でシーケンス呼び出し・・・？ */
/* きれいに作るには最初の思想が大事だと思われる。 */

#include <xc.h>
#include "../userdefine.h"
#include "../tools/hbridge.h"
#include "../mcufunc/timer_driver.h"

/* 定義 */
#define SEQ_HBRIDGE_MODE_FREERUN         ((u8)0)
#define SEQ_HBRIDGE_MODE_BRAKE           ((u8)1)
#define SEQ_HBRIDGE_MODE_DRIVE_CW        ((u8)2)
#define SEQ_HBRIDGE_MODE_DRIVE_CCW       ((u8)3)

#define SEQ_HBRIDGE_STATE_CHG_ENABLE     ((u8)0)
#define SEQ_HBRIDGE_STATE_CHG_DISABLE    ((u8)1)

/* パラメータ */
#define U16_HBRIDGE_DIR_SET_WAIT             ((u16)1000)

/* 関数プロトタイプ宣言 */
static u16 func_inverter_s_duty_area_check( u16 u16_duty_req );
static void func_hbridge_g_h_bridge_dir_set( u8 u8_state_req );
static void func_inverter_s_h_bridge_duty_update( u16 u16_duty );


/* グローバル変数 */
u16 u16_hbridge_g_output_duty;          /* 出力duty (要求) */        /* 回転数制御でも使うので、0~100だとちょっと精度が足りない。->RC_DUTYとは別のdutyを定義する */


/* 静的変数 */
static u32 u32_hbridge_s_sequence_cnt;          /* Hブリッジシーケンスカウント */
static u8 u8_hbridge_s_sequence;                /* Hブリッジシーケンス */
static u8 u8_hbridge_s_sequence_1ms_before;         /* Hブリッジシーケンス 前回 */
static u8 u8_hbridge_s_output_state_req;        /* 出力方向 */
static u16 u16_hbridge_s_duty_output;           /* 出力duty */


static u8 u8_hbridge_s_dir_req;                 /* Hブリッジ動作方向 現在 */
static u8 u8_hbridge_s_dir_before;              /* Hブリッジ動作方向 前回 */
static u16 u16_hbridge_s_dir_continue_cnt;      /* 動作方向 一定カウント */

/**************************************************************/
/*  Function:                                                 */
/*  インバータ制御 初期化処理                                   */
/*                                                            */
/**************************************************************/
void func_inverter_g_init( void )
{
    u32_hbridge_s_sequence_cnt = (u32)0;
    u8_hbridge_s_sequence = SEQ_HBRIDGE_STATE_CHG_ENABLE;
    u8_hbridge_s_sequence_1ms_before = SEQ_HBRIDGE_MODE_FREERUN;

    u8_hbridge_s_dir_req = FORWARD;
    u8_hbridge_s_dir_before = FORWARD;
    u16_hbridge_g_output_duty = HBRIDGE_DUTY_0P;
    u16_hbridge_s_dir_continue_cnt = (u16)0;
}

/**************************************************************/
/*  Function:                                                 */
/*  インバータ制御 メインループ処理                              */
/*                                                            */
/**************************************************************/
void func_hbridge_g_main( void )
{                                                            /* 以降、このグローバル関数内で操作する形にする */
    ;
}

/**************************************************************/
/*  Function:                                                 */
/*  duty更新 外部関数                                          */
/*                                                            */
/**************************************************************/
void func_hbridge_control_set( u8 u8_state_req, u16 u16_duty_req )
{  
    func_hbridge_g_h_bridge_dir_set( u8_state_req );          /* Hブリッジ動作方向設定 ※要求ではなく、シーケンス遷移後の確定方向を返す */
    func_inverter_s_h_bridge_duty_update( u16_duty_req );       /* Hブリッジduty設定    */
}

/**************************************************************/
/* Function:                                                  */
/* Hブリッジ回路の動作方向設定関数                               */
/**************************************************************/
static void func_hbridge_g_h_bridge_dir_set( u8 u8_state_req )
{
    if( u8_state_req == HBRIDGE_OUTPUT_BRAKE )
    { /* ブレーキの設定要求 */
        td_g_cwg1_mode_full_bridge_brake();
    }
    else
    { /* ブレーキ以外の設定要求：駆動要求 */
        td_g_cwg1_mode_full_bridge_drive_dir_set( u8_state_req );
    }
}


/**************************************************************/
/* Function:                                                  */
/* Hブリッジ回路への出力duty設定関数                             */
/* 注意：ブレーキ設定中の場合は、CWG変調は受け付けられないのでduty設定は無効化される */
/**************************************************************/
static void func_inverter_s_h_bridge_duty_update( u16 u16_duty )
{
    u16 u16_output_duty;

    u16_output_duty = func_inverter_s_duty_area_check( u16_duty );
    td_g_pwm4_pwm_duty_set( u16_output_duty );
}

/**************************************************************/
/*  Function:                                                 */
/*  インバータ制御 制御用dutyの範囲チェック処理                  */
/*  基本的に外部の関数でdutyを計算させるが、範囲がバグってないかチェック*/
/**************************************************************/
static u16 func_inverter_s_duty_area_check( u16 u16_duty_req )
{
    if( u16_duty_req > HBRIDGE_DUTY_100P )
    {
        u16_duty_req = HBRIDGE_DUTY_100P;
    }
    else if( u16_duty_req < HBRIDGE_DUTY_0P )
    {
        u16_duty_req = HBRIDGE_DUTY_0P;
    }
    else
    {
        ;
    }
    
    return u16_duty_req;
}

