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

/* 定義 */
#define SEQ_HBRIDGE_MODE_FREERUN         ((u8)0)
#define SEQ_HBRIDGE_MODE_BRAKE           ((u8)1)
#define SEQ_HBRIDGE_MOED_DRIVE_CW        ((u8)2)
#define SEQ_HBRIDGE_MOED_DRIVE_CCW       ((u8)3)

/* パラメータ */
#define U16_HBRIDGE_DIR_INVERT_WAIT             ((u16)1000)

/* 関数プロトタイプ宣言 */
static void func_inverter_s_duty_area_check( void );
static void func_inverter_s_h_bridge_dir_set( u8 u8_state_req );
static void func_inverter_s_h_bridge_duty_output( u16 u16_duty );

/* グローバル変数 */
u8  u8_hbridge_g_output_state_request;          /* 出力方向 (要求) */
u16 u16_hbridge_g_duty_output_request;          /* 出力duty (要求) */        /* 回転数制御でも使うので、0~100だとちょっと精度が足りない。->RC_DUTYとは別のdutyを定義する */


/* 静的変数 */
static u8 u8_hbridge_s_sequence;                /* Hブリッジシーケンス */
static u8 u8_hbridge_s_output_dir;              /* 出力方向 */
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
    u8_hbridge_s_sequence = SEQ_HBRIDGE_MODE_FREERUN;

    u8_hbridge_s_dir_req = FORWARD;
    u8_hbridge_s_dir_before = FORWARD;
    u16_hbridge_g_duty_output_request = INV_DUTY_0P;
    u16_hbridge_s_dir_continue_cnt = (u16)0;
}

/**************************************************************/
/*  Function:                                                 */
/*  インバータ制御 メインループ処理                              */
/*                                                            */
/**************************************************************/
void func_hbridge_g_main( void )
{
    /* 関数呼び出しでネスト深くする構造の方が、ぱっと見はどこで何の変数更新してるかわかるのでわかりやすいかも・・・ */
    /* 外部関数を複数処理で更新、それぞれのファイル内で実行タイミングを制御する形もアリかな～と思ったが・・・ */
    /* いろいろ構成考えてみたい。 */
    func_hbridge_s_get_request();                           /* 他のグローバル関数で更新した出力要求を反映させる */
                                                            /* 以降、このグローバル関数内で操作する形にする */
    func_inverter_s_control_sequence();                     /* Hブリッジ動作 制御シーケンス */
    
    
    func_inverter_s_h_bridge_dir_set();       /* Hブリッジ動作方向設定 */
    func_inverter_s_h_bridge_duty_output();       /* Hブリッジduty設定    */
}

/**************************************************************/
/*  Function:                                                 */
/*  Hブリッジ動作遷移 制御シーケンス                            */
/*  禁止操作とかはこの処理内で設定する                           */
/**************************************************************/
static void func_inverter_s_control_sequence( void )
{
    switch ( u8_hbridge_s_sequence )
    {
        case SEQ_HBRIDGE_MODE_FREERUN:
            /* 初回はここから開始 */
            
            break;
        case SEQ_HBRIDGE_MODE_BRAKE:
            
            break;
            
        case SEQ_HBRIDGE_MOED_DRIVE_CW:
            
            break;
            
        case SEQ_HBRIDGE_MOED_DRIVE_CCW:
            
            break;
        
        default:
            break;
    }
}



/**************************************************************/
/*  Function:                                                 */
/*  インバータ制御 要求反映処理                                  */
/*  外部で変更するグローバル変数の内容は、この関数内で明示的に代入! */
/**************************************************************/
static void func_hbridge_s_get_request( void )
{

    u8_hbridge_s_output_dir = u8_hbridge_g_output_state_request;
    u16_hbridge_s_duty_output = u16_hbridge_g_duty_output_request;
}

/**************************************************************/
/*  Function:                                                 */
/*  インバータ制御 制御用dutyの範囲チェック処理                  */
/*  基本的に外部の関数でdutyを計算させるが、範囲がバグってないかチェック*/
/**************************************************************/
static void func_inverter_s_duty_area_check( void )
{
    if( u16_hbridge_g_duty_output_request > INV_DUTY_100P )
    {
        u16_hbridge_g_duty_output_request = INV_DUTY_100P;
    }
    else if( u16_hbridge_g_duty_output_request < INV_DUTY_0P )
    {
        u16_hbridge_g_duty_output_request = INV_DUTY_0P;
    }
    else
    {
        ;
    }
}

/**************************************************************/
/* Function:                                                  */
/* Hブリッジ回路の動作方向設定関数                               */
/**************************************************************/
static void func_inverter_s_h_bridge_dir_set( u8 u8_state_req )
{
    /* timer driver はレジスタ操作のみ、hbridge.c内では、インバータ制御に関する制約を吸収する関数、speedcontrolは何も考えずに出力を設定する層にしたい。 */
    /* 方向切り替えの制約あるかな？ あるわ。 */
    
    /* 動作方向継続カウンタ処理 */
    if( u16_hbridge_s_dir_continue_cnt < U16_MAX )
    {
        u16_hbridge_s_dir_continue_cnt++;
    }
    
    if( u8_state_req != u8_hbridge_s_dir_before )
    { /* 前回までに設定した動作方向と同じ */
        u16_hbridge_s_dir_continue_cnt = (u16)0;
        u8_hbridge_s_dir_req = u8_state_req;                  /* 変更後の設定要求を保存 */
        u8_hbridge_s_dir_before = u8_state_req;               /* 今回更新された値を保存 */
    }
    
    if( u16_hbridge_s_dir_continue_cnt > U16_HBRIDGE_DIR_INVERT_WAIT )
    { /* 動作 */

    }
    td_g_cwg1_mode_full_bridge_dir_set( u8_state_req );
}


/**************************************************************/
/* Function:                                                  */
/* Hブリッジ回路への出力duty設定関数                             */
/**************************************************************/
static void func_inverter_s_h_bridge_duty_output( u16 u16_duty )
{
    U16_HBRIDGE_DIR_INVERT_WAIT
}

