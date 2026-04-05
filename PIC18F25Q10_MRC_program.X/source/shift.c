/*
 * File:   shift.c
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
#include "radio_control.h"
#include "./tools/speedsens.h"
#include "./speedcontrol.h"

#define SHIFT_DRIVE_STOP_CNT                ((u8)10)     /* 10ms */
#define SHIFT_DRIVE_STOP_SPEED              ((u16)0)     /* 0rpm */

#define SHIFT_SEQ_CLUTCH_OFF_STOP               ((u8)0)      /* 停止状態でクラッチOFF */
#define SHIFT_SEQ_CLUTCH_MEETING                ((u8)1)
#define SHIFT_SEQ_CLUTCH_ON_DRIVE               ((u8)2)
#define SHIFT_SEQ_CLUTCH_OFF_BFORE_CHG          ((u8)3)
#define SHIFT_SEQ_CLUTCH_OFF_SHIFT_CHG          ((u8)4)
#define SHIFT_SEQ_CLUTCH_OFF_AFTER_CHG          ((u8)5)
#define SHIFT_SEQ_CLUTCH_OFF_KICK               ((u8)6)

/* 時間定義 */
#define SHIFT_CLUTCH_OFF_TIME               ((u16)100)
#define SHIFT_CLUTCH_OFF_TIME_KICK          ((u16)200)
#define SHIFT_CLUTCH_MEET_TIME              ((u16)2000)

/* クラッチ制御用定義 */
#define SHIFT_SERVO_ANGLE_CLUTCH_ON         SERVO_DEG_IDX__85
#define SHIFT_SERVO_ANGLE_CLUTCH_OFF        SERVO_DEG_IDX__0

/* シフトチェンジの制御を担当 */

/* 関数プロトタイプ宣言 */
static void func_shift_s_shift_mode_decide( void );
static void func_shift_s_shift_degree_calc( void );
static void func_shift_s_shift_position_req_decide( void );
static void func_shift_s_clutch_control( void );
static void func_shift_s_shifting_control( void );
static void func_shift_s_shift_sequence( void );

static u8 u8_shift_s_shift_chg_enable_wait_cnt;
static u8 u8_shift_s_shift_mode_req;
static u8 u8_shift_s_blip_req;
static u8 u8_shift_s_shifting_status;
static u16 u16_shift_s_clutch_meet_cnt;
static u8 u8_shift_s_clutch_meet_angle;
static u8 u8_shift_s_clutch_meet_complete;

u8 u8_shift_g_shift_mode;
u8 u8_shift_g_shift_position_req;
u8 u8_shift_g_shift_position_output;

u16 u16_shift_s_clutch_off_cnt;


/* グローバル変数 */
/**************************************************************/
/*  Function:                                                 */
/*  初期化関数                                                 */
/*                                                            */
/*                                                            */
/**************************************************************/
void func_shift_g_init( void )
{
    u8_shift_s_shift_mode_req = SHIFT_MODE_MANUAL;
    u8_shift_g_shift_mode = SHIFT_MODE_MANUAL;
    u8_shift_g_shift_position_req = SHIFT_POSI_0;
    u8_shift_g_shift_position_output = SHIFT_POSI_0;
    u8_shift_s_shift_chg_enable_wait_cnt = (u8)0;
    u8_shift_s_shifting_status = SHIFT_SEQ_CLUTCH_OFF_STOP;
    u16_shift_s_clutch_off_cnt = (u16)0;
    u16_shift_s_clutch_meet_cnt = (u16)0;
    u8_shift_s_clutch_meet_angle = (u8)0;
    u8_shift_s_clutch_meet_complete = CLEAR;
}

/**************************************************************/
/*  Function:                                                 */
/*  main関数                                                   */
/*                                                            */
/*                                                            */
/**************************************************************/
void func_shift_g_main( void )
{
    /* 共通入力判定 */
    func_shift_s_shift_mode_decide();           /* 変則モード確定処理 */
    func_shift_s_shift_position_req_decide();   /* シフト位置要求設定処理 */

    func_shift_s_shift_sequence();              /* 変速制御 */

    /* 出力制御 */
    func_shift_s_clutch_control();              /* クラッチ制御 */
    func_shift_s_shifting_control();            /* シフト位置出力処理 */
}

/**************************************************************/
/*  Function:                                                 */
/*  関数                                                 */
/*                                                            */
/*                                                            */
/**************************************************************/
static void func_shift_s_shift_position_req_decide( void )
{
    if( u8_shift_g_shift_mode == SHIFT_MODE_MANUAL )
    { /* マニュアルシフト */
        if( ( gpio_g_paddle_shift_sw.u8_state == LOW ) &&
            ( gpio_g_paddle_shift_sw.u8_state_bf == MID ))
        { /* シフトアップ */
            if( u8_shift_g_shift_position_req < SHIFT_POSI_7 )
            {
                u8_shift_g_shift_position_req++;
            }
            else
            {
                u8_shift_g_shift_position_req = SHIFT_POSI_7;
            }
        }
        else if( ( gpio_g_paddle_shift_sw.u8_state == HI ) &&
                 ( gpio_g_paddle_shift_sw.u8_state_bf == MID ))
        { /* シフトダウン */
            if( u8_shift_g_shift_position_req > SHIFT_POSI_1 )
            {
                u8_shift_g_shift_position_req--;
            }
            else
            {
                u8_shift_g_shift_position_req = SHIFT_POSI_0;
            }
        }
        else
        {
            ;               /* 現在のシフト位置を維持 */
        }
    }
    else if( u8_shift_g_shift_mode == SHIFT_MODE_AUTOMATIC )
    {

    }
    else
    {
        ;
    }
}


/**************************************************************/
/*  Function:                                                 */
/*  シフト位置出力処理                                          */
/*                                                            */
/**************************************************************/
static void func_shift_s_shifting_control( void )
{
    if( u8_shift_s_shifting_status == SHIFT_SEQ_CLUTCH_OFF_SHIFT_CHG )
    { /* シフトチェンジ有効 シーケンス内だお!：クラッチOFF中 */
        u8_shift_g_shift_position_output = u8_shift_g_shift_position_req;
    }

    /* 0bit目 */
    if( ( u8_shift_g_shift_position_output & ((u8)0x01) ) != (u8)0 )
    { /* 0bit目が立っている */
        U8_GPIO_G_OUT_SHIFT_0 = SET;
    }
    else
    {
        U8_GPIO_G_OUT_SHIFT_0 = CLEAR;
    }

    /* 1bit目 */
    if( ( u8_shift_g_shift_position_output & ((u8)0x02) ) != (u8)0 )
    { /* 0bit目が立っている */
        U8_GPIO_G_OUT_SHIFT_1 = SET;
    }
    else
    {
        U8_GPIO_G_OUT_SHIFT_1 = CLEAR;
    }

    /* 2bit目 */
    if( ( u8_shift_g_shift_position_output & ((u8)0x04) ) != (u8)0 )
    { /* 0bit目が立っている */
        U8_GPIO_G_OUT_SHIFT_2 = SET;
    }
    else
    {
        U8_GPIO_G_OUT_SHIFT_2 = CLEAR;
    }
}


/**************************************************************/
/*  Function:                                                 */
/*  変速モード確定関数                                          */
/*                                                            */
/**************************************************************/
static void func_shift_s_shift_mode_decide( void )
{
    u8 u8_mode_before;

    u8_mode_before = u8_shift_g_shift_mode;

    if( u8_shift_s_shift_chg_enable_wait_cnt < U8_MAX )
    {
        u8_shift_s_shift_chg_enable_wait_cnt++;
    }
    
    /* 変速モード変更要求 */
    if( gpio_g_shift_mode_sw.u8_state == HI )
    {
        u8_shift_s_shift_mode_req = SHIFT_MODE_MANUAL;
    }
    else
    {
        u8_shift_s_shift_mode_req = SHIFT_MODE_AUTOMATIC;
    }

    /* 停止状態でのみ、シフト操作モードの変更を許可する */
    if( ( u8_shift_s_shift_chg_enable_wait_cnt >= SHIFT_DRIVE_STOP_CNT ) &&
        ( u16_speedsens_g_speed_ave_1stgear == SHIFT_DRIVE_STOP_SPEED ) )               /* 1次ギヤで止まってる判定する場合、0,1,2のシフトレバーが接続状態でないとダメなので、条件としては微妙かも */
    { /* 現在車は停止している */
        u8_shift_g_shift_mode = u8_shift_s_shift_mode_req;          /* 現在の変速モード要求を反映する */
    }

    if( u8_mode_before != u8_shift_s_shift_chg_enable_wait_cnt )
    { /* 変速モードに変化があった */
        u8_shift_s_shift_chg_enable_wait_cnt = (u8)0;       /* クリア */
    }
}


/**************************************************************/
/*  Function:                                                 */
/*  クラッチサーボ制御状態更新処理                               */
/*                                                            */
/**************************************************************/
static void func_shift_s_shift_sequence( void )
{
    switch ( u8_shift_s_shifting_status )
    {
    case SHIFT_SEQ_CLUTCH_OFF_STOP:
        if( u8_sc_s_throttle_dir != SC_THROTTLE_DIR_NONE )
        { /* スロットル中立ではなくなった */
            u8_shift_s_shifting_status = SHIFT_SEQ_CLUTCH_MEETING;                  /* クラッチミート開始 */
        }

        /* 停止状態でも変速は許可する ※ただしニュートラル */
        if( u8_shift_g_shift_position_output != u8_shift_g_shift_position_req )
        { /* シフトポジション要求が変化 */
            u8_shift_s_shifting_status = SHIFT_SEQ_CLUTCH_OFF_BFORE_CHG;            /* クラッチOFFへ */
        }
        break;

    case SHIFT_SEQ_CLUTCH_MEETING:
        /* 始動時のクラッチミート */
        if( u8_shift_s_clutch_meet_complete == SET )
        { /* 半クラッチ完了 */
            u8_shift_s_shifting_status = SHIFT_SEQ_CLUTCH_ON_DRIVE;
        }
        break;
    
    case SHIFT_SEQ_CLUTCH_ON_DRIVE:
        u16_shift_s_clutch_off_cnt = (u16)0;

        /* 駆動中は常時変速許可 */
        if( u8_shift_g_shift_position_output != u8_shift_g_shift_position_req )
        { /* シフトポジション要求が変化 */
            u8_shift_s_shifting_status = SHIFT_SEQ_CLUTCH_OFF_BFORE_CHG;            /* クラッチOFFへ */
        }

        if( u16_speedsens_g_speed_ave_1stgear == (u16)0 )
        { /* 車が完全に停止した */
            u8_shift_s_shifting_status = SHIFT_SEQ_CLUTCH_OFF_STOP;             /* クラッチ切る */
        }
#if 0
        /* クラッチキック要求 */
        else if(  )
        {
            u8_shift_s_shifting_status = SHIFT_SEQ_CLUTCH_OFF_KICK;            /* クラッチOFFへ */
        }
#endif
        break;
    
    case SHIFT_SEQ_CLUTCH_OFF_BFORE_CHG:
        /* シフトチェンジ前のクラッチOFF */
        if( u16_shift_s_clutch_off_cnt < U16_MAX )
        {
            u16_shift_s_clutch_off_cnt++;
        }

        if( u16_shift_s_clutch_off_cnt > SHIFT_CLUTCH_OFF_TIME )
        { /* クラッチOFFを最低維持する時間経過：サーボの動作応答を待つ */
            u8_shift_s_shifting_status = SHIFT_SEQ_CLUTCH_OFF_SHIFT_CHG;
            u16_shift_s_clutch_off_cnt = (u16)0;
        }
        break;

    case SHIFT_SEQ_CLUTCH_OFF_SHIFT_CHG:
        /* シフトチェンジ */
        /*u8_shift_g_shift_position_output = u8_shift_g_shift_position_req;*/       /* シフト要求を反映 */      /* func_shift_s_shifting_control() 側で制御 */
        u8_shift_s_shifting_status = SHIFT_SEQ_CLUTCH_OFF_AFTER_CHG;
        break;

    case SHIFT_SEQ_CLUTCH_OFF_AFTER_CHG:
        /* シフトチェンジ後のクラッチOFF */
        if( u16_shift_s_clutch_off_cnt < U16_MAX )
        {
            u16_shift_s_clutch_off_cnt++;
        }
        if( u16_shift_s_clutch_off_cnt > SHIFT_CLUTCH_OFF_TIME )
        { /* クラッチOFFを最低維持する時間経過：サーボの動作応答を待つ */
            if( u16_speedsens_g_speed_ave_1stgear == (u16)0 )
            { /* 車が完全に停止した状態での変速だった */
                u8_shift_s_shifting_status = SHIFT_SEQ_CLUTCH_OFF_STOP;             /* クラッチ切った状態での停止状態へ戻る */
            }
            else if( u16_speedsens_g_speed_ave_1stgear < (u16)1000 )
            { /* 車が動作中の変速ではあるが、あまり速度が出ていない */
                u8_shift_s_shifting_status = SHIFT_SEQ_CLUTCH_MEETING;                  /* 半クラッチで繋ぐ */
            }
            else
            { /* 速度が十分に出た状態での変速 */
                u8_shift_s_shifting_status = SHIFT_SEQ_CLUTCH_ON_DRIVE;                 /* クラッチ繋ぐ */
            }
        
            u16_shift_s_clutch_off_cnt = (u16)0;
        }
        break;

    case SHIFT_SEQ_CLUTCH_OFF_KICK:
        /* 過負荷気味なときのクラッチキック操作 */
        if( u16_shift_s_clutch_off_cnt < U16_MAX )
        {
            u16_shift_s_clutch_off_cnt++;
        }
        if( u16_shift_s_clutch_off_cnt > SHIFT_CLUTCH_OFF_TIME_KICK )
        {
            u8_shift_s_shifting_status = SHIFT_SEQ_CLUTCH_MEETING;
            u16_shift_s_clutch_off_cnt = (u16)0;
        }

        break;

    default:
        break;
    }
}

/**************************************************************/
/*  Function:                                                 */
/*  クラッチサーボ制御処理                                      */
/*                                                            */
/**************************************************************/
static void func_shift_s_clutch_control( void )
{
    u8 u8_clutch_angle_req;
    u8 u8_clutch_meet_angle_idx_diff;
    u8 u8_clutch_meet_dir_prop;
    u8 u8_clutch_servo_angle_idx_result;
    u16 u16_clutch_meet_time_base;
    u16 u16_clutch_meet_time_gap;
    u32 u32_calc_buff;

    /* 初期化 */
    u8_clutch_angle_req = (u8)0;
    u8_clutch_meet_angle_idx_diff = (u8)0;
    u8_clutch_meet_dir_prop = SET;
    u8_clutch_servo_angle_idx_result = (u8)0;
    u16_clutch_meet_time_base = (u16)0;
    u16_clutch_meet_time_gap  = (u16)0;
    u32_calc_buff = (u32)0;


    /*-----------------------------------------------------------------------*/
    switch ( u8_shift_s_shifting_status )
    {
        case SHIFT_SEQ_CLUTCH_OFF_STOP:
        case SHIFT_SEQ_CLUTCH_OFF_BFORE_CHG:
        case SHIFT_SEQ_CLUTCH_OFF_AFTER_CHG:
        case SHIFT_SEQ_CLUTCH_OFF_SHIFT_CHG:
        case SHIFT_SEQ_CLUTCH_OFF_KICK:
            u16_shift_s_clutch_meet_cnt = (u16)0;
            u8_shift_s_clutch_meet_complete = CLEAR;
            u8_clutch_servo_angle_idx_result = SHIFT_SERVO_ANGLE_CLUTCH_OFF;
            break;

        case SHIFT_SEQ_CLUTCH_MEETING:
            /* クラッチ角度を時間基準で制御する */
            if( u16_shift_s_clutch_meet_cnt < U16_MAX )
            {
                u16_shift_s_clutch_meet_cnt++;
            }
            u16_clutch_meet_time_base = SHIFT_CLUTCH_MEET_TIME;
            u16_clutch_meet_time_gap = u16_shift_s_clutch_meet_cnt;

            /* 角度基準 ※角度idx基準 */
            if( SHIFT_SERVO_ANGLE_CLUTCH_ON > SHIFT_SERVO_ANGLE_CLUTCH_OFF )
            { /* ONで角度が増える方向 */
                u8_clutch_meet_angle_idx_diff = SHIFT_SERVO_ANGLE_CLUTCH_ON - SHIFT_SERVO_ANGLE_CLUTCH_OFF;
                u8_clutch_meet_dir_prop = SET;              /* 比例増加 */
            }
            else
            { /* ONで角度が減る方向 */
                u8_clutch_meet_angle_idx_diff = SHIFT_SERVO_ANGLE_CLUTCH_OFF - SHIFT_SERVO_ANGLE_CLUTCH_ON;
                u8_clutch_meet_dir_prop = CLEAR;            /* 比例減少 */
            }

            /* 基準に対する現在の制御量を取得 */
            u32_calc_buff = func_ud_g_calcmul_2x2_byte( u16_clutch_meet_time_gap, (u16)u8_clutch_meet_angle_idx_diff );
            u32_calc_buff = func_ud_g_calcdiv_4x4_byte( u32_calc_buff, (u32)u16_clutch_meet_time_base );

            /* 出力 */
            if( u8_clutch_meet_dir_prop == SET )
            { /* 時間に比例して角度上げる */
                u8_clutch_servo_angle_idx_result = SHIFT_SERVO_ANGLE_CLUTCH_OFF + (u8)u32_calc_buff;
            }
            else
            { /* 時間に比例して角度を下げる */
                u8_clutch_servo_angle_idx_result = SHIFT_SERVO_ANGLE_CLUTCH_OFF - (u8)u32_calc_buff;
            }

            /* クラッチミート完了判定 */
            if( u16_shift_s_clutch_meet_cnt > SHIFT_CLUTCH_MEET_TIME )
            { /* クラッチ結合時間経過 */
                u8_shift_s_clutch_meet_complete = SET;
            }
            break;

        case SHIFT_SEQ_CLUTCH_ON_DRIVE:
            u8_clutch_servo_angle_idx_result = SHIFT_SERVO_ANGLE_CLUTCH_ON;
            break;

        default:
            u8_clutch_servo_angle_idx_result = SHIFT_SERVO_ANGLE_CLUTCH_OFF;
            break;
    }

    /* サーボ角度出力 */
    servo_s_angle_set( u8_clutch_servo_angle_idx_result ,SERVO_CLUTCH );
}
