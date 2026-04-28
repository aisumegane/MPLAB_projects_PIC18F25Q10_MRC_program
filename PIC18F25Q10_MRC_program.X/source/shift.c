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


/* 変速状態定義 */
#define SHIFT_POSITION_UP                   ((u8)0)
#define SHIFT_POSITION_DOWN                 ((u8)1)
#define SHIFT_POSITION_STABLE               ((u8)2)      /* シフト位置 変化なし */

#define SHIFT_DRIVE_STOP_CNT                ((u8)10)     /* 10ms */
#define SHIFT_DRIVE_STOP_SPEED              ((u16)0)     /* 0rpm */

/* 時間定義 */
#define SHIFT_CLUTCH_ON_TO_OFF_WAIT_TIME    ((u16)100)
#define SHIFT_CLUTCH_OFF_TO_ON_WAIT_TIME    ((u16)50)
#define SHIFT_CLUTCH_OFF_TIME_KICK          ((u16)200)
#define SHIFT_CLUTCH_MEET_TIME              ((u16)1000)
#define SHIFT_CLUTCH_OFF_BY_STOP_TIME       ((u16)400)             /* 車が動いてないときのクラッチオフ(アイドリング移行用) 判断時間 */      /* 速度をどれだけ拾えるかに依存してくる。　ループタスク側で割り込みが何ms置きに来てるかみて低速まで検知できるようにしたほうがよさそう @@ */

/* クラッチ制御用定義 */
#define SHIFT_SERVO_ANGLE_CLUTCH_ON         SERVO_DEG_IDX__85
#define SHIFT_SERVO_ANGLE_CLUTCH_OFF        SERVO_DEG_IDX__0

/* オートマチック変速定義 */
/* ※メインMCU側では0~7の計8ポジションで指令を送っているため、idxは0始まり... */
#define SHIFT_POSI_THRESHOLD_NUM            ((u8)8)
#define SHIFT_AUTOMATIC_START_SHIFT_POSI    SHIFT_POSI_1            /* オートマは1速発進、空ぶかしなし */

/* マニュアル変速定義 */
#define SHIFT_MANUAL_START_SHIFT_POSI       SHIFT_POSI_0            /* 停止状態から動き出す際のギヤ位置は、初回は空ぶかし可能設定 */

/* ブリッピング制御定義 */
/* ブリッピングステータス */
#define SHIFT_BLIP_STATUS_WAITING           ((u8)0)                 /* ブリッピング待機中 */
#define SHIFT_BLIP_STATUS_COMPLETE_MATCH    ((u8)1)                 /* 回転数一致によりブリッピング完了 */
#define SHIFT_BLIP_STATUS_COMPLETE_TIMEOUT  ((u8)2)                 /* タイムアウトによりブリッピング完了 */
/*  */
#define SHIFT_BLIP_SPEED_ACCURACY           ((u16)100)              /* ブリッピングの正確さ 実写でいうとシフトチェンジのヘタクソ具合 */
#define SHIFT_BLIP_TIMEOUT                  ((u16)1000)         /*  */

/* 変速基準回転数定義 */
/* 実機のトルクを見て決めたい */
/* あんまり速度上がらない気がする */
/* ➡方針決め：インバータへの印加duty50%の時の出力で得られる回転数を閾値として設定する */
/* バッテリ電圧がなくなってきたときの対応はどうするか？上の方へはシフトできない仕様でOK？ */
#define SHIFT_POSI_0_APPR_RPM       ((u16)0)        /* ニュートラル */
#define SHIFT_POSI_1_APPR_RPM       ((u16)1000)
#define SHIFT_POSI_2_APPR_RPM       ((u16)2000)
#define SHIFT_POSI_3_APPR_RPM       ((u16)3000)
#define SHIFT_POSI_4_APPR_RPM       ((u16)4000)
#define SHIFT_POSI_5_APPR_RPM       ((u16)5000)
#define SHIFT_POSI_6_APPR_RPM       ((u16)8000)
#define SHIFT_POSI_7_APPR_RPM       ((u16)11000)
#define SHIFT_POSI_8_APPR_RPM       ((u16)14000)

#define SHIFT_POSI_CHG_HIS_MAX      ((u16)200)                /* 変速のヒステリシス最大 回転数[rpm] */

#define SHIFT_POSI_REMAIN_CNT       ((u16)300)       /* サーボの稼働累計時間よりは長くしないとダメ。 */


/* 変速比設定 (ギヤ比基準) */
/* 実際は小数なので、桁ましのために*10000 */
#define SHIFT_GEAR_RATIO_POSI_0     ((u16)1)        /* ニュートラル */
#define SHIFT_GEAR_RATIO_POSI_1     ((u16)512)
#define SHIFT_GEAR_RATIO_POSI_2     ((u16)718)
#define SHIFT_GEAR_RATIO_POSI_3     ((u16)1340)
#define SHIFT_GEAR_RATIO_POSI_4     ((u16)1879)
#define SHIFT_GEAR_RATIO_POSI_5     ((u16)3200)
#define SHIFT_GEAR_RATIO_POSI_6     ((u16)4488)
#define SHIFT_GEAR_RATIO_POSI_7     ((u16)8374)
#define SHIFT_GEAR_RATIO_POSI_8     ((u16)11743)


/* だめかも・・・↑この回転数は現在のシフト位置に対して、どこまで上げられるのかギヤ費によって決まる。モータの回転数範囲に対して、上の段でどこまで上がるのか・・・・*/
/* ある領域における電圧変化に対してどこまで回転数幅を持たせられるかはどうやって決めるのか？ */
/* ひとまず実装を進めるために暫定値として計算式を立てることにした 2026/04/06 */

/* @@メモ */
/* 変速域内でのdutyはリモコンのレバー位置に応じていつでも0~100で操作可能にする。 */
/* ただし、入力に対して実際の速度が上記のヒステリシス範囲を超えた場合、変速を行う */
/* →この制御が一番単純でいいんじゃないか説 */
/* 本来シフト位置に対してどの程度の回転数域に収めるかは運転者が決めるが、そこの判断をマイコンにやらせる */

/* ・・・この時のヒステリシスをレブリミットと定義するのが適切かも？？？ */
/* １つの変速範囲において、どこまで回転数を上げないと上のシフト位置に遷移できないか、そのレベルを可変にする・・・→あっている気がするンゴ */



/* 関数プロトタイプ宣言 */
static void func_shift_s_shift_mode_decide( void );
static void func_shift_s_shift_degree_calc( void );
static void func_shift_s_shift_position_req_decide( void );
static void func_shift_s_shift_position_decide_mode_manual( void );
static void func_shift_s_shift_position_decide_mode_automatic( void );
static void func_shift_s_shift_dir_judge( void );
static void func_shift_s_blip_control( void );
static void func_shift_s_clutch_control( void );
static void func_shift_s_shift_position_control( void );
static void func_shift_s_shift_sequence( void );

static u8 u8_shift_s_shift_chg_enable_wait_cnt;
static u8 u8_shift_s_shift_mode_req;
static u8 u8_shift_s_blip_req;
u8 u8_shift_g_shifting_sequence;
u8 u8_shift_g_shifting_sequence_before;
static u16 u16_shift_s_clutch_meet_cnt;
static u8 u8_shift_s_clutch_meet_angle;
static u8 u8_shift_s_clutch_meet_complete;

u8 u8_shift_g_shift_mode;
u8 u8_shift_g_shift_position_req;
u8 u8_shift_g_shift_position_output;
static u8 u8_shift_s_shift_dir;
u8 u8_shift_g_blip_complete_status;
static u8 u8_shift_s_shifting_complete;
static u16 u16_shift_s_blip_timeout_cnt;


u16 u16_shift_s_clutch_off_cnt;

static u16 u16_shift_s_auto_position_cnt;

/* パラメータ定義 */
/* 現在のシフト位置で、duty50%で出せる速度設定 */
static const u16 u16_shift_s_shift_posi_50per_rpm_ary[ SHIFT_POSI_NUM ] =
{
    SHIFT_POSI_0_APPR_RPM,
    SHIFT_POSI_1_APPR_RPM,
    SHIFT_POSI_2_APPR_RPM,
    SHIFT_POSI_3_APPR_RPM,
    SHIFT_POSI_4_APPR_RPM,
    SHIFT_POSI_5_APPR_RPM,
    SHIFT_POSI_6_APPR_RPM,
    SHIFT_POSI_7_APPR_RPM,
    SHIFT_POSI_8_APPR_RPM,
};

const u16 u16_shift_s_gear_ratio_ary[ SHIFT_POSI_NUM ] =
{
    SHIFT_GEAR_RATIO_POSI_0,
    SHIFT_GEAR_RATIO_POSI_1,
    SHIFT_GEAR_RATIO_POSI_2,
    SHIFT_GEAR_RATIO_POSI_3,
    SHIFT_GEAR_RATIO_POSI_4,
    SHIFT_GEAR_RATIO_POSI_5,
    SHIFT_GEAR_RATIO_POSI_6,
    SHIFT_GEAR_RATIO_POSI_7,
    SHIFT_GEAR_RATIO_POSI_8,
};


/* 1つの関数の制御対象、目的を広げすぎない設計が望ましい */

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

    u8_shift_g_shifting_sequence = SHIFT_SEQ_CLUTCH_OFF_STOP;
    u8_shift_g_shifting_sequence_before = SHIFT_SEQ_CLUTCH_OFF_STOP;

    u16_shift_s_clutch_off_cnt = (u16)0;
    u16_shift_s_clutch_meet_cnt = (u16)0;
    u8_shift_s_clutch_meet_angle = (u8)0;
    u8_shift_s_clutch_meet_complete = CLEAR;

    u16_shift_s_auto_position_cnt = (u16)0;
    u8_shift_s_shift_dir = SHIFT_POSITION_UP;

    u8_shift_g_blip_complete_status = SHIFT_BLIP_STATUS_WAITING;
    u16_shift_s_blip_timeout_cnt = (u16)0;

    u8_shift_s_shifting_complete = CLEAR;
}

/**************************************************************/
/*  Function:                                                 */
/*  main関数                                                   */
/*                                                            */
/*                                                            */
/**************************************************************/
void func_shift_g_main( void )
{
    func_shift_s_shift_mode_decide();           /* 変則モード確定処理 */

    func_shift_s_shift_sequence();              /* 変速シーケンス制御 */        /* エンジン制御はこの処理が主体になるはず。 */
    func_shift_s_shift_position_req_decide();   /* シフト位置要求設定処理 */
    func_shift_s_shift_dir_judge();             /* 変速方向判定処理 */

    /* 出力制御 */
    func_shift_s_clutch_control();              /* クラッチ制御 */
    func_shift_s_blip_control();                /* ブリッピング制御 */
    func_shift_s_shift_position_control();      /* シフト位置出力制御 */
}

/**************************************************************/
/*  Function:                                                 */
/*  関数                                                      */
/*  シフト位置を確定させる                                      */
/*                                                            */
/**************************************************************/
static void func_shift_s_shift_position_req_decide( void )
{
    if( u8_shift_g_shift_mode == SHIFT_MODE_MANUAL )
    { /* マニュアルシフト */
        func_shift_s_shift_position_decide_mode_manual();
    }
    else if( u8_shift_g_shift_mode == SHIFT_MODE_AUTOMATIC )
    {
        func_shift_s_shift_position_decide_mode_automatic();
    }
    else
    {
        ;
    }
}

/**************************************************************/
/*  Function:                                                 */
/*  シフト位置確定制御 マニュアルバージョン                       */
/*                                                            */
/**************************************************************/
static void func_shift_s_shift_position_decide_mode_manual( void )
{
    if( ( gpio_g_paddle_shift_sw.u8_state == LOW ) &&
             ( gpio_g_paddle_shift_sw.u8_state_bf == MID ))
    { /* シフトアップ */
        if( u8_shift_g_shift_position_req < SHIFT_POSI_MAX )
        {
            u8_shift_g_shift_position_req++;
        }
        else
        {
            u8_shift_g_shift_position_req = SHIFT_POSI_MAX;
        }
    }
    else if( ( gpio_g_paddle_shift_sw.u8_state == HI ) &&
             ( gpio_g_paddle_shift_sw.u8_state_bf == MID ))
    { /* シフトダウン */
        if( u8_shift_g_shift_position_req > SHIFT_POSI_0 )
        {
            u8_shift_g_shift_position_req--;
        }
        else
        {
            u8_shift_g_shift_position_req = SHIFT_POSI_0;
        }
    }
    else
    { /* 要求なし */
        ;               /* 現在のシフト位置を維持 */
    }
}

/**************************************************************/
/*  Function:                                                 */
/*  シフト位置確定制御 オートマチックバージョン                   */
/*  一番ここが複雑になるかも                                    */
/**************************************************************/
static void func_shift_s_shift_position_decide_mode_automatic( void )
{
    u16 u16_shift_position_rpm_diff;        /* 上の */
    u16 u16_shift_up_thr_speed;
    u16 u16_shift_down_thr_speed;

    u8 u8_shift_chg_dir;
    u16 u16_calc_buff;
    u16 u16_shift_his_speed;
    u32 u32_calc_buff;
    
    u16 u16_shift_chg_thr_rpm_his;        /* 変速閾値ヒステリシス */
    
    u16 u16_position_rpm_1minus;
    u16 u16_position_rpm_1plus;

    u8 u8_shift_position_req_before;

    u8 u8_shift_start_range;
    
    
    u8_shift_start_range = (u8)0;


    u8 u8_shift_up_enable_flag;
    u8 u8_shift_down_enable_flag;
    

    /*========*/
    /* 初期化 */
    /*========*/
    u8_shift_up_enable_flag = CLEAR;
    u8_shift_down_enable_flag = CLEAR;

    /* グローバル変数取得 */
    u8_shift_position_req_before = u8_shift_g_shift_position_req;

    
    /* アクセルON/OFF時の動作設定 */
    /* AT自動車特有の、ある速度域でアクセルON/OFFしても自然に速度がつながる制御の模擬 */
    /* 現在のスピードエリアを判別する */
    if( ( u8_sc_s_throttle_dir == SC_THROTTLE_DIR_NONE )  &&
        ( u16_speedsens_g_rpm_ary[ SPEEDSENS_CH_1STGEAR ] == (u16)0 ) )
    { /* アクセル踏まれてない & 停止状態 */
        /*======================*/
        /* ニュートラル状態の固定 */
        /*======================*/
        u8_shift_g_shift_position_req = SHIFT_POSI_0;               /* ニュートラル状態：エンジンブレーキがないことを再現する */
    }
    else if( ( u8_sc_s_throttle_dir        != SC_THROTTLE_DIR_NONE ) &&
             ( u8_sc_s_throttle_dir_before == SC_THROTTLE_DIR_NONE ) )
    {
        /*=================================*/
        /* アクセルON初回のシフト位置決定処理 */
        /*=================================*/
        /* 現在の回転数がどのシフトポジションの間にいるか判定しに行く */
        for( u8_shift_start_range = SHIFT_POSI_0; u8_shift_start_range < ( SHIFT_POSI_NUM - (u8)1 ); u8_shift_start_range++ )
        { /* ※8速目は検索から除外、どのみち7速でbreakできなかったら、8速で出てくるため。 */
            if( u16_speedsens_g_rpm_ary[ SPEEDSENS_CH_1STGEAR ] <= u16_shift_s_shift_posi_50per_rpm_ary[ u8_shift_start_range ] )
            { /* 現在の速度域から、最適な変速開始位置を探す */
                break;
            }
        }

        if( u8_shift_start_range == SHIFT_POSI_0 )
        { /* 0速 0rpmで停止判定だった */
            u8_shift_g_shift_position_req = SHIFT_POSI_1;              /* アクセルON & 停止状態 = "始動時" と判断し、ATモードなのでひとまず1速を設定 */
        }
        else
        {
            u8_shift_g_shift_position_req = u8_shift_start_range;      /* 1~8速を返す */
        }
    }
    else
    { /* 上記以外... (変な条件で入ってこないか要確認) */
        /* 注意：ここに来るときは必ず　SHIFT_POSI_1 以上になっている。 */


        /*==================*/
        /* 変速許可条件の設定 */
        /*==================*/
        /* 配列の範囲外参照を避けるために条件を用意した */
        if( u8_shift_g_shift_position_req <= ( SHIFT_POSI_MIN + (u8)1 ) )
        { /* 最小シフト位置 */
            u8_shift_down_enable_flag = CLEAR;
            u8_shift_up_enable_flag = SET;          /* シフトアップのみ許可 */
        }
        else if( u8_shift_g_shift_position_req >= SHIFT_POSI_MAX )
        { /* 最大シフト位置 */
            u8_shift_down_enable_flag = SET;        /* シフトダウンのみ許可 */
            u8_shift_up_enable_flag = CLEAR;
        }
        else
        {
            u8_shift_up_enable_flag = SET;
            u8_shift_down_enable_flag = SET;
        }

        /*=============*/
        /* 変速実行処理 */
        /*=============*/
        if( u8_shift_down_enable_flag == SET )
        {
            /* 閾値検索 */
            /* シフトダウン閾値の決定：現在のギヤ位置の回転数シフトアップ閾値回転数 - ヒステリシス */
            u16_calc_buff = u16_shift_s_shift_posi_50per_rpm_ary[ u8_shift_g_shift_position_req ] - u16_shift_s_shift_posi_50per_rpm_ary[ u8_shift_g_shift_position_req - (u8)1 ];
            u32_calc_buff = func_ud_g_calcmul_2x2_byte( u16_calc_buff, (u16)u8_rc_g_ch_duty_tbl[RC_DUTY_CH_REV_LIMIT] );
            u32_calc_buff = func_ud_g_calcdiv_4x4_byte( u32_calc_buff, (u32)RC_CH_DUTY_100P );
            u32_calc_buff += (u32)u16_shift_s_shift_posi_50per_rpm_ary[ u8_shift_g_shift_position_req - (u8)1 ];        /* 基準回転数 加算 */
            /* ヒステリシスを加算 */
            if( u32_calc_buff > (u32)SHIFT_POSI_CHG_HIS_MAX )
            {
                u16_shift_down_thr_speed = (u16)u32_calc_buff - SHIFT_POSI_CHG_HIS_MAX;
            }
            else
            {
                u16_shift_down_thr_speed = u16_shift_s_shift_posi_50per_rpm_ary[ u8_shift_g_shift_position_req - (u8)1 ];       /* ひとまず1段下の適正回転数をシフトダウン閾値の下限としておく */
            }

            /* 変速 */
            if( u16_speedsens_g_rpm_ary[ SPEEDSENS_CH_1STGEAR ] <= u16_shift_down_thr_speed )
            {
                if( u8_shift_g_shift_position_req > (SHIFT_POSI_MIN + (u8)2) )
                {
                    u8_shift_g_shift_position_req--;
                }
                else
                {
                    u8_shift_g_shift_position_req = SHIFT_POSI_1;               /* ※0速へ戻る判定は、アクセルOFF判定で行う */
                }
            }
        }

        if( u8_shift_up_enable_flag == SET )
        {
            /* 閾値検索 */
            /* シフトアップ閾値の決定： */
            u16_calc_buff = u16_shift_s_shift_posi_50per_rpm_ary[ u8_shift_g_shift_position_req + (u8)1 ] - u16_shift_s_shift_posi_50per_rpm_ary[ u8_shift_g_shift_position_req ];
            u32_calc_buff = func_ud_g_calcmul_2x2_byte( u16_calc_buff, (u16)u8_rc_g_ch_duty_tbl[RC_DUTY_CH_REV_LIMIT] );
            u32_calc_buff = func_ud_g_calcdiv_4x4_byte( u32_calc_buff, (u32)RC_CH_DUTY_100P );
            u32_calc_buff += (u32)u16_shift_s_shift_posi_50per_rpm_ary[ u8_shift_g_shift_position_req ];        /* 基準回転数 加算 */
            /* ヒステリシスを加算 */
            u16_shift_up_thr_speed = (u16)u32_calc_buff + SHIFT_POSI_CHG_HIS_MAX;
            if( u16_shift_up_thr_speed > u16_shift_s_shift_posi_50per_rpm_ary[ u8_shift_g_shift_position_req + (u8)1 ] )
            { /* 1つ上のギヤ位置での最適回転数を超えてしまった */
                u16_shift_up_thr_speed = u16_shift_s_shift_posi_50per_rpm_ary[ u8_shift_g_shift_position_req + (u8)1 ];       /* ひとまず1段上の適正回転数をシフトアップ閾値の上限としておく */
            }

            /* 変速 */
            if( u16_speedsens_g_rpm_ary[ SPEEDSENS_CH_1STGEAR ] >= u16_shift_up_thr_speed )
            { /* シフトアップ閾値回転数を超えた */
                if( u8_shift_g_shift_position_req < SHIFT_POSI_MAX )
                {
                    u8_shift_g_shift_position_req++;
                }
            }
        }
    }
}


/**************************************************************/
/*  Function:                                                 */
/*  シフト位置出力処理                                          */
/*                                                            */
/**************************************************************/
static void func_shift_s_shift_position_control( void )
{
    u8 u8_shift_posi_recalc;

    u8_shift_posi_recalc = SHIFT_POSI_0;

#if 0
    if( u 8_shift_s_posision_start_reset_req == SET )
    {
        u8_shift_g_shift_position_output = u8_shift_g_shift_position_req;       /* 変速状態を直ちに反映 */
        u 8_shift_s_posision_start_reset_req = CLEAR;                                  /* 要求クリア */
    }
#endif


    if( u8_shift_g_shifting_sequence == SHIFT_SEQ_CLUTCH_OFF_SHIFT_CHG )
    { /* 変速シーケンス内 / 変速後、クラッチを戻す前のタイミングで変速 */
        if( ( u8_shift_g_blip_complete_status == SHIFT_BLIP_STATUS_COMPLETE_MATCH   ) ||
            ( u8_shift_g_blip_complete_status == SHIFT_BLIP_STATUS_COMPLETE_TIMEOUT ) )
        { /* 変速シーケンスにきて いずれかの方法でブリッピング完了している */
            u8_shift_g_shift_position_output = u8_shift_g_shift_position_req;
            u8_shift_s_shifting_complete = SET;                 /* 変速完了 */
        }
        else
        {
            u8_shift_s_shifting_complete = CLEAR;               /* 変速未完了 */
        }
    }
    else
    {
        u8_shift_s_shifting_complete = CLEAR;           /* ひとまず変速なしの時は検完了扱い(使ってない) */
    }

    /* ニュートラル出力設定 */
    if( u8_shift_g_shift_position_output == SHIFT_POSI_0 )
    { /* 現在 0速 */
        U8_GPIO_G_OUT_NEUTRAL = SET;        /* 強制ニュートラル信号を出力 */
    }
    else
    {
        U8_GPIO_G_OUT_NEUTRAL = CLEAR;


        /* 0bit目 */
        /* 0-7で1を送っている都合上、現在のシフトポジションから１引いて伝える */
        u8_shift_posi_recalc = u8_shift_g_shift_position_output - (u8)1;

        if( ( u8_shift_posi_recalc & ((u8)0x01) ) != (u8)0 )
        { /* 0bit目が立っている */
            U8_GPIO_G_OUT_SHIFT_0 = SET;
        }
        else
        {
            U8_GPIO_G_OUT_SHIFT_0 = CLEAR;
        }

        /* 1bit目 */
        if( ( u8_shift_posi_recalc & ((u8)0x02) ) != (u8)0 )
        { /* 0bit目が立っている */
            U8_GPIO_G_OUT_SHIFT_1 = SET;
        }
        else
        {
            U8_GPIO_G_OUT_SHIFT_1 = CLEAR;
        }

        /* 2bit目 */
        if( ( u8_shift_posi_recalc & ((u8)0x04) ) != (u8)0 )
        { /* 0bit目が立っている */
            U8_GPIO_G_OUT_SHIFT_2 = SET;
        }
        else
        {
            U8_GPIO_G_OUT_SHIFT_2 = CLEAR;
        }
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

    u8_mode_before = u8_shift_g_shift_mode;             /* 現在の設定を一度保存する */

    
    /* 変速モード 要求値の取得 */
    if( gpio_g_shift_mode_sw.u8_state == HI )
    {
        u8_shift_s_shift_mode_req = SHIFT_MODE_MANUAL;
    }
    else
    {
        u8_shift_s_shift_mode_req = SHIFT_MODE_AUTOMATIC;
    }

    /* 停止状態でのみ、シフト操作モードの変更を許可する */
    if( u16_speedsens_g_rpm_ary[ SPEEDSENS_CH_1STGEAR ] <= SHIFT_DRIVE_STOP_SPEED )               /* 1次ギヤで止まってる判定する場合、0,1,2のシフトレバーが接続状態でないとダメなので、条件としては微妙かも */
    { /* 現在車は停止している */
        u8_shift_g_shift_mode = u8_shift_s_shift_mode_req;          /* 現在の変速モード要求を反映する */
    }
}


/**************************************************************/
/*  Function:                                                 */
/*  クラッチサーボ制御状態更新処理                               */
/*                                                            */
/**************************************************************/
static void func_shift_s_shift_sequence( void )
{
    /* 前回までのシーケンスを保存 */
    u8_shift_g_shifting_sequence_before = u8_shift_g_shifting_sequence;

    /* シーケンス管理 */
    switch ( u8_shift_g_shifting_sequence )
    {
        case SHIFT_SEQ_CLUTCH_OFF_STOP:
            /* 車体が完全に停止状態のときのみ、ここに来ているとして設計 */

            //デバッグ
            if( u8_shift_g_shift_mode == SHIFT_MODE_AUTOMATIC )
            { /* 現在オートマ選択中 */
                if( u8_sc_s_throttle_dir != SC_THROTTLE_DIR_NONE )
                { /* スロットルONされた */
                    //u 8_shift_s_posision_start_reset_req = SET;                             /* アクセルONで即始動させる必要があるので、初期変速要求(始動時のギヤ位置を指定する) */     /* こちら側にシフト位置変更要件を入れると処理の流れが複雑化するのでやめた */
                    u8_shift_g_shifting_sequence = SHIFT_SEQ_CLUTCH_OFF_BFORE_CHG;         /* すでにクラッチオフだが、まずは変速の初回ステップであるクラッチオフに遷移させる */
                }
            }
            else if( u8_shift_g_shift_mode == SHIFT_MODE_MANUAL )
            { /* 現在マニュアル選択中 */
                if( u8_shift_g_shift_position_req != u8_shift_g_shift_position_output )
                { /* 変速要求が発生している */
                    u8_shift_g_shifting_sequence = SHIFT_SEQ_CLUTCH_OFF_SHIFT_CHG;          /* 停止状態での変速はすぐに変速シーケンスへ飛ぶ */
                }
                else if( u8_sc_s_throttle_dir != SC_THROTTLE_DIR_NONE )
                { /* スロットルONされた */
                    if( u8_shift_g_shift_position_output != SHIFT_POSI_0 )
                    { /* 現在0速(ニュートラルではない) */
                        u8_shift_g_shifting_sequence = SHIFT_SEQ_CLUTCH_MEETING;            /* このタイミングで変速要求があった場合は、以降のシーケンスで受け付ける */
                    }
                }
            }
            break;

        case SHIFT_SEQ_CLUTCH_MEETING:
            /* 始動時のクラッチミート */
            if( u8_shift_s_clutch_meet_complete == SET )
            { /* 半クラッチ完了 */
                u8_shift_g_shifting_sequence = SHIFT_SEQ_CLUTCH_ON_DRIVE;
            }
            break;
        
        case SHIFT_SEQ_CLUTCH_ON_DRIVE:
            /* ↓↓↓↓ココの条件もう少し考えたい */
            if( u16_shift_s_clutch_off_cnt < U16_MAX )
            {
                u16_shift_s_clutch_off_cnt++;
            }

            /* 駆動中は常時変速許可 */
            if( u8_shift_g_shift_position_output != u8_shift_g_shift_position_req )
            { /* シフトポジション要求が変化 */
                u8_shift_g_shifting_sequence = SHIFT_SEQ_CLUTCH_OFF_BFORE_CHG;            /* クラッチOFFへ */
            }

            /* 停止判定 */
            if( u16_shift_s_clutch_off_cnt > SHIFT_CLUTCH_OFF_BY_STOP_TIME )
            { /* 車が動作中 or 変速後 or クラッチミート完了後 */
                if( u16_speedsens_g_rpm_ary[ SPEEDSENS_CH_1STGEAR ] <= (u16)200 )
                { /* 一定時間たってるのに車に動きがない */
                    if( u8_sc_s_throttle_dir == SC_THROTTLE_DIR_NONE )
                    { /* スロットル開けておらず停止している：単純に停止した */
                        u8_shift_g_shifting_sequence = SHIFT_SEQ_CLUTCH_OFF_STOP;             /* クラッチ切る */
                    }
                    else
                    { /* スロットルを開けており、クラッチミート完了したのに速度が出ていない：起動負荷高くて始動できてない判定とする */
                        u8_shift_g_shifting_sequence = SHIFT_SEQ_CLUTCH_OFF_KICK;             /* クラッチ蹴飛ばせッッ！！ */
                    }
                    u16_shift_s_clutch_off_cnt = (u16)0;
                }
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

            if( u16_shift_s_clutch_off_cnt > SHIFT_CLUTCH_ON_TO_OFF_WAIT_TIME )
            { /* クラッチOFFを最低維持する時間経過：サーボの動作応答を待つ */
                u8_shift_g_shifting_sequence = SHIFT_SEQ_CLUTCH_OFF_SHIFT_CHG;
                u16_shift_s_clutch_off_cnt = (u16)0;
            }
            break;

        case SHIFT_SEQ_CLUTCH_OFF_SHIFT_CHG:
            /* シフトチェンジ */
            /*u8_shift_g_shift_position_output = u8_shift_g_shift_position_req;*/       /* シフト要求を反映 */      /* func_shift_s_shift_position_control() 側で制御 */
            if( u8_shift_s_shifting_complete == SET )           /* 変速完了 */
            { /*  */
                u8_shift_g_shifting_sequence = SHIFT_SEQ_CLUTCH_OFF_AFTER_CHG;
            }
            break;

        case SHIFT_SEQ_CLUTCH_OFF_AFTER_CHG:
            /* シフトチェンジ後のクラッチOFF */
            if( u16_shift_s_clutch_off_cnt < U16_MAX )
            {
                u16_shift_s_clutch_off_cnt++;
            }
             
            if( u8_sc_s_throttle_dir == SC_THROTTLE_DIR_NONE )
            { /* スロットル開けずに変速した */
                u8_shift_g_shifting_sequence = SHIFT_SEQ_CLUTCH_OFF_STOP;                 /* マニュアルモード、クラッチ切った状態での変速（ガチャガチャしてるだけ判定）：停止状態へ戻る */
            }
            else /* (u8_sc_s_throttle_dir!=SC_THROTTLE_DIR_NONE) && (u8_sc_s_throttle_dir_before!=SC_THROTTLE_DIR_NONE) */
            { /* アクセルをふかし続けている状態でシフトアップ発生 */
                if( u8_shift_g_blip_complete_status == SHIFT_BLIP_STATUS_COMPLETE_MATCH )     /* 回転数一致による変速 */
                {
                    if( u16_shift_s_clutch_off_cnt > SHIFT_CLUTCH_OFF_TO_ON_WAIT_TIME )           /* 最低限は、変速後もクラッチOFFを継続（変速用サーボの応答を待機） */
                    { /* クラッチOFFを最低維持する時間経過：変速用サーボの動作応答を待つ */
                        u8_shift_g_shifting_sequence = SHIFT_SEQ_CLUTCH_ON_DRIVE;                 /* すぐにバチッとクラッチ繋ぐ */
                        u16_shift_s_clutch_off_cnt = (u16)0;
                    }
                }
                else
                { /* 速度が出てない状態でスロットル開けながら変速(始動) or タイムアウトにより何とか変速(回転数あってない) */
                    u8_shift_g_shifting_sequence = SHIFT_SEQ_CLUTCH_MEETING;                  /* 半クラッチで繋ぐ */    /* @@半クラッチは、適切な速度域でない場合に、タイムアウトでクラッチ繋ぐ場合に使うほうが、実車の挙動に近いかも？ */
                }
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
                u8_shift_g_shifting_sequence = SHIFT_SEQ_CLUTCH_ON_DRIVE;                       /* そのまま駆動状態に戻る (クラッチがガツンとつながり、以降始動できなければここと行き来してガクガクさせる) */
                u16_shift_s_clutch_off_cnt = (u16)0;
            }

            break;

        default:
            break;
    }
}


/**************************************************************/
/*  Function:                                                 */
/*  変速方向確認                                               */
/*                                                            */
/**************************************************************/
static void func_shift_s_shift_dir_judge( void )
{
    if( u8_shift_g_shift_position_req > u8_shift_g_shift_position_output )
    { /* シフトアップ中 */
        u8_shift_s_shift_dir = SHIFT_POSITION_UP;
    }
    else if( u8_shift_g_shift_position_req < u8_shift_g_shift_position_output )
    { /* シフトダウン */
        u8_shift_s_shift_dir = SHIFT_POSITION_DOWN;
    }
    else /* ( u8_shift_g_shift_position_req == u8_shift_g_shift_position_output ) */
    { /*  */
        u8_shift_s_shift_dir = SHIFT_POSITION_STABLE;
    }
}

/**************************************************************/
/*  Function:                                                 */
/*  ブリッピング制御要求                                        */
/*                                                            */
/**************************************************************/
static void func_shift_s_blip_control( void )
{
    u8 u8_blip_sts;
    u16 u16_calc_buff;

    /* 初期化 */
    u8_blip_sts = SHIFT_BLIP_STATUS_WAITING;                /* ブリッピング待機中 */

    if( u8_shift_g_shifting_sequence == SHIFT_SEQ_CLUTCH_OFF_SHIFT_CHG )
    { /* 変速シーケンス内 & 0速ではない */
        /* タイムアウト判定用 */
        if( u16_shift_s_blip_timeout_cnt < U16_MAX )
        {
            u16_shift_s_blip_timeout_cnt++;
        }

        if( u16_shift_s_blip_timeout_cnt < SHIFT_BLIP_TIMEOUT )
        { /* ブリッピング有効時間内 */
#if 0
            /* こっちのほうが実装単純 */
            if( u8_shift_s_shift_dir == SHIFT_POSITION_UP )
            { /* シフトアップしようとしている */
                if( u16_speedsens_g_rpm_ary[SPEEDSENS_CH_MTR]  < u16_speedsens_g_rpm_ary[ SPEEDSENS_CH_1STGEAR ] )
                { /* エンジン回転数 < 負荷軸側回転数 になるまで回転数が下がった */
                    u8_blip_sts = SET;
                }
            }
            else if( u8_shift_s_shift_dir == SHIFT_POSITION_DOWN )
            { /* シフトダウンしようとしている */
                if( u16_speedsens_g_rpm_ary[SPEEDSENS_CH_MTR] > u16_speedsens_g_rpm_ary[ SPEEDSENS_CH_1STGEAR ] )
                { /* エンジン回転数 > 負荷軸側回転数 になるまで回転数が上がった */
                    u8_blip_sts = SET;
                }
            }
#else
            /* モータとギヤの回転数の一致度合いを判断 */
            if( u16_speedsens_g_rpm_ary[ SPEEDSENS_CH_MTR ] > u16_speedsens_g_rpm_ary[ SPEEDSENS_CH_1STGEAR ] )
            {
                u16_calc_buff = u16_speedsens_g_rpm_ary[ SPEEDSENS_CH_MTR ] - u16_speedsens_g_rpm_ary[ SPEEDSENS_CH_1STGEAR ];
            }
            else
            {
                u16_calc_buff = u16_speedsens_g_rpm_ary[ SPEEDSENS_CH_1STGEAR ] - u16_speedsens_g_rpm_ary[ SPEEDSENS_CH_MTR ];
            }

            if( u16_calc_buff < SHIFT_BLIP_SPEED_ACCURACY )     /* PIの応答にもよるが、大体で合わせる */
            { /* 大体回転数あってる */
                u8_blip_sts = SHIFT_BLIP_STATUS_COMPLETE_MATCH;
            }
#endif
        }
        else
        { /* ブリッピングできていなくても、変速完了させる */
            u8_blip_sts = SHIFT_BLIP_STATUS_COMPLETE_TIMEOUT;        /* @@デバッグ時は一時無効化したほうがよさそう */
        }
    }
    else
    {
        u8_blip_sts = SHIFT_BLIP_STATUS_WAITING;          /* 変速時以外は使わないが、一応完了側に振っておく */
        u16_shift_s_blip_timeout_cnt = (u16)0;
    }

    u8_shift_g_blip_complete_status = u8_blip_sts;
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
    switch ( u8_shift_g_shifting_sequence )
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
                u8_clutch_meet_angle_idx_diff = (u8)( SHIFT_SERVO_ANGLE_CLUTCH_ON - SHIFT_SERVO_ANGLE_CLUTCH_OFF );     /* ここに来る場合は定義された値がCLUTCH_OFFのほうが大きいときなので問題なし */
                u8_clutch_meet_dir_prop = SET;              /* 比例増加 */
            }
            else
            { /* ONで角度が減る方向 */
                u8_clutch_meet_angle_idx_diff = (u8)( SHIFT_SERVO_ANGLE_CLUTCH_OFF - SHIFT_SERVO_ANGLE_CLUTCH_ON );     /* ここに来る場合は定義された値がCLUTCH_OFFのほうが大きいときなので問題なし */
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
                u8_clutch_servo_angle_idx_result = SHIFT_SERVO_ANGLE_CLUTCH_OFF - (u8)u32_calc_buff;        /* ここに来る場合は定義された値がCLUTCH_OFFのほうが大きいときなので問題なし */
            }

            /* クラッチミート完了判定 */
            if( u16_shift_s_clutch_meet_cnt >= SHIFT_CLUTCH_MEET_TIME )
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
