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

#define SHIFT_POSITION_UP                   ((u8)0)
#define SHIFT_POSITION_DOWN                 ((u8)1)
#define SHIFT_POSITION_STABLE               ((u8)2)      /* シフト位置 変化なし */

#define SHIFT_DRIVE_STOP_CNT                ((u8)10)     /* 10ms */
#define SHIFT_DRIVE_STOP_SPEED              ((u16)0)     /* 0rpm */


/* 時間定義 */
#define SHIFT_CLUTCH_OFF_TIME               ((u16)100)
#define SHIFT_CLUTCH_OFF_TIME_KICK          ((u16)200)
#define SHIFT_CLUTCH_MEET_TIME              ((u16)2000)

/* クラッチ制御用定義 */
#define SHIFT_SERVO_ANGLE_CLUTCH_ON         SERVO_DEG_IDX__85
#define SHIFT_SERVO_ANGLE_CLUTCH_OFF        SERVO_DEG_IDX__0

/* オートマチック変速定義 */
/* ※メインMCU側では0~7の計8ポジションで指令を送っているため、idxは0始まり... */
#define SHIFT_POSI_THRESHOLD_NUM            ((u8)8)


/* ブリッピング制御定義 */
#define SHIFT_BLIP_SPEED_ACCURACY           ((u16)100)              /* ブリッピングの正確さ 実写でいうとシフトチェンジのヘタクソ具合 */
#define SHIFT_BLIP_TIMEOUT                  ((u16)1000)         /*  */

/* 変速基準回転数定義 */
/* 実機のトルクを見て決めたい */
/* あんまり速度上がらない気がする */
/* ➡方針決め：インバータへの印加duty50%の時の出力で得られる回転数を閾値として設定する */
/* バッテリ電圧がなくなってきたときの対応はどうするか？上の方へはシフトできない仕様でOK？ */
#define SHIFT_POSI_0_APPR_RPM       ((u16)1000)
#define SHIFT_POSI_1_APPR_RPM       ((u16)2000)
#define SHIFT_POSI_2_APPR_RPM       ((u16)3000)
#define SHIFT_POSI_3_APPR_RPM       ((u16)4000)
#define SHIFT_POSI_4_APPR_RPM       ((u16)5000)
#define SHIFT_POSI_5_APPR_RPM       ((u16)8000)
#define SHIFT_POSI_6_APPR_RPM       ((u16)11000)
#define SHIFT_POSI_7_APPR_RPM       ((u16)14000)

#define SHIFT_POSI_CHG_HIS_MAX                ((u8)500)                /* 変速のヒステリシス最大値(実質、レブリミット) */

#define SHIFT_POSI_REMAIN_CNT       ((u16)300)       /* サーボの稼働累計時間よりは長くしないとダメ。 */


/* 変速比設定 (ギヤ比基準) */
/* 実際は小数なので、桁ましのために*10000 */
#define SHIFT_GEAR_RATIO_POSI_0     ((u16)512)
#define SHIFT_GEAR_RATIO_POSI_1     ((u16)718)
#define SHIFT_GEAR_RATIO_POSI_2     ((u16)1340)
#define SHIFT_GEAR_RATIO_POSI_3     ((u16)1879)
#define SHIFT_GEAR_RATIO_POSI_4     ((u16)3200)
#define SHIFT_GEAR_RATIO_POSI_5     ((u16)4488)
#define SHIFT_GEAR_RATIO_POSI_6     ((u16)8374)
#define SHIFT_GEAR_RATIO_POSI_7     ((u16)11743)


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
static void func_shift_s_shifting_control( void );
static void func_shift_s_shift_sequence( void );

static u8 u8_shift_s_shift_chg_enable_wait_cnt;
static u8 u8_shift_s_shift_mode_req;
static u8 u8_shift_s_blip_req;
u8 u8_shift_g_shifting_sequence;
u8 u8_shift_g_shifting_sequence_before;
static u16 u16_shift_s_clutch_meet_cnt;
static u8 u8_shift_s_clutch_meet_angle;
static u8 u8_shift_s_clutch_meet_complete;
static u8 u8_shift_s_posision_reset_req;

u8 u8_shift_g_shift_mode;
u8 u8_shift_g_shift_position_req;
u8 u8_shift_g_shift_position_output;
static u8 u8_shift_s_shift_dir;
u8 u8_shift_g_blip_complete;
static u8 u8_shift_s_shifting_complete;
static u16 u16_shift_s_blip_timeout_cnt;


u16 u16_shift_s_clutch_off_cnt;

static u16 u16_shift_s_auto_position_cnt;

/* パラメータ定義 */
static const u16 u16_shift_s_shift_posi_50per_speed_ary[ SHIFT_POSI_NUM ] =
{
    SHIFT_POSI_0_APPR_RPM,
    SHIFT_POSI_1_APPR_RPM,
    SHIFT_POSI_2_APPR_RPM,
    SHIFT_POSI_3_APPR_RPM,
    SHIFT_POSI_4_APPR_RPM,
    SHIFT_POSI_5_APPR_RPM,
    SHIFT_POSI_6_APPR_RPM,
    SHIFT_POSI_7_APPR_RPM,
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
    u8_shift_g_blip_complete = SET;
    u8_shift_s_shifting_complete = SET;
    u16_shift_s_blip_timeout_cnt = (u16)0;

    u8_shift_s_posision_reset_req = CLEAR;
}

/**************************************************************/
/*  Function:                                                 */
/*  main関数                                                   */
/*                                                            */
/*                                                            */
/**************************************************************/
void func_shift_g_main( void )
{
    func_shift_s_shift_sequence();              /* 変速シーケンス制御 */        /* エンジン制御はこの処理が主体になるはず。 */

    func_shift_s_shift_mode_decide();           /* 変則モード確定処理 */
    func_shift_s_shift_position_req_decide();   /* シフト位置要求設定処理 */
    func_shift_s_shift_dir_judge();             /* 変速方向判定処理 */

    /* 出力制御 */
    func_shift_s_clutch_control();              /* クラッチ制御 */
    func_shift_s_blip_control();                /* ブリッピング制御 */
    func_shift_s_shifting_control();            /* シフト位置出力制御 */
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
    if( u8_shift_s_posision_reset_req == SET )
    { /* シフト位置リセット要求が発生 */
        u8_shift_g_shift_position_req = SHIFT_POSI_0;               /* 初期位置 */
    }
    else if( ( gpio_g_paddle_shift_sw.u8_state == LOW ) &&
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
    u16 u16_shift_up_thr_speed;
    u16 u16_shift_down_thr_speed;
    u8 u8_shift_chg_dir;
    u16 u16_shift_his_speed;
    u32 u32_calc_buff;
    
    u16 u16_shift_thr_speed_his;        /* 変速閾値ヒステリシス */
    
    u16 u16_position_rpm_1minus;
    u16 u16_position_rpm_1plus;

    u8 u8_shift_position_req_before;
    
    
    /* 変速位置固定カウント */
    /* 変速後は一定時間変速を許可しない */
    if( u16_shift_s_auto_position_cnt < U16_MAX )
    {
        u16_shift_s_auto_position_cnt++;
    }
    
    
    /* 実際には、この関数内で回転数がついてこれなければ、勝手に減速する仕様となる */

    /* シフトチェンジのレブリミットレベルを算出する */
    /* ただし、現在のギヤ位置から見て、最大出力で到達できそうな回転数範囲に制限をかける */
    /* @@シフトダウンの下限値回転数が、１段下のシフトアップ回転数と被らないように注意する。実測合わせ要検討 */
    u32_calc_buff = (u32)0;
    u32_calc_buff = func_ud_g_calcmul_2x2_byte( (u16)u8_rc_g_ch_duty_tbl[RC_DUTY_CH_REV_LIMIT], (u16)SHIFT_POSI_CHG_HIS_MAX );
    u32_calc_buff = func_ud_g_calcdiv_4x4_byte( u32_calc_buff, (u32)RC_CH_DUTY_100P );
    u16_shift_thr_speed_his = (u16)u32_calc_buff;
    
    
    /* シフトチェンジの回転数閾値を計算する */
    if( u8_shift_g_shift_position_output == SHIFT_POSI_0 )
    { /* 現在0速：シフトアップのみ可能 */
        u16_shift_up_thr_speed   = u16_shift_s_shift_posi_50per_speed_ary[ SHIFT_POSI_0 ] + u16_shift_thr_speed_his;
        u16_shift_down_thr_speed = (u16)0;      /* @@ココの値どうしよう... */
    }
    else if( u8_shift_g_shift_position_output == SHIFT_POSI_7 )
    { /* 現在7速：シフトダウンのみ可能 */
        u16_shift_up_thr_speed   = U16_MAX;    /* ひとまず変速不可能な回転数にしておく */
        if( u16_shift_s_shift_posi_50per_speed_ary[ SHIFT_POSI_7 ] > u16_shift_thr_speed_his )
        {
            u16_shift_down_thr_speed = u16_shift_s_shift_posi_50per_speed_ary[ SHIFT_POSI_6 ] - u16_shift_thr_speed_his;
        }
    }
    else
    { /* "現在" 1～6速 */
        u16_shift_up_thr_speed = u16_shift_s_shift_posi_50per_speed_ary[ u8_shift_g_shift_position_output ] + u16_shift_thr_speed_his;
        u16_shift_down_thr_speed = u16_shift_s_shift_posi_50per_speed_ary[ u8_shift_g_shift_position_output ] - u16_shift_thr_speed_his;
    }
    
    /* 変速処理 */
    /* 変速要求設定 */
    u8_shift_position_req_before = u8_shift_g_shift_position_req;

    if( u16_shift_s_auto_position_cnt > SHIFT_POSI_REMAIN_CNT )
    { /* 変速後は少し間を置く */
        if( u16_speedsens_g_speed_ave_1stgear >= u16_shift_up_thr_speed )
        { /* シフトアップ閾値回転数を超えた */
            if( u8_shift_g_shift_position_req < SHIFT_POSI_7 )
            {
                u8_shift_g_shift_position_req++;
            }
            else
            {
                u8_shift_g_shift_position_req = SHIFT_POSI_7;
            }
        }
        else if( u16_speedsens_g_speed_ave_1stgear <= u16_shift_down_thr_speed )
        {
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
        { /* 50%出力での変速域内 */
            ;       /* 現在のシフトポジションを維持 */
        }
    }

    /* 始動時の初期位置設定 */      //とりあえずここに置いておく
    if( u8_shift_s_posision_reset_req == SET )
    { /* シフト位置リセット要求が発生 */
        u8_shift_g_shift_position_req = SHIFT_POSI_0;               /* 初期位置 */
    }

    /* 変速発生時の処理 */
    if( u8_shift_position_req_before != u8_shift_g_shift_position_req )
    { /* シフト位置の更新があった */
        u16_shift_s_auto_position_cnt = (u16);
    }
}


/**************************************************************/
/*  Function:                                                 */
/*  シフト位置出力処理                                          */
/*                                                            */
/**************************************************************/
static void func_shift_s_shifting_control( void )
{
    if( u8_shift_s_posision_reset_req == SET )
    {
        u8_shift_g_shift_position_output = u8_shift_g_shift_position_req;       /* 変速状態を直ちに反映 */
        u8_shift_s_posision_reset_req = CLEAR;                                  /* 要求クリア */
    }
    if( u8_shift_g_shifting_sequence == SHIFT_SEQ_CLUTCH_OFF_SHIFT_CHG )
    { /* 変速シーケンス内 / ブリッピング完了 */
        if( u8_shift_g_blip_complete == SET )
        { /* 変速シーケンスにきてブリッピング完了している */
            u8_shift_g_shift_position_output = u8_shift_g_shift_position_req;
            u8_shift_s_shifting_complete = SET;                 /* 変速完了 */
        }
        else
        {
            u8_shift_s_shifting_complete = CLEAR;               /* 変速完了 */
        }
    }
    else
    {
        u8_shift_s_shifting_complete = CLEAR;           /* ひとまず変速なしの時は検完了扱い(使ってない) */
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
    /* 前回までのシーケンスを保存 */
    u8_shift_g_shifting_sequence_before = u8_shift_g_shifting_sequence;

    /* シーケンス管理 */
    switch ( u8_shift_g_shifting_sequence )
    {
    case SHIFT_SEQ_CLUTCH_OFF_STOP:
        if( u8_sc_s_throttle_dir != SC_THROTTLE_DIR_NONE )
        { /* スロットル中立ではなくなった */
            u8_shift_g_shifting_sequence = SHIFT_SEQ_CLUTCH_MEETING;                  /* クラッチミート開始 */

            if( u8_shift_g_shift_position_req != SHIFT_POSI_0 )
            {
                u8_shift_s_posision_reset_req = SET;                /* 初期位置に戻してから動作開始させる */
            }
        }
        else if( u8_shift_g_shift_position_output != u8_shift_g_shift_position_req )
        { /* 停止状態で シフトポジション要求が変化 */
            u8_shift_g_shifting_sequence = SHIFT_SEQ_CLUTCH_OFF_BFORE_CHG;            /* クラッチOFFへ */
        }
        else
        {
            ;       /* 待機 */
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
        u16_shift_s_clutch_off_cnt = (u16)0;

        /* 駆動中は常時変速許可 */
        if( u8_shift_g_shift_position_output != u8_shift_g_shift_position_req )
        { /* シフトポジション要求が変化 */
            u8_shift_g_shifting_sequence = SHIFT_SEQ_CLUTCH_OFF_BFORE_CHG;            /* クラッチOFFへ */
        }

        if( u16_speedsens_g_speed_ave_1stgear == (u16)0 )
        { /* 車が完全に停止した */
            u8_shift_g_shifting_sequence = SHIFT_SEQ_CLUTCH_OFF_STOP;             /* クラッチ切る */
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
            u8_shift_g_shifting_sequence = SHIFT_SEQ_CLUTCH_OFF_SHIFT_CHG;
            u16_shift_s_clutch_off_cnt = (u16)0;
        }
        break;

    case SHIFT_SEQ_CLUTCH_OFF_SHIFT_CHG:
        /* シフトチェンジ */
        /*u8_shift_g_shift_position_output = u8_shift_g_shift_position_req;*/       /* シフト要求を反映 */      /* func_shift_s_shifting_control() 側で制御 */
        u8_shift_g_shifting_sequence = SHIFT_SEQ_CLUTCH_OFF_AFTER_CHG;
        break;

    case SHIFT_SEQ_CLUTCH_OFF_AFTER_CHG:
        /* シフトチェンジ後のクラッチOFF */
        if( u16_shift_s_clutch_off_cnt < U16_MAX )
        {
            u16_shift_s_clutch_off_cnt++;
        }
        if( u16_shift_s_clutch_off_cnt > SHIFT_CLUTCH_OFF_TIME )
        { /* クラッチOFFを最低維持する時間経過：変速用サーボの動作応答を待つ */
            if( ( u8_shift_g_blip_complete == SET ) &&              /* ブリッピング完了 */
                ( u8_shift_s_shifting_complete == SET ) )           /* 変速完了 */
            {
                if( u16_speedsens_g_speed_ave_1stgear == (u16)0 )
                { /* 車が完全に停止した状態での変速だった */
                    u8_shift_g_shifting_sequence = SHIFT_SEQ_CLUTCH_OFF_STOP;                 /* クラッチ切った状態での停止状態へ戻る */
                }
                else if( u16_speedsens_g_speed_ave_1stgear < (u16)1000 )
                { /* 車が動作中の変速ではあるが、あまり速度が出ていない */
                    u8_shift_g_shifting_sequence = SHIFT_SEQ_CLUTCH_MEETING;                  /* 半クラッチで繋ぐ */
                }
                else
                { /* 速度が十分に出た状態での変速 */
                    u8_shift_g_shifting_sequence = SHIFT_SEQ_CLUTCH_ON_DRIVE;                 /* すぐにクラッチ繋ぐ */
                }
            
                u16_shift_s_clutch_off_cnt = (u16)0;
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
            u8_shift_g_shifting_sequence = SHIFT_SEQ_CLUTCH_MEETING;
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
    u8 u8_blip_cmp;
    u16 u16_calc_buff;

    /* 初期化 */
    u8_blip_cmp = CLEAR;

    if( u8_shift_g_shifting_sequence == SHIFT_SEQ_CLUTCH_OFF_SHIFT_CHG )
    { /* 変速シーケンス内 */
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
                if( u16_speedsens_g_speed_ave_mtr < u16_speedsens_g_speed_ave_1stgear )
                { /* エンジン回転数 < 負荷軸側回転数 になるまで回転数が下がった */
                    u8_blip_cmp = SET;
                }
            }
            else if( u8_shift_s_shift_dir == SHIFT_POSITION_DOWN )
            { /* シフトダウンしようとしている */
                if( u16_speedsens_g_speed_ave_mtr > u16_speedsens_g_speed_ave_1stgear )
                { /* エンジン回転数 > 負荷軸側回転数 になるまで回転数が上がった */
                    u8_blip_cmp = SET;
                }
            }
#else
            /* モータとギヤの回転数の一致度合いを判断 */
            if( u16_speedsens_g_speed_ave_mtr > u16_speedsens_g_speed_ave_1stgear )
            {
                u16_calc_buff = u16_speedsens_g_speed_ave_mtr - u16_speedsens_g_speed_ave_1stgear;
            }
            else
            {
                u16_calc_buff = u16_speedsens_g_speed_ave_1stgear - u16_speedsens_g_speed_ave_mtr;
            }

            if( u16_calc_buff < SHIFT_BLIP_SPEED_ACCURACY )     /* PIの応答にもよるが、大体で合わせる */
            { /* 大体回転数あってる */
                u8_blip_cmp = SET;
            }
#endif
        }
        else
        { /* ブリッピングできていなくても、変速完了させる */
            //u8_blip_cmp = SET;        /* デバッグ時は一時無効化 */
        }
    }
    else
    {
        u8_blip_cmp = SET;          /* 変速時以外は使わないが、一応完了側に振っておく */
        u16_shift_s_blip_timeout_cnt = (u16)0;
    }

    u8_shift_g_blip_complete = u8_blip_cmp;
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
