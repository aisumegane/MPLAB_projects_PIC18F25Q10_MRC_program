/*
 * File:   speedcontrol.c
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
#include "./mcufunc/timer_driver.h"
#include "shift.h"

#include "speedcontrol.h"

/* パラメータ */
#define SC_THROTTLE_DEFINE_0P           RC_CH_DUTY_50P          /* スロットル中立時の入力 */
#define SC_THROTTLE_HYSTERESIS          RC_CH_DUTY_5P           /* スロットル操作のヒステリシス */
#define SC_IDLING_DUTY                  RC_CH_DUTY_10P          /* アイドリング中のduty */
#define SC_CLUTCH_MEETING_DUTY          RC_CH_DUTY_20P          /* クラッチミート中のduty */

#define SC_RPM_CTRL_GAIN_P_DIV2N        ((u8)6)                 /* Pゲイン 割り算形式で実装　※大きくするほど変化が緩やかになる */
#define SC_RPM_CTRL_GAIN_I_DIV2N        ((u8)6)                 /* Iゲイン 割り算形式で実装　※大きくするほど変化が緩やかになる */   /* @@大きすぎる(割り算しすぎ)だと、i項がs16の範囲を外れてしまうので注意。飽和状態で32767は超えない設計にすること！！！！ */


/* ※そもそもプロポからのduty更新指令が50ms間隔でしか送れないので、若干意味がない設定。何もしなくても緩やかに変化してくれる？ */
#define SC_CONST_DUTY_CTRL_UPDATE_CYCLE ((u8)20)                /* 10msに1回更新 例:1%-20ms -> 100%-2000ms  */
#define SC_CONST_DUTY_CTRL_INC          TIMER_INVERTER_DUTY_1P  /* 一定duty制御での加速量 */
#define SC_CONST_DUTY_CTRL_DEC          TIMER_INVERTER_DUTY_1P  /* 一定duty制御での減速量 */

u8 u8_sc_s_throttle_dir;
u8 u8_sc_g_throttle_rc_ch_duty_target;        /* 目標duty */
u8 u8_sc_g_throttle_duty_ref;           /* 指令duty */
u16 u16_sc_s_throttle_inv_duty_output;      /* インバータへの出力duty */        /* 回転数制御でも使うので、0~100だとちょっと精度が足りない。->RC_DUTYとは別のdutyを定義する */


/* duty制御 */
static u8 u8_sc_s_duty_update_cycle_cnt;

/* 回転数PI制御 */
u8 u8_sc_s_pi_saturate_flag;
s16 s16_sc_s_integral_cnt;


/* プロトタイプ宣言 */
static void func_sc_s_throttle_per_dir_update( u8 *u8_per, u8 *u8_dir );
static void func_sc_s_throttle_threshold_update( u8 *u8_forward_th, u8 *u8_backward_th );
static void func_sc_s_output_duty_update( u16 *u16_duty_output );
static u16 func_sc_s_duty_ctrl( u8 u8_duty_target, u16 u16_duty_now );
static u16 func_sc_s_rpm_pi_ctrl( u16 u16_speed_target, u16 u16_speed_now, u16 u16_duty_now )

/**************************************************************/
/*  Function:                                                 */
/*                                                            */
/**************************************************************/
void func_speedcontrol_g_init( void )
{
    u8_sc_s_throttle_dir = SC_THROTTLE_DIR_NONE;
    u8_sc_g_throttle_rc_ch_duty_target = RC_CH_DUTY_0P;
    u8_sc_g_throttle_duty_ref    = RC_CH_DUTY_0P;
    u16_sc_s_throttle_inv_duty_output = RC_CH_DUTY_0P;

    u8_sc_s_duty_update_cycle_cnt = (u8)0;
    s16_sc_s_integral_cnt = (s16)0;
    u8_sc_s_pi_saturate_flag = CLEAR;
}


/**************************************************************/
/*  Function:                                                 */
/*                                                            */
/**************************************************************/
/* モジュールの切り分けが微妙かも　センスの問題もある。 */
void func_speedcontrol_g_main( void )
{
    /* グローバル変数更新 */
    func_sc_s_throttle_per_dir_update( &u8_sc_g_throttle_rc_ch_duty_target ,&u8_sc_s_throttle_dir );     /* dutyをスロットル中心が0になるように再計算 */
    func_sc_s_output_duty_update( &u16_sc_s_throttle_inv_duty_output );                                  /* 出力duty更新       */


    func_sc_s_speed_output( u16_sc_s_throttle_inv_duty_output );                                         /* 出力制御           */
}


/**************************************************************/
/*  Function:                                                 */
/*  スロットル入力量 判断処理                                   */
/*  レバー操作に対して0~100の入力としているため、これを前後に振り分ける */
/*  分解能的に成立していないところがあるが、実害なのでOKとした     */
/**************************************************************/
static void func_sc_s_throttle_per_dir_update( u8 *u8_per, u8 *u8_dir )
{
    u8 u8_duty_input;
    u8 u8_duty_forward_th;
    u8 u8_duty_backward_th;
    u8 u8_duty_recalc_base;
    u8 u8_duty_recalc_gap;

    u32 u32_calc_buff;

    /* 初期化 */ 
    u8_duty_input = (u8)0;
    u8_duty_forward_th = (u8)0;
    u8_duty_backward_th = (u8)0;
    u8_duty_recalc_base = (u8)0;
    u8_duty_recalc_gap = (u8)0;

    /*-----------------------------------------------------------------------*/

    u8_duty_input = u8_rc_g_ch_duty_tbl[RC_DUTY_CH_THROTTLE];

    /* スロットル閾値更新 */
    func_sc_s_throttle_threshold_update( &u8_duty_forward_th, &u8_duty_backward_th );


    /* 入力判定 */
    if( u8_duty_input > u8_duty_forward_th )
    { /* 前進方向のduty入力 */
        u8_duty_recalc_base = RC_CH_DUTY_100P - u8_duty_forward_th;
        u8_duty_recalc_gap = u8_duty_input - u8_duty_forward_th;

        u32_calc_buff = func_ud_g_calcmul_2x2_byte( (u16)u8_duty_recalc_gap, (u16)RC_CH_DUTY_100P );
        u32_calc_buff = func_ud_g_calcdiv_4x4_byte( u32_calc_buff, (u32)u8_duty_recalc_base );

        *u8_per = (u8)u32_calc_buff;            /* 前進方向のduty(再計算後)を設定 */
        *u8_dir = SC_THROTTLE_DIR_FORWARD;      /* 前進 */
    }
    else if( u8_duty_input < u8_duty_backward_th )
    { /* 後退方向のduty入力 */
        u8_duty_recalc_base = u8_duty_backward_th;
        u8_duty_recalc_gap = u8_duty_backward_th - u8_duty_input;

        u32_calc_buff = func_ud_g_calcmul_2x2_byte( (u16)u8_duty_recalc_gap, (u16)RC_CH_DUTY_100P );
        u32_calc_buff = func_ud_g_calcdiv_4x4_byte( u32_calc_buff, (u32)u8_duty_recalc_base );

        *u8_per = (u8)u32_calc_buff;            /* 後進方向のduty(再計算後)を設定 */
        *u8_dir = SC_THROTTLE_DIR_BACKWARD;     /* 後進 */
    }
    else
    { /* 中立状態 */
        *u8_per = RC_CH_DUTY_0P;
        *u8_dir = SC_THROTTLE_DIR_NONE;
    }
}

/**************************************************************/
/*  Function:                                                 */
/*  スロットル開閉閾値 更新処理                                 */
/**************************************************************/
static void func_sc_s_throttle_threshold_update( u8 *u8_forward_th, u8 *u8_backward_th )
{
    /* 前進 入力閾値レベルの設定 */
    *u8_forward_th = SC_THROTTLE_DEFINE_0P + SC_THROTTLE_HYSTERESIS;
    if( *u8_forward_th > RC_CH_DUTY_100P )
    { /* ヒステリシスを加味すると100%を超える */
        *u8_forward_th = RC_CH_DUTY_100P;
    }

    /* 更新 入力閾値レベルの設定 */
    if( SC_THROTTLE_DEFINE_0P > SC_THROTTLE_HYSTERESIS )
    { /* ヒステリシスを加味すると0%を下回る */
        *u8_backward_th = SC_THROTTLE_DEFINE_0P - SC_THROTTLE_HYSTERESIS;
    }
    else
    {
        *u8_backward_th = RC_CH_DUTY_0P;
    }
}


/**************************************************************/
/*  Function:                                                 */
/*  スロットル開口度 調節処理                                   */
/*  一定dutyでの制御と、負荷軸回転数に合わせに行く制御を切り替える  */
/*  通常変速状態ではduty制御を、変速時のシフトアップ・シフトダウンは */
/*  負荷軸側回転数を目標値として合わせに行く                      */
/**************************************************************/
static void func_sc_s_output_duty_update( u16 *u16_duty_now )
{
    /* 変速シーケンスに依存して出力するdutyが変わってくるので、switch文にしてみた */

    u16 u16_duty_new;

    /* 初期化 */
    u16_duty_new = RC_CH_DUTY_0P;

    switch ( u8_shift_g_shifting_sequence )
    {
        case SHIFT_SEQ_CLUTCH_OFF_STOP:
            /* 停止中 */
            u16_duty_new = SC_IDLING_DUTY;            /* 停止中もアイドリングdutyを出しておく */
            break;
    
        case SHIFT_SEQ_CLUTCH_MEETING:
            u16_duty_new = SC_CLUTCH_MEETING_DUTY;    /* 始動時のクラッチミートdutyはひとまず固定値にしておく */
            break;

        case SHIFT_SEQ_CLUTCH_ON_DRIVE:
            /* 一定duty制御領域 */
            u16_duty_new = func_sc_s_duty_ctrl( u8_sc_g_throttle_rc_ch_duty_target, *u16_duty_now );           /* 狙いのdutyを出したときに回転数が変速閾値超えたなら、勝手に変速するだけ duty制御側は特に何もしない */
            break;

        case SHIFT_SEQ_CLUTCH_OFF_BFORE_CHG:        /* PIの応答を考えて、最初にクラッチ切った時からPI制御開始する */
        case SHIFT_SEQ_CLUTCH_OFF_AFTER_CHG:
        case SHIFT_SEQ_CLUTCH_OFF_SHIFT_CHG:
            /* 変速中 */
            u16_duty_new = func_sc_s_rpm_pi_ctrl( u16_speedsens_g_speed_ave_1stgear, u16_speedsens_g_speed_ave_mtr, *u16_duty_now );     /* 回転数追従制御によるduty指令値 */
            break;
        
        default:
            u16_duty_new = SC_IDLING_DUTY;                /*  */
            break;
    }

    /* 現在の出力dutyを更新 */
    *u16_duty_now = u16_duty_new;
}


/**************************************************************/
/*  Function:                                                 */
/*  一定duty制御処理                                           */
/*  指令dutyに追従させる制御                                    */
/*  急な変化に対しては緩やかに追従させる                         */
/*  アクセルレスポンスに相当する                                 */
/**************************************************************/
static u16 func_sc_s_duty_ctrl( u8 u8_rc_ch_duty_target, u16 u16_duty_now )
{
    u32 u32_duty_now;
    u32 u32_calc_buff;
    u16 u16_duty_target;

    /* 初期化 */
    u32_duty_now = (u16)0;
    u32_calc_buff = (u32)0;
    u16_duty_target = (u16)0;

    /* 初期値代入 */
    u32_duty_now = (u32)u16_duty_now;           /* オーバーフローチェックのために一時的に32へ拡張 */

    /* スケーリング */
    /* 回転数制御でない側は、RCプロポからの入力dutyをそのまま出力dutyとして出力する */
    /* インバータへの出力duty指定は、回転数制御時を考慮して16bitに拡張しているので、 */
    /* スロットルからの入力dutyを、再度スケーリングする必要がある */
    u32_calc_buff = func_ud_g_calcmul_2x2_byte( (u16)u8_rc_ch_duty_target, (u16)TIMER_INVERTER_DUTY_MAX_CNT );
    u32_calc_buff = func_ud_g_calcdiv_4x4_byte( u32_calc_buff, (u32)RC_CH_DUTY_100P );
    u16_duty_target = (u16)u32_calc_buff;


    /* duty更新周期カウント計算 */
    if( u8_sc_s_duty_update_cycle_cnt < U8_MAX )
    {
        u8_sc_s_duty_update_cycle_cnt++;
    }

    if( u8_sc_s_duty_update_cycle_cnt > SC_CONST_DUTY_CTRL_UPDATE_CYCLE )
    { /* duty更新周期以上のカウントになった */
        if( (u32)u16_duty_target > u32_duty_now )
        { /* これから上げる */
            u32_duty_now += (u32)SC_CONST_DUTY_CTRL_INC;
#if 1
            if( u32_duty_now > (u32)U16_MAX )
            { /* 加算後、u16型の最大値を超えている */
                u32_duty_now = (u32)U16_MAX;
            }

            if( (u32)u16_duty_target < u32_duty_now )
            { /* 目標値を超えた */
                u32_duty_now = (u32)u16_duty_target;            /* 目標値をセット */
            }
#else
            /* この判定方法は一度のインクリメント量が小さく、足した程度でオーバーフローしない場合にしか使えない。 */
            /* 結局型拡張しないと、真の意味でのオーバーフロー/アンダーフローは防止できない */
            if( u8_duty_now_buff > RC_CH_DUTY_100P )
            {
                u8_duty_now_buff = RC_CH_DUTY_100P;
            }
#endif
        }
        else if( (u32)u16_duty_target < u32_duty_now )
        { /* これから下げる */
            if( u32_duty_now > (u32)SC_CONST_DUTY_CTRL_DEC )
            { /* まだ減算可能 */
                u32_duty_now -= (u32)SC_CONST_DUTY_CTRL_DEC;
            }
            else
            { /* これ以上減算不可 */
                u32_duty_now = (u32)u16_duty_target;            /* 目標値をセット */
            }
        }
        else
        { /* 同一 */
            ;           /* 特に更新はしない */
        }

        u8_sc_s_duty_update_cycle_cnt = (u8)0;              /* カウンタリセット */
    }

    /* 出力duty更新 */
    return (u16)u32_duty_now;
}


/**************************************************************/
/*  Function:                                                 */
/*  PIC制御                                                   */
/*  変速時はエンジンと負荷軸の回転数を合わせることが必要になる     */
/*  PI制御により、回転数をおおよそ追従させる                     */
/**************************************************************/
static u16 func_sc_s_rpm_pi_ctrl( u16 u16_speed_target, u16 u16_speed_now, u16 u16_duty_now )
{
    u16 u16_duty_now_buff;
    s16 s16_speed_diff;
    s16 s16_term_p;
    s16 s16_term_i;
    s16 s16_term_pi;
    s16 s16_duty_new;
    s16 s16_calc_buff;

    /* 初期化 */
    u16_duty_now_buff = (u16)0;
    s16_speed_diff = (s16)0;
    s16_term_p = (s16)0;
    s16_term_i = (s16)0;
    s16_term_pi = (s16)0;
    s16_duty_new = (s16)0;

    s16_calc_buff = (s16)0;

    /* 初期値の設定 */
    if( ( u8_shift_g_shifting_sequence_before != SHIFT_SEQ_CLUTCH_OFF_SHIFT_CHG ) &&
        ( u8_shift_g_shifting_sequence == SHIFT_SEQ_CLUTCH_OFF_SHIFT_CHG ) )
    { /* 変速のためのPI制御初回 */
        s16_sc_s_integral_cnt = (s8)0;
    }
    s16_term_i = s16_sc_s_integral_cnt;         /* 前回までにためた積分項 */


    /* 範囲制限 */
    /* 符号型で取り扱える最大値を超えている場合は入力を修正する */
    /* 符号付を使用するため、誤計算防止のために範囲を明確に再設定する */
    /* 元の入力は符号なしなので、下側の補正はナシでOK */
    if( u16_speed_target > (u16)32767 )
    { /* 符号付に変換後、目標値が負の数になってしまう値 */
        u16_speed_target = (u16)32767;      /* 符号あり16bit 最大値 */
    }

    if( u16_speed_now > (u16)32767 )
    {
        u16_speed_now = (u16)32767;         /* 符号あり16bit 最大値 */
    }

    /* 初期dutyの設定 */
    /* duty制御と回転数制御で切り替えが発生するので、前回値は今の出力dutyとする */


    /* 回転数偏差を取得 */
    s16_speed_diff = (s16)u16_speed_target - (s16)u16_speed_now;

    /* P項の計算 */
    if( s16_speed_diff >= (s16)0 )
    { /* 偏差が正 */
        s16_calc_buff = s16_speed_diff >> SC_RPM_CTRL_GAIN_P_DIV2N;

        s16_term_p = s16_calc_buff;
    }
    else /* 符号付で0未満 */
    { /* 偏差が負 */
        s16_calc_buff = -s16_speed_diff;
        s16_calc_buff = s16_calc_buff >> SC_RPM_CTRL_GAIN_P_DIV2N;    /* 整数状態でビットシフト */
        s16_calc_buff = -s16_calc_buff;

        s16_term_p = s16_calc_buff;
    }
    

    /* I項の計算 */
    /* 前回までの値を累積 */
    if( u8_sc_s_pi_saturate_flag == CLEAR )
    { /* 出力 飽和していない */
        s16_term_i = s16_term_i + s16_speed_diff;           /* 前回までの積分値+今回の偏差 */

        if( s16_term_i >= (s16)0 )
        { /* 積分項は正 */
            s16_term_i = s16_term_i >> SC_RPM_CTRL_GAIN_I_DIV2N;            /* Iゲイン積算 */
        }
        else
        { /* 積分項は負 */
            s16_calc_buff = -s16_term_i;                                    /* 絶対値に戻す */
            s16_calc_buff = s16_calc_buff >> SC_RPM_CTRL_GAIN_I_DIV2N;      /* Iゲイン積算 */
            s16_calc_buff = -s16_calc_buff;                                 /* 符号を戻す */

            s16_term_i = s16_calc_buff;
        }
    }
    else
    { /* 出力 飽和状態 */
        ;           /* I項現在の値を維持 */
    }

    /* 操作量：P項 + I項 */
    s16_term_pi = s16_term_p + s16_term_i;
    s16_duty_new = (s16)u16_duty_now + s16_term_pi;           /* 常に正の範囲の出力duty + 符号を持つ操作量 の和 ※あくまで現在の出力dutyを初期値として偏差を制御して、一定duty制御->PI制御切り替え時に緩やかにつなぎたい */
                                                              /* 回転数が一致した場合、PI制御出力は0になる設計をしてみた。 */

    /* 範囲制限 */
    if( s16_duty_new > (s16)RC_CH_DUTY_100P )
    { /* 合計操作量が100%を超えた */
        s16_duty_new =  (s16)RC_CH_DUTY_100P;
        u8_sc_s_pi_saturate_flag = SET;
    }
    else if( s16_duty_new < (s16)RC_CH_DUTY_0P )
    { /* 操作量が0%を下回った */
        s16_duty_new =  (s16)RC_CH_DUTY_0P;
        u8_sc_s_pi_saturate_flag = SET;
    }
    else
    {
        u8_sc_s_pi_saturate_flag = CLEAR;
    }


    /* グローバル変数 更新 */
    s16_sc_s_integral_cnt = s16_term_i;             /* 積分器 更新 */
    
    return (u16)s16_duty_new;           /* 出力duty 更新 */
}






