/*
 * File:   speedcontrol.c
 * Author: ICE_MEGANE
 *
 * Created on 2020/03/5, 22:26
 */

/* このファイル内で設定する処理は、エンジンとアクセル制御のみ担当 */
/* アクセル制御の結果、エンジン回転数が所定の域に到達した場合シフトチェンジが必要になるが、 */
/* これはshift.c側で実施する。 */

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
#include "./tools/hbridge.h"

#include "speedcontrol.h"


/* シーケンス */
#define SC_DRIVE_STATUS_IDLING          ((u8)0)                /* スロットルオフ中 */
#define SC_DRIVE_STATUS_DRIVE_CW        ((u8)1)                /* 駆動中CW  */
#define SC_DRIVE_STATUS_DRIVE_CCW       ((u8)2)                /* 駆動中CCW */
#define SC_DRIVE_STATUS_BRAKE_CW        ((u8)3)                /* ブレーキCW  */
#define SC_DRIVE_STATUS_BRAKE_CCW       ((u8)4)                /* ブレーキCCW */


/* パラメータ */
#define SC_THROTTLE_DEFINE_0P           RC_CH_DUTY_50P          /* スロットル中立時の入力 */
#define SC_THROTTLE_HYSTERESIS          RC_CH_DUTY_5P           /* スロットル操作のヒステリシス */
#define SC_IDLING_DUTY                  HBRIDGE_DUTY_10P        /* アイドリング中のduty */

#define SC_RPM_CTRL_GAIN_P_DIV2N        ((u8)6)                 /* Pゲイン 割り算形式で実装　※大きくするほど変化が緩やかになる */
#define SC_RPM_CTRL_GAIN_I_DIV2N        ((u8)6)                 /* Iゲイン 割り算形式で実装　※大きくするほど変化が緩やかになる */   /* @@大きすぎる(割り算しすぎ)だと、i項がs16の範囲を外れてしまうので注意。飽和状態で32767は超えない設計にすること！！！！ */

#define SC_THROTTLE_DIR_RESET_TIME      ((u8)100)               /* スロットル中立 遷移待機カウント */


/* ※そもそもプロポからのduty更新指令が50ms間隔でしか送れないので、若干意味がない設定。何もしなくても緩やかに変化してくれる？ */
#define SC_CONST_DUTY_CTRL_UPDATE_CYCLE ((u8)20)                /* Xmsに1回更新 例:1%-20ms -> 100%-2000ms  */
#define SC_CONST_DUTY_CTRL_INC          HBRIDGE_DUTY_1P         /* 一定duty制御での加速量 */
#define SC_CONST_DUTY_CTRL_DEC          HBRIDGE_DUTY_1P         /* 一定duty制御での減速量 */


u8 u8_sc_s_drive_status;

u8 u8_sc_s_throttle_dir_reset_cnt;      
u8 u8_sc_s_throttle_dir;
u8 u8_sc_s_throttle_dir_before;
u8 u8_sc_s_throttle_dir_1state_before;
u8 u8_sc_g_throttle_rc_ch_duty_target;        /* 目標duty */
u8 u8_sc_g_throttle_duty_ref;           /* 指令duty */


/* duty制御 */
static u8 u8_sc_s_duty_update_cycle_cnt;

/* 回転数PI制御 */
u8 u8_sc_s_pi_saturate_flag;
s16 s16_sc_s_integral_cnt;


/* プロトタイプ宣言 */
static void func_sc_s_throttle_per_dir_update( void );
static void func_sc_s_throttle_threshold_update( u8 *u8_forward_th, u8 *u8_backward_th );
static void func_sc_s_drive_status_update( void);
static void func_sc_s_calc_duty( void );
static u16 func_sc_s_const_duty_ctrl( u8 u8_duty_target, u16 u16_duty_now );
static u16 func_sc_s_rpm_pi_ctrl( u16 u16_speed_target, u16 u16_speed_now, u16 u16_duty_now );
static void func_sc_s_output_control( void );

/**************************************************************/
/*  Function:                                                 */
/*                                                            */
/**************************************************************/
void func_speedcontrol_g_init( void )
{
    u8_sc_s_throttle_dir        = SC_THROTTLE_DIR_NONE;
    u8_sc_s_throttle_dir_before = SC_THROTTLE_DIR_NONE;
    u8_sc_s_throttle_dir_1state_before = SC_THROTTLE_DIR_NONE;

    u8_sc_g_throttle_rc_ch_duty_target = RC_CH_DUTY_0P;
    u8_sc_g_throttle_duty_ref    = RC_CH_DUTY_0P;

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
    func_sc_s_throttle_per_dir_update();     /* dutyをスロットル中心が0になるように再計算 */
    func_sc_s_drive_status_update();         /* スロットル操作による車体の動作状態指令更新 */
    func_sc_s_calc_duty();                   /* 出力duty更新       */
    func_sc_s_output_control();              /* 出力制御 */
}


/**************************************************************/
/*  Function:                                                 */
/*  スロットル入力量 判断処理                                   */
/*  レバー操作に対して0~100の入力としているため、これを前後に振り分ける */
/*  分解能的に成立していないところがあるが、実害なのでOKとした     */
/**************************************************************/
static void func_sc_s_throttle_per_dir_update( void )
{
    u8 u8_throttle_per_result;
    u8 u8_throttle_dir_result;
    u8 u8_duty_input;
    u8 u8_duty_forward_th;
    u8 u8_duty_backward_th;
    u8 u8_duty_recalc_base;
    u8 u8_duty_recalc_gap;

    u32 u32_calc_buff;

    /* 初期化 */ 
    u8_throttle_per_result = (u8)0;
    u8_throttle_dir_result = SC_THROTTLE_DIR_CW;

    u8_duty_input = (u8)0;
    u8_duty_forward_th = (u8)0;
    u8_duty_backward_th = (u8)0;
    u8_duty_recalc_base = (u8)0;
    u8_duty_recalc_gap = (u8)0;

    /*-----------------------------------------------------------------------*/

    u8_duty_input = u8_rc_g_ch_duty_tbl[RC_DUTY_CH_THROTTLE];
    u8_sc_s_throttle_dir_before = u8_sc_s_throttle_dir;                 /* 前回のスロットル設定方向を保存 */

    /* スロットル閾値更新 */
    func_sc_s_throttle_threshold_update( &u8_duty_forward_th, &u8_duty_backward_th );


    /* 入力判定 */
    if( u8_duty_input > u8_duty_forward_th )
    { /* 前進方向のduty入力 */
        u8_duty_recalc_base = RC_CH_DUTY_100P - u8_duty_forward_th;
        u8_duty_recalc_gap = u8_duty_input - u8_duty_forward_th;

        u32_calc_buff = func_ud_g_calcmul_2x2_byte( (u16)u8_duty_recalc_gap, (u16)RC_CH_DUTY_100P );
        u32_calc_buff = func_ud_g_calcdiv_4x4_byte( u32_calc_buff, (u32)u8_duty_recalc_base );

        u8_throttle_per_result = (u8)u32_calc_buff;            /* 前進方向のduty(再計算後)を設定 */
        u8_throttle_dir_result = SC_THROTTLE_DIR_CW;      /* 前進 */

        u8_sc_s_throttle_dir_reset_cnt = (u8)0;                /* 中立以降カウント クリア */
    }
    else if( u8_duty_input < u8_duty_backward_th )
    { /* 後退方向のduty入力 */
        u8_duty_recalc_base = u8_duty_backward_th;
        u8_duty_recalc_gap = u8_duty_backward_th - u8_duty_input;

        u32_calc_buff = func_ud_g_calcmul_2x2_byte( (u16)u8_duty_recalc_gap, (u16)RC_CH_DUTY_100P );
        u32_calc_buff = func_ud_g_calcdiv_4x4_byte( u32_calc_buff, (u32)u8_duty_recalc_base );

        u8_throttle_per_result = (u8)u32_calc_buff;            /* 後進方向のduty(再計算後)を設定 */
        u8_throttle_dir_result = SC_THROTTLE_DIR_CCW;     /* 後進 */

        u8_sc_s_throttle_dir_reset_cnt = (u8)0;                /* 中立以降カウント クリア */
    }
    else
    { /* 中立状態 */
        u8_throttle_per_result = RC_CH_DUTY_0P;                /* リモコンのオフセット設定に依存して中心がずれちゃうっぽい。あとT8FBの可変抵抗がBカーブになっているような雰囲気あり？？？ */

        if( u8_sc_s_throttle_dir_reset_cnt < U8_MAX )
        {
            u8_sc_s_throttle_dir_reset_cnt++;
        }

        if( u8_sc_s_throttle_dir_reset_cnt > SC_THROTTLE_DIR_RESET_TIME )
        { /* 中立への復帰にはdelayを設ける ※ブレーキ判定のため */
            u8_throttle_dir_result = SC_THROTTLE_DIR_NONE;      /* 時間内に B->F or F->B へ切り替えた場合、中立を経過せずに切り替わる */
        }
    }

    /*==================*/
    /* グローバル変数更新 */
    /*==================*/
    u8_sc_g_throttle_rc_ch_duty_target = u8_throttle_per_result;
    u8_sc_s_throttle_dir = u8_throttle_dir_result;

    if( u8_sc_s_throttle_dir != u8_sc_s_throttle_dir_before )
    { /* 前回の"ステート"保存 */
        u8_sc_s_throttle_dir_1state_before = u8_sc_s_throttle_dir_before;       /* 次の1ms以降もこの値は保持される。 */
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
/*   */
/**************************************************************/

/**************************************************************/
/*  Function:                                                 */
/*  出力duty計算処理                                            */
/*  一定dutyでの制御と、負荷軸回転数に合わせに行く制御を切り替える  */
/*  通常変速状態ではduty制御を、変速時のシフトアップ・シフトダウンは */
/*  負荷軸側回転数を目標値として合わせに行く                      */
/**************************************************************/
static void func_sc_s_calc_duty( void )
{
    u16 u16_duty_now;
    u16 u16_duty_new;

    /* 初期化 */
    /* 注意：この関数内で扱うdutyはすべてhbridgへの出力duty ※RC-CH-DUTYではない */
    u16_duty_now = HBRIDGE_DUTY_0P;
    u16_duty_new = HBRIDGE_DUTY_0P;

    /* グローバル変数更新 */
    u16_duty_now = u16_hbridge_g_output_duty;

    /* 変速シーケンスに依存して出力するdutyが変わってくるので、switch文にしてみた */
    switch ( u8_sc_s_drive_status )
    {
        case SC_DRIVE_STATUS_IDLING:
            u16_duty_new = SC_IDLING_DUTY;            /* 停止中もアイドリングdutyを出しておく */
            break;
        
        case SC_DRIVE_STATUS_DRIVE_CW:
        case SC_DRIVE_STATUS_DRIVE_CCW:
            if( ( u8_shift_g_shifting_sequence == SHIFT_SEQ_CLUTCH_OFF_BFORE_CHG ) ||
                ( u8_shift_g_shifting_sequence == SHIFT_SEQ_CLUTCH_OFF_AFTER_CHG ) ||
                ( u8_shift_g_shifting_sequence == SHIFT_SEQ_CLUTCH_OFF_SHIFT_CHG ) )
            { /* 変速移行期間 */
                u16_duty_new = func_sc_s_rpm_pi_ctrl( u16_speedsens_g_speed_ave_1stgear, u16_speedsens_g_speed_ave_mtr, u16_duty_now );     /* 回転数追従制御によるduty指令値 */
            }
            else
            { /*  */
                u16_duty_new = func_sc_s_const_duty_ctrl( u8_sc_g_throttle_rc_ch_duty_target, u16_duty_now );           /* 狙いのdutyを出したときに回転数が変速閾値超えたなら、勝手に変速するだけ duty制御側は特に何もしない */
            }
            break;

        case SC_DRIVE_STATUS_BRAKE_CW:
        case SC_DRIVE_STATUS_BRAKE_CCW:
            u16_duty_new = HBRIDGE_DUTY_0P;
            break;
    
        default:
            break;
    }

    /*===================*/
    /* グローバル変数更新 */
    /*===================*/
    u16_hbridge_g_output_duty = u16_duty_new;
}


/**************************************************************/
/*  Function:                                                 */
/*  一定duty制御処理                                           */
/*  指令dutyに追従させる制御                                    */
/*  急な変化に対しては緩やかに追従させる                         */
/*  アクセルレスポンスに相当する                                 */
/**************************************************************/
static u16 func_sc_s_const_duty_ctrl( u8 u8_rc_ch_duty_target, u16 u16_duty_now )
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
    u32_calc_buff = func_ud_g_calcmul_2x2_byte( (u16)u8_rc_ch_duty_target, HBRIDGE_DUTY_100P );
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


/**************************************************************/
/*  Function:                                                 */
/*  出力制御                                                   */
/**************************************************************/
static void func_sc_s_output_control( void )
{
    u8 u8_hbridge_state_req;
    
    u8_hbridge_state_req = CLEAR;

    if( ( u8_sc_s_drive_status == SC_DRIVE_STATUS_BRAKE_CW ) || 
        ( u8_sc_s_drive_status == SC_DRIVE_STATUS_BRAKE_CCW ) )
    { /* ブレーキ要求あり */
        u8_hbridge_state_req = HBRIDGE_OUTPUT_BRAKE;
    }
    else if( u8_sc_s_drive_status == SC_DRIVE_STATUS_DRIVE_CW )
    {
        u8_hbridge_state_req = HBRIDGE_OUTPUT_CW;
    }
    else if( u8_sc_s_drive_status == SC_DRIVE_STATUS_DRIVE_CCW )
    {
        u8_hbridge_state_req = HBRIDGE_OUTPUT_CCW;
    }
    else if( u8_sc_s_drive_status == SC_DRIVE_STATUS_IDLING )
    { /* 空ぶかし方向は前回までのスロットル操作方向に合わせておく */
        if( u8_sc_s_throttle_dir_1state_before == SC_THROTTLE_DIR_CW )
        {
            u8_hbridge_state_req = HBRIDGE_OUTPUT_CW;
        }
        else if( u8_sc_s_throttle_dir_1state_before == SC_THROTTLE_DIR_CCW )
        {
            u8_hbridge_state_req = HBRIDGE_OUTPUT_CCW;
        }
        else
        { /* 一度もスロットル操作がない初期状態 */
            u8_hbridge_state_req = HBRIDGE_OUTPUT_CW;       /* ひとまず前進方向でアイドリング・・・・ */
        }
    }
    else
    {
        ;
    }

    /* Hブリッジ回路への出力要求 */
    /* dutyはそのまま設定する */
    func_hbridge_control_set( u8_hbridge_state_req, u16_hbridge_g_output_duty );
}

/**************************************************************/
/*  Function:                                                 */
/*  動作方向アップデート                                        */
/*  スロットルレバーの操作から、ラジコン本体の動作状態を確定する    */
/*  タミヤのラジコンあたりに入っている仕様を踏襲してみた           */
/**************************************************************/
static void func_sc_s_drive_status_update( void )
{
    switch ( u8_sc_s_drive_status )
    {
    case SC_DRIVE_STATUS_IDLING:
        if( u8_sc_s_throttle_dir == SC_THROTTLE_DIR_CW )
        {
            u8_sc_s_drive_status = SC_DRIVE_STATUS_DRIVE_CW;
        }
        else if( u8_sc_s_throttle_dir == SC_THROTTLE_DIR_CCW )
        {
            u8_sc_s_drive_status = SC_DRIVE_STATUS_DRIVE_CCW;
        }
        else
        {
            ;
        }
        /* ※中立からのブレーキ発生はできない仕様としておく */
        break;

    case SC_DRIVE_STATUS_DRIVE_CW:
        if( ( u8_sc_s_throttle_dir        == SC_THROTTLE_DIR_CCW ) &&
            ( u8_sc_s_throttle_dir_before == SC_THROTTLE_DIR_CW  ) )
        { /* 前進からのブレーキ */
            u8_sc_s_drive_status = SC_DRIVE_STATUS_BRAKE_CW;
        }
        else if( u8_sc_s_throttle_dir == SC_THROTTLE_DIR_NONE )
        { /* スロットルオフされた */
            if( u16_speedsens_g_speed_ave_1stgear <= (u16)0 )
            { /* 車体が停止状態 */
                u8_sc_s_drive_status = SC_DRIVE_STATUS_IDLING;
            }
            else
            {
                ;       /* このシーケンスで待機 */
            }
        }
        else
        {
            ;       /* このシーケンスで待機 */
        }
        break;

    case SC_DRIVE_STATUS_DRIVE_CCW:
        if( ( u8_sc_s_throttle_dir        == SC_THROTTLE_DIR_CW ) &&
            ( u8_sc_s_throttle_dir_before == SC_THROTTLE_DIR_CCW  ) )
        { /* 前進からのブレーキ */
            u8_sc_s_drive_status = SC_DRIVE_STATUS_BRAKE_CCW;
        }
        else if( u8_sc_s_throttle_dir == SC_THROTTLE_DIR_NONE )
        { /* スロットルオフされた */
            if( u16_speedsens_g_speed_ave_1stgear <= (u16)0 )
            { /* 車体が停止状態 */
                u8_sc_s_drive_status = SC_DRIVE_STATUS_IDLING;
            }
            else
            {
                ;       /* このシーケンスで待機 */
            }
        }
        else
        {
            ;       /* このシーケンスで待機 */
        }
        break;

    case SC_DRIVE_STATUS_BRAKE_CW:
        if( u8_sc_s_throttle_dir == SC_THROTTLE_DIR_CW )
        { /* ブレーキキャンセル */
            u8_sc_s_drive_status = SC_DRIVE_STATUS_DRIVE_CW;       /* 駆動状態に戻る */
        }
        else if( u8_sc_s_throttle_dir == SC_THROTTLE_DIR_NONE )
        {
            u8_sc_s_drive_status = SC_DRIVE_STATUS_IDLING;
        }
        else
        {
            ;       /* ここで待機 */
        }
        break;
    
    case SC_DRIVE_STATUS_BRAKE_CCW:
        if( u8_sc_s_throttle_dir == SC_THROTTLE_DIR_CCW )
        { /* ブレーキキャンセル */
            u8_sc_s_drive_status = SC_DRIVE_STATUS_DRIVE_CCW;       /* 駆動状態に戻る */
        }
        else if( u8_sc_s_throttle_dir == SC_THROTTLE_DIR_NONE )
        {
            u8_sc_s_drive_status = SC_DRIVE_STATUS_IDLING;
        }
        else
        {
            ;       /* ここで待機 */
        }
        break;

    default:
        break;
    }
}




