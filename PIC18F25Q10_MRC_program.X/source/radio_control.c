/*
 * File:   radio_control.c
 * Author: ICE_MEGANE
 * ラジコン用リモコン処理全般
 * Created on 2020/03/5, 22:26
 */

#include <xc.h>
#include "./mcufunc/pic18f25q10.h"
#include "userdefine.h"


#include "./mcufunc/gpio.h"
#include "./radio_control.h"


/* 割り込み定義 */
/* ※割り込み処理側をあまりごちゃごちゃにしたくないので、関連する関数ごとにファイル分けしとく※ */
/* ！あとで定義位置要件等！ */
/* 配列要素指定用 */
#define RC_DUTY_IDX_SPEED                 ((u8)0)
#define RC_DUTY_IDX_SHIFT_MODE            ((u8)1)
#define RC_DUTY_IDX_SPEED_GAIN            ((u8)2)
#define RC_DUTY_IDX_SHIFT_UPDOWN          ((u8)3)
#define RC_DUTY_IDX_SHIFT_REV_LIMIT       ((u8)4)


#define RC_INT_IOC_FLAG_SPEED             IOCAF0
#define RC_INT_IOC_FLAG_SHIFT_MODE        IOCAF1
#define RC_INT_IOC_FLAG_SPEED_GAIN        IOCAF2
#define RC_INT_IOC_FLAG_SHIFT_UPDOWN      IOCAF3
#define RC_INT_IOC_FLAG_SHIFT_REV_LIMIT   IOCAF4

#define RC_INT_IOC_FLAG_NUM               ((u8)5)


/* 設定関連 */
#define RC_DUTY_PULSE_END_LOGIC           HI       /* パルスON区間終了時のポート状態  ※LPF回路あるのでLOW-Active */
#define RC_DUTY_JUDGE_TIMEOUT_CNT         ((u8)5)


u8 u8_rc_g_duty_judge_sequence;             /* パルス幅測定シーケンス */

/* マイコンレジスタでチャネル指定できる機能があるが、ややこしくなるので使わない方がいいかも */
/* 変数宣言 */
u8 u8_rc_g_duty_speed;              /* 前進/後進 速度入力 */
u8 u8_rc_g_duty_shift_mode;         /* シフトチェンジモード入力 (セミオートマ/オート) */
u8 u8_rc_g_duty_speed_gain;         /* 加速度合い(エンジン応答)入力 */
u8 u8_rc_g_duty_shift_updown;       /* シフトアップダウン入力 */
u8 u8_rc_g_duty_rev_limit;          /* レブリミット閾値入力 */

static u8 u8_rc_s_duty_judge_timeout_flag;         /* パルス幅取得 タイムアウト */
static u8 u8_rc_s_duty_judge_timeout_cnt;          /* パルス幅取得 失敗カウント */


typedef struct rc_duty_get_status
{
    u8 *ioc_port_reg_addr;              /* IOCポートレジスタへのポインタ */
    u8 *ioc_flag_reg_addr;              /* IOCフラグレジスタへのポインタ(アドレス指定しておく) */
    u8 u8_judge_complete_flag;          /* 指定のチャネルのduty測定完了フラグ */
    u8 u8_ch_duty_val;                  /* 取得したdutyのパルス幅値 ( タイマレジスタからの取得値 ) */
}rc_duty_status;


/* メモリからの毎回呼び出しを指定する */
/* 本当はアドレスだけはconstにしておきたいが？　対応要検討 */
volatile static rc_duty_status rc_duty_status_tbl[ RC_INT_IOC_FLAG_NUM ] =
{ /* IOCポートレジスタのアドレス,     IOCフラグレジスタのアドレス,    確認完了フラグ,     タイマレジスタ保持値  */
    { &GPIO_IN_RC_CH_SPEED,                &RC_INT_IOC_FLAG_SPEED,            CLEAR,        (u8)0},
    { &GPIO_IN_RC_CH_SHIFT_MODE,           &RC_INT_IOC_FLAG_SHIFT_MODE,       CLEAR,        (u8)0},
    { &GPIO_IN_RC_CH_SPEED_GAIN,           &RC_INT_IOC_FLAG_SPEED_GAIN,       CLEAR,        (u8)0},
    { &GPIO_IN_RC_CH_SHIFT_UPDOWN,         &RC_INT_IOC_FLAG_SHIFT_UPDOWN,     CLEAR,        (u8)0},
    { &GPIO_IN_RC_CH_SHIFT_REV_LIMIT,      &RC_INT_IOC_FLAG_SHIFT_REV_LIMIT,  CLEAR,        (u8)0},
};
/* 特定のビットめがけての1bitポインタってどうなの？ SFR読み出し時の解釈が知りたい */
/* リードモディファイ的にどうなるか。 */


/* 関数プロトタイプ宣言 */
static void func_rc_s_get_duty( void );


/* グローバル変数 */
/**************************************************************/
/*  Function:                                                 */
/*  変数初期設定                                               */
/*                                                            */
/**************************************************************/
void func_rc_g_init( void )
{
    u8 u8_loopcnt;
    
    u8_rc_s_duty_judge_timeout_flag = CLEAR;
    u8_rc_s_duty_judge_timeout_cnt = (u8)0;
    u8_rc_g_duty_judge_sequence = RC_SEQ_DUTY_JUDGE_INIT;
    
    u8_rc_g_duty_speed          = (u8)0;
    u8_rc_g_duty_shift_mode     = (u8)0;
    u8_rc_g_duty_speed_gain     = (u8)0;
    u8_rc_g_duty_shift_updown   = (u8)0;
    u8_rc_g_duty_rev_limit      = (u8)0;
}



/**************************************************************/
/*  Function:                                                 */
/*  main関数                                                   */
/*                                                            */
/*                                                            */
/**************************************************************/
void func_rc_g_main( void )
{
    func_rc_s_get_duty();
}


/**************************************************************/
/*  Function:                                                 */
/*  入力パルスのduty識別・決定関数                              */
/*                                                            */
/*  ほんとはMCUハードウェア機能だけで完結させたいが、タイマ数が足りないのでパワー実装 */
/* @@多少判定処理のクロック誤差が生まれると思うが、実測で誤差として流せるか確認して判断したい */
/* ポート指定を容易に行えるように&同一タイミングのパルスはなるべく同じタイマ値を取得したいので、この処理内で判別する */
/**************************************************************/
void func_rc_g_duty_detection( void )
{   
    u8 u8_loopcnt;
    u8 u8_timer_register_by_ioc_int;
    u8 u8_duty_judge_ch_remain_flag;
    
    /* ローカル変数初期化 */
    u8_loopcnt = (u8)0;
    u8_timer_register_by_ioc_int = (u8)0;
    u8_duty_judge_ch_remain_flag = CLEAR;
    
    
    switch ( u8_rc_g_duty_judge_sequence )
    {
        case RC_SEQ_DUTY_JUDGE_INIT:
            /* 初回のトリガ操作を待機中・・・ */
            /* パルス幅測定 基準チャネルの確認 */
            if( ( RC_INT_IOC_FLAG_SPEED == SET ) &&
                ( GPIO_IN_RC_CH_SPEED == LOW ) )
            { /* CH0のパルス変化入力 & HI->LOWへの変化だった */
                u8_rc_g_duty_judge_sequence = RC_SEQ_DUTY_JUDGE_START;
            }
            
            /* タイマスタート */
            /* タイマレジスタクリア */
            /* @@後で追加 */
            
            break;
            
        case RC_SEQ_DUTY_JUDGE_START:
            IOCIE = CLEAR;                                  /* IOC割り込み禁止：この時点で同時に発生してないフラグは次に回す */
            u8_timer_register_by_ioc_int = TMR5;            /* 割り込みが来た時点でいったんタイマレジスタの値を保持しておく */
            
            
            /* 順番にチャネル判定 */
            if( u8_rc_s_duty_judge_timeout_flag == SET )
            { /* dutyのパルス幅のフォーマットから逸脱している(取得タイミングずれた疑惑) */
                u8_rc_g_duty_judge_sequence = RC_SEQ_DUTY_JUDGE_TIMEOUT;
            }
            else
            { /* 全チャネル パルス幅測定中 */
                /* IOC割り込みフラグが立ってるポートを確認 */
                for( u8_loopcnt = (u8)0; u8_loopcnt < RC_INT_IOC_FLAG_NUM ; u8_loopcnt++ )
                {
                    if( ( *(rc_duty_status_tbl[u8_loopcnt].ioc_port_reg_addr) == RC_DUTY_PULSE_END_LOGIC ) &&
                        ( rc_duty_status_tbl[u8_loopcnt].u8_judge_complete_flag == CLEAR ) )
                    { /* IOC割り込み発生時点の極性があっている & まだパルス幅測定完了前のチャネル */
                        if( *(rc_duty_status_tbl[u8_loopcnt].ioc_flag_reg_addr) != (u8)0  )
                        { /* IOC割り込み発生中のポートを発見 */
                            *(rc_duty_status_tbl[u8_loopcnt].ioc_flag_reg_addr) = CLEAR;                         /* @@ビット指定で消せるのか要確認 多チャンネル対応のため。 */
                            rc_duty_status_tbl[u8_loopcnt].u8_ch_duty_val = u8_timer_register_by_ioc_int;        /* IOC割り込み発生時点でのタイマ値を取得 */
                            rc_duty_status_tbl[u8_loopcnt].u8_judge_complete_flag = SET;                         /* 対象のチャネルのパルス幅測定 完了 */
                        }
                    }
                    else
                    { /* ACTIVE-LOGICではないタイミングでパルス幅測定してしまった */
                        u8_rc_g_duty_judge_sequence = RC_SEQ_DUTY_JUDGE_TIMEOUT;        /* とりあえずタイムアウトに振っておく */
                    }
                    
                    /* まだ割り込み未発生のチャネルがある */
                    if( rc_duty_status_tbl[u8_loopcnt].u8_judge_complete_flag == CLEAR )
                    {
                        u8_duty_judge_ch_remain_flag = SET;
                    }
                }
                
                /* 全チャネル パルス幅測定 完了判定 */
                if( u8_duty_judge_ch_remain_flag == CLEAR )
                {
                    u8_rc_g_duty_judge_sequence = RC_SEQ_DUTY_JUDGE_INIT;                           /* 初期状態へ戻す */
                }
            }
            
            IOCIE = SET;
            break;
            
        case RC_SEQ_DUTY_JUDGE_TIMEOUT:
            /* タイムアウト発生 */  /* 現状どのチャネルかどうかまでは判断しない？ */
            if( u8_rc_s_duty_judge_timeout_cnt < U8_MAX )
            {
                u8_rc_s_duty_judge_timeout_cnt++;
            }
            
            if( u8_rc_s_duty_judge_timeout_cnt > RC_DUTY_JUDGE_TIMEOUT_CNT )
            { /* 何度もタイムアウトが発生している */
                u8_rc_g_duty_judge_sequence = RC_SEQ_DUTY_JUDGE_FAILE;
            }
            else
            {
                u8_rc_g_duty_judge_sequence = RC_SEQ_DUTY_JUDGE_INIT;
            }
        
            break;
            
        case RC_SEQ_DUTY_JUDGE_FAILE:
            /* パルス幅測定失敗 */
            /* ここに来た時点で設計ミスなので、デバッグ時点ではあえて抜けるないようにしておく。 */
            for( u8_loopcnt = (u8)0; u8_loopcnt < RC_INT_IOC_FLAG_NUM ; u8_loopcnt++ )
            {
                *(rc_duty_status_tbl[u8_loopcnt].ioc_port_reg_addr) != (u8)0;                        /* 読み取り専用なので特に意味なし */
                *(rc_duty_status_tbl[u8_loopcnt].ioc_flag_reg_addr) != (u8)0;                        /* 割り込みフラグはクリアしておく */
                rc_duty_status_tbl[u8_loopcnt].u8_ch_duty_val = (u8)0;                               /* duty0%設定 */
                rc_duty_status_tbl[u8_loopcnt].u8_judge_complete_flag = SET;                         /* duty取得完了フラグ クリア */
            }
            
            u8_rc_s_duty_judge_timeout_cnt = (u8)0;
        
            break;
        
        default:
            break;
    }
}



/**************************************************************/
/*  Function:                                                 */
/*  入力パルス幅の一時保存値を回収                              */
/*  1msタスク側で回収する値                                     */
/**************************************************************/
static void func_rc_s_get_duty( void )
{
    /* 値の取得時も割り込み禁止にはしない */
    /* グローバル変数へ移植 */   
    u8_rc_g_duty_speed          = rc_duty_status_tbl [RC_DUTY_IDX_SPEED ].u8_ch_duty_val;
    u8_rc_g_duty_shift_mode     = rc_duty_status_tbl [RC_DUTY_IDX_SHIFT_MODE ].u8_ch_duty_val;
    u8_rc_g_duty_speed_gain     = rc_duty_status_tbl [RC_DUTY_IDX_SPEED_GAIN ].u8_ch_duty_val;
    u8_rc_g_duty_shift_updown   = rc_duty_status_tbl [RC_DUTY_IDX_SHIFT_UPDOWN ].u8_ch_duty_val;
    u8_rc_g_duty_rev_limit      = rc_duty_status_tbl [RC_DUTY_IDX_SHIFT_REV_LIMIT ].u8_ch_duty_val;
}




/* 静的関数 */



