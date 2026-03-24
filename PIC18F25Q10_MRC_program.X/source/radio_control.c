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
#include "./mcufunc/mcu_setup.h"
#include "./radio_control.h"


/* 割り込み定義 */
/* ※割り込み処理側をあまりごちゃごちゃにしたくないので、関連する関数ごとにファイル分けしとく※ */
/* ！あとで定義位置要件等！ */
/* 配列要素指定用 */
#define RC_DUTY_CH_IDX_0                  ((u8)0)
#define RC_DUTY_CH_IDX_1                  ((u8)1)
#define RC_DUTY_CH_IDX_2                  ((u8)2)
#define RC_DUTY_CH_IDX_3                  ((u8)3)
#define RC_DUTY_CH_IDX_4                  ((u8)4)
#define RC_DUTY_CH_IDX_MAX                ((u8)RC_DUTY_CH_IDX_4+(u8)1)


/* 設定関連 */
#define RC_DUTY_PULSE_END_LOGIC           HI       /* パルスON区間終了時のポート状態  ※LPF回路あるのでLOW-Active */
#define RC_DUTY_JUDGE_TIMEOUT_CNT         ((u8)5)


u8 u8_rc_g_duty_judge_sequence;                    /* パルス幅測定シーケンス */

u16 u16_rc_g_duty_speed;                             /* 前進/後進 速度入力 */
u16 u16_rc_g_duty_shift_mode;                        /* シフトチェンジモード入力 (セミオートマ/オート) */
u16 u16_rc_g_duty_speed_gain;                        /* 加速度合い(エンジン応答)入力 */
u16 u16_rc_g_duty_shift_updown;                      /* シフトアップダウン入力 */
u16 u16_rc_g_duty_rev_limit;                         /* レブリミット閾値入力 */

static u8 u8_rc_s_duty_judge_timeout_flag;         /* パルス幅取得 タイムアウト */
static u8 u8_rc_s_duty_judge_timeout_cnt;          /* パルス幅取得 失敗カウント */


/* SFRからの読み止しだけは、メモリからの毎回呼び出しを指定する */
typedef struct rc_duty_get_status
{   /* レジスタ設定 */
    volatile u8 *port_addr;              /* IOCポートレジスタへのポインタ */
    u8 port_bitmask;            /* IOCポートレジスタのビット指定 */
    volatile u8 *flag_addr;              /* IOCフラグレジスタへのポインタ(アドレス指定しておく) */
    u8 flag_bitmask;            /* IOCフラグレジスタのビット指定 */
    
    /* ソフトウェア処理用 */
    u8 u8_judge_complete_flag;  /* 指定のチャネルのduty測定完了フラグ */
    u16 u16_ch_duty_cnt;        /* 取得したdutyのパルス幅値 ( タイマレジスタからの取得値 ) */
}rc_duty_status;


/* ここで全チャネルまとめて指定できるようにしておきたい */
static rc_duty_status rc_duty_get_register_combi_tbl[ RC_DUTY_CH_IDX_MAX ] =
{ /* IOCポート/ビット,  IOCフラグ/ビット,   duty測定完了フラグ,   タイマレジスタ保持値  */
    { &PORTA,    (u8)(1U << 0U),     &IOCAF,      (u8)0,      CLEAR,        (u16)0},      /* RC_DUTY_CH_IDX_0 */
    { &PORTA,    (u8)(1U << 1U),     &IOCAF,      (u8)1,      CLEAR,        (u16)0},      /* RC_DUTY_CH_IDX_1 */
    { &PORTA,    (u8)(1U << 2U),     &IOCAF,      (u8)2,      CLEAR,        (u16)0},      /* RC_DUTY_CH_IDX_2 */
    { &PORTA,    (u8)(1U << 3U),     &IOCAF,      (u8)3,      CLEAR,        (u16)0},      /* RC_DUTY_CH_IDX_3 */
    { &PORTA,    (u8)(1U << 4U),     &IOCAF,      (u8)4,      CLEAR,        (u16)0},      /* RC_DUTY_CH_IDX_4 */
};
/* リードモディファイ的にどうなるか。 */


/* 関数プロトタイプ宣言 */
static void func_rc_s_get_duty( void );
static void func_rc_s_ioc_flag_erase( u8 u8_ioc_ch );
static void func_rc_s_ioc_state_read( u8 *u8_flag_buff, u8 *u8_port_buff ,u8 u8_ioc_ch );


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
    
    u16_rc_g_duty_speed          = (u16)0;
    u16_rc_g_duty_shift_mode     = (u16)0;
    u16_rc_g_duty_speed_gain     = (u16)0;
    u16_rc_g_duty_shift_updown   = (u16)0;
    u16_rc_g_duty_rev_limit      = (u16)0;
}



/**************************************************************/
/*  Function:                                                 */
/*  main関数                                                   */
/*                                                            */
/*                                                            */
/**************************************************************/
void func_rc_g_main( void )
{
    /* @@割り込み関数で更新される変数を、この処理内で再更新しないこと */
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
    u16 u16_timer_register_buff;
    u8 u8_duty_judge_ch_remain_flag;
    
    u8 u8_ioc_flag_buff;
    u8 u8_ioc_port_buff;
    
    /* ローカル変数初期化 */
    u8_ioc_flag_buff = (u8)0;
    u8_ioc_port_buff = (u8)0;
    u8_loopcnt = (u8)0;
    u16_timer_register_buff = (u16)0;
    u8_duty_judge_ch_remain_flag = CLEAR;
    
    
    switch ( u8_rc_g_duty_judge_sequence )
    {
        case RC_SEQ_DUTY_JUDGE_INIT:
            /* 初回のトリガ操作を待機中・・・ */
            /* パルス幅測定 基準チャネルの確認 */
            /* IOCフラグの確認 */
            func_rc_s_ioc_state_read( &u8_ioc_flag_buff, &u8_ioc_port_buff ,RC_DUTY_CH_IDX_0 );
            
            if( ( u8_ioc_flag_buff == SET ) &&
                ( u8_ioc_port_buff == ((u8)~RC_DUTY_PULSE_END_LOGIC )) )
            { /* CH0のパルス変化入力 & HI->LOWへの変化だった */
                u8_rc_g_duty_judge_sequence = RC_SEQ_DUTY_JUDGE_START;
            }
            
            func_mset_g_timer5_clear();             /* タイマ レジスタクリア */
            func_mset_g_timer5_onoff( ON );         /* タイマ カウントスタート */            
            break;
            
        case RC_SEQ_DUTY_JUDGE_START:
            IOCIE = CLEAR;                                              /* IOC割り込み禁止：この時点で同時に発生してないフラグは次に回す */
            u16_timer_register_buff = func_mset_g_timer5_read();          /* 割り込みが来た時点でいったんタイマレジスタの値を保持しておく */
            
            
            /* 順番にチャネル判定 */
            if( u8_rc_s_duty_judge_timeout_flag == SET )
            { /* dutyのパルス幅のフォーマットから逸脱している(取得タイミングずれた疑惑) */
                u8_rc_g_duty_judge_sequence = RC_SEQ_DUTY_JUDGE_TIMEOUT;
            }
            else
            { /* 全チャネル パルス幅測定中 */ 
                /* IOC割り込みフラグが立ってるポートを確認 */
                for( u8_loopcnt = (u8)0; u8_loopcnt < RC_DUTY_CH_IDX_MAX ; u8_loopcnt++ )
                {
                    func_rc_s_ioc_state_read( &u8_ioc_flag_buff, &u8_ioc_port_buff ,u8_loopcnt );
                    
                    if( ( u8_ioc_port_buff == RC_DUTY_PULSE_END_LOGIC ) &&                                       /* IOC割り込み発生時点の極性があっている */
                        ( rc_duty_get_register_combi_tbl[u8_loopcnt].u8_judge_complete_flag == CLEAR ) )         /* まだパルス幅測定完了前のチャネル */
                    {
                        if( u8_ioc_flag_buff == SET )                   /* IOC割り込み発生中のポートを発見 */
                        {
                            func_rc_s_ioc_flag_erase( u8_loopcnt );             /* IOCフラグクリア */
                            rc_duty_get_register_combi_tbl[u8_loopcnt].u16_ch_duty_cnt = u16_timer_register_buff;        /* IOC割り込み発生時点でのタイマ値を取得 */
                            rc_duty_get_register_combi_tbl[u8_loopcnt].u8_judge_complete_flag = SET;                         /* 対象のチャネルのパルス幅測定 完了 */
                        }
                    }
                    else
                    { /* ACTIVE-LOGICではないタイミングでパルス幅測定してしまった */
                        u8_rc_g_duty_judge_sequence = RC_SEQ_DUTY_JUDGE_TIMEOUT;        /* とりあえずタイムアウトに振っておく */
                    }
                    
                    /* まだ割り込み未発生のチャネルがある */
                    if( rc_duty_get_register_combi_tbl[u8_loopcnt].u8_judge_complete_flag == CLEAR )
                    {
                        u8_duty_judge_ch_remain_flag = SET;
                    }
                }
                
                /* 全チャネル パルス幅測定 完了判定 */
                if( u8_duty_judge_ch_remain_flag == CLEAR )
                { /* 全チャネル検索し、1chもduty測定未完了のチャネルがなかった */
                    u8_rc_g_duty_judge_sequence = RC_SEQ_DUTY_JUDGE_INIT;                           /* 待機状態へ戻す */
                    func_mset_g_timer5_clear();             /* タイマ レジスタクリア */
                    func_mset_g_timer5_onoff( OFF );         /* タイマ カウントストップ */
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
            for( u8_loopcnt = (u8)0; u8_loopcnt < RC_DUTY_CH_IDX_MAX ; u8_loopcnt++ )
            {
                // *(rc_duty_get_register_combi_tbl[u8_loopcnt].port_addr) != (u8)0;                     /* 読み取り専用なので特に意味なし */
                func_rc_s_ioc_flag_erase( u8_loopcnt );                                                  /* IOCフラグクリア */
                rc_duty_get_register_combi_tbl[u8_loopcnt].u8_judge_complete_flag = SET;                 /* duty取得完了フラグ クリア */
                rc_duty_get_register_combi_tbl[u8_loopcnt].u16_ch_duty_cnt = (u8)0;                      /* duty0%設定 */
            }
              
            u8_rc_s_duty_judge_timeout_cnt = (u8)0;
            
            /* @@何度もここに来るので、ここに関数記述するのは微妙かもしれない。実害ないのでいったん保留。 */
            func_mset_g_timer5_clear();             /* タイマ レジスタクリア */
            func_mset_g_timer5_onoff( OFF );         /* タイマ カウントスタート */            
        
            break;
        
        default:
            break;
    }
}


/**************************************************************/
/*  Function:                                                 */
/* *u8_flagbuff：引数/変数 IOCフラグ状態                          */
/* *u8_portbuff：引数/変数 IOC割り当てのポート状態                */
/* u8_ioc_ch：引数 IOCのチャネル指定                             */
/**************************************************************/
static void func_rc_s_ioc_state_read( u8 *u8_flag_buff, u8 *u8_port_buff ,u8 u8_ioc_ch )
{
    *u8_flag_buff = *(rc_duty_get_register_combi_tbl[ u8_ioc_ch ].flag_addr);                             /* IOCフラグレジスタの値を取得 */
    *u8_flag_buff = *u8_flag_buff & (rc_duty_get_register_combi_tbl[ u8_ioc_ch ].flag_bitmask);        /* ビットマスク */
    
    if( *u8_flag_buff != (u8)0 )
    { /* IOCフラグレジスタの該当ビットが0ではない */
        *u8_flag_buff = SET;         /* のちの処理でビット位置気にしないよう、この時点で上書きする */
    }
    else
    {
        *u8_flag_buff = CLEAR;
    }
    
    /* IOCポートの確認 */
    *u8_port_buff = *(rc_duty_get_register_combi_tbl[ u8_ioc_ch ].port_addr);                       /* IOCに該当のポート状態を取得 */
    
    if( *u8_port_buff != (u8)0 )
    {
        *u8_port_buff = SET;
    }
    else
    {
        *u8_port_buff = CLEAR;
    }
}

/**************************************************************/
/*  Function:                                                 */
/* IOC割り込みの特定のビットをクリアする                          */
/**************************************************************/
static void func_rc_s_ioc_flag_erase( u8 u8_ioc_ch )
{
    u8 u8_ioc_flag_buff;
    
    u8_ioc_flag_buff = rc_duty_get_register_combi_tbl[ u8_ioc_ch ].flag_bitmask;
    u8_ioc_flag_buff = (u8)~u8_ioc_flag_buff;       /* ビット反転 */
    
    rc_duty_get_register_combi_tbl[ u8_ioc_ch ].flag_bitmask &= u8_ioc_flag_buff;
}



/**************************************************************/
/*  Function:                                                 */
/*  入力パルス幅の一時保存値を回収                              */
/*  1msタスク側で回収する値                                     */
/**************************************************************/
static void func_rc_s_get_duty( void )
{
    /* グローバル変数へ移植 */
    /* 割り込み更新がないタイミングでデータを取り出す */
    if( u8_rc_g_duty_judge_sequence == RC_SEQ_DUTY_JUDGE_INIT )
    { /* 次のパルス開始タイミングまで待機中 (duty更新なし中) */
        u16_rc_g_duty_speed          = rc_duty_get_register_combi_tbl [ RC_DUTY_CH_IDX_0 ].u16_ch_duty_cnt;
        u16_rc_g_duty_shift_mode     = rc_duty_get_register_combi_tbl [ RC_DUTY_CH_IDX_1 ].u16_ch_duty_cnt;
        u16_rc_g_duty_speed_gain     = rc_duty_get_register_combi_tbl [ RC_DUTY_CH_IDX_2 ].u16_ch_duty_cnt;
        u16_rc_g_duty_shift_updown   = rc_duty_get_register_combi_tbl [ RC_DUTY_CH_IDX_3 ].u16_ch_duty_cnt;
        u16_rc_g_duty_rev_limit      = rc_duty_get_register_combi_tbl [ RC_DUTY_CH_IDX_4 ].u16_ch_duty_cnt;
    }
    
}




/* 静的関数 */



