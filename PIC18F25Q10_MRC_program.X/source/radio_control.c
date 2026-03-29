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
#include "./mcufunc/timer_driver.h"
#include "./radio_control.h"


/* 割り込み定義 */
/* ※割り込み処理側をあまりごちゃごちゃにしたくないので、関連する関数ごとにファイル分けしとく※ */
/* ！あとで定義位置要件等！ */
/* 配列要素指定用 */
#define RC_DUTY_CH_THROTTLE               ((u8)0)
#define RC_DUTY_CH_SHIFT_MODE             ((u8)1)
#define RC_DUTY_CH_RESPONSE               ((u8)2)
#define RC_DUTY_CH_PADDLE_SHIFT_INPUT     ((u8)3)
#define RC_DUTY_CH_REV_LIMIT              ((u8)4)
#define RC_DUTY_CH_IDX_MAX                ((u8)RC_DUTY_CH_REV_LIMIT+(u8)1)


/* 設定関連 */
#define RC_DUTY_PULSE_ON_LOGIC            LOW            /* 入力パルスのアクティブロジック */       /* ※IOCAF、IOCANの設定と合わせること */
#define RC_DUTY_JUDGE_TIMEOUT_CNT         ((u8)5)
#define RC_DUTY_CONVERSION_ROUND          ((u16)100)     /* dutyの端点丸め数値 */
#define RC_DUTY_0P_CNT                    ((u16)2000)    /* duty0%相当のカウント現在値 */
#define RC_DUTY_100P_CNT                  ((u16)4000)    /* duty100%相当のカウント現在値　 */
#define RC_DUTY_100P_CNT_DIFF             ( RC_DUTY_100P_CNT - RC_DUTY_0P_CNT )     /* duty100%相当のタイマカウント差分 */

/* duty増減方向の設定(リモコンの操作反転したい場合に使う) */
#define RC_DUTY_PROP                       ((u8)0)       /* 比例 */
#define RC_DUTY_REVERSE_PROP               ((u8)1)       /* 反比例 */


u8 u8_rc_s_debug_flag;

u8 u8_rc_g_duty_judge_sequence;                    /* パルス幅測定シーケンス */

u16 u16_rc_g_duty_speed;                             /* 前進/後進 速度入力 */
u16 u16_rc_g_duty_shift_mode;                        /* シフトチェンジモード入力 (セミオートマ/オート) */
u16 u16_rc_g_duty_speed_gain;                        /* 加速度合い(エンジン応答)入力 */
u16 u16_rc_g_duty_shift_updown;                      /* シフトアップダウン入力 */
u16 u16_rc_g_duty_rev_limit;                         /* レブリミット閾値入力 */

static u8 u8_rc_s_duty_judge_timeout_flag;         /* パルス幅取得 タイムアウト */
static u8 u8_rc_s_duty_judge_timeout_cnt;          /* パルス幅取得 失敗カウント */
static u8 u8_rc_s_duty_get_complete;               /* 1msタスクでのduty取得完了フラグ */        /* タイミング的にループと割り込みで同時書き込みすることはないはず。 */

static u8 u8_rc_s_duty_cnt_debug;


/* 割り込みで取得する値保存用の構造体 */
typedef struct rc_duty_sample_str
{
    u8 u8_start_capture_complete_flag;    /* 指定のチャネルのタイマカウント取得完了フラグ(ON開始時) */
    u8 u8_end_capture_complete_flag;      /* 指定のチャネルのタイマカウント取得完了フラグ(ON終了時) */
    u16 u16_ch_duty_cnt_start;            /* duty  ON開始時のタイマカウント ( タイマレジスタからの取得値 ) */
    u16 u16_ch_duty_cnt_end;              /* duty OFF開始時のタイマカウント ( タイマレジスタからの取得値 ) */
}rc_duty_sample_data;

static rc_duty_sample_data rc_duty_get_by_int_tbl[ RC_DUTY_CH_IDX_MAX ] =
{
    { CLEAR,    CLEAR,    (u16)0,    (u16)0 },
    { CLEAR,    CLEAR,    (u16)0,    (u16)0 },
    { CLEAR,    CLEAR,    (u16)0,    (u16)0 },
    { CLEAR,    CLEAR,    (u16)0,    (u16)0 },
    { CLEAR,    CLEAR,    (u16)0,    (u16)0 },
};

/* ループタスクで使用するduty比保存テーブル (0~100) ※外部の処理でも使う */
u8 u8_rc_g_ch_duty_tbl[ RC_DUTY_CH_IDX_MAX ] =
{
    (u8)0,
    (u8)0,
    (u8)0,
    (u8)0,
    (u8)0
};

/*===================================================================================================================================*/
/*===================================================================================================================================*/
/* レジスタ設定 */
/* SFRからの読み止しだけは、メモリからの毎回呼び出しを指定する */
typedef struct rc_duty_sample_register_setting
{   /* レジスタ設定 */
    volatile u8 *port_addr;               /* IOCポートレジスタへのポインタ ※指している先をvolatileとして扱う */
    u8 port_bitmask;                      /* IOCポートレジスタのビット指定 */
    volatile u8 *flag_addr;               /* IOCフラグレジスタへのポインタ(アドレス指定しておく) */
    u8 flag_bitmask;                      /* IOCフラグレジスタのビット指定 */
    u8 duty_direction;                    /* プロポのレバーに対するduty増減方向 */
}rc_duty_sample_setting;


/* ここで全チャネルまとめて指定できるようにしておきたい */
static const rc_duty_sample_setting rc_duty_get_register_combi_tbl[ RC_DUTY_CH_IDX_MAX ] =
{ /* IOCポート,  IOCポート-ビット位置,  IOCフラグ,  IOCフラグ-ビット位置,    dutyの増減方向    */
    { &PORTA,    (u8)(1U << 0U),     &IOCAF,      (u8)(1U << 0U),      RC_DUTY_REVERSE_PROP },      /* RC_DUTY_CH_THROTTLE           */
    { &PORTA,    (u8)(1U << 1U),     &IOCAF,      (u8)(1U << 1U),      RC_DUTY_PROP         },      /* RC_DUTY_CH_SHIFT_MODE         */
    { &PORTA,    (u8)(1U << 2U),     &IOCAF,      (u8)(1U << 2U),      RC_DUTY_PROP         },      /* RC_DUTY_CH_RESPONSE           */
    { &PORTA,    (u8)(1U << 3U),     &IOCAF,      (u8)(1U << 3U),      RC_DUTY_PROP         },      /* RC_DUTY_CH_PADDLE_SHIFT_INPUT */
    { &PORTA,    (u8)(1U << 4U),     &IOCAF,      (u8)(1U << 4U),      RC_DUTY_PROP         },      /* RC_DUTY_CH_REV_LIMIT          */
};
/*===================================================================================================================================*/
/*===================================================================================================================================*/


/* 関数プロトタイプ宣言 */
static void func_rc_s_convert_tmrcount_to_duty( void );
static void func_rc_s_debug_pulseout( void );
static void func_rc_s_ioc_flag_erase( u8 u8_ioc_ch );
static void func_rc_s_ioc_state_read_back( u8 *u8_flag_buff, u8 *u8_port_buff ,u8 u8_ioc_ch );


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
    u8_rc_g_duty_judge_sequence = RC_SEQ_DUTY_SAMPLING_WAIT;
    
    u16_rc_g_duty_speed          = (u16)0;
    u16_rc_g_duty_shift_mode     = (u16)0;
    u16_rc_g_duty_speed_gain     = (u16)0;
    u16_rc_g_duty_shift_updown   = (u16)0;
    u16_rc_g_duty_rev_limit      = (u16)0;
    
    u8_rc_s_debug_flag = CLEAR;
    u8_rc_s_duty_get_complete = CLEAR;
    u8_rc_s_duty_cnt_debug = (u8)0;
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
    func_rc_s_convert_tmrcount_to_duty();       /* タイマカウントからdutyへの変換 */
    func_rc_s_debug_pulseout();                 /* パルス幅取得の動作確認処理 */

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
    u8 u8_ch_sample_not_complete_flag;
    
    u8 u8_ioc_flag_buff;
    u8 u8_ioc_port_buff;
    
    /* ローカル変数初期化 */
    u8_ioc_flag_buff = (u8)0;
    u8_ioc_port_buff = (u8)0;
    u8_loopcnt = (u8)0;
    u16_timer_register_buff = (u16)0;
    u8_ch_sample_not_complete_flag = CLEAR;
    
    
    switch ( u8_rc_g_duty_judge_sequence )
    {
        case RC_SEQ_DUTY_SAMPLING_WAIT:
            /* 初回のトリガ操作を待機中・・・ */
            /* いずれかのチャネルでIOC割り込みが発生した場合にタイマスタート */
            /* 最初のチャネルだけdutyが小さくならないよう、タイミング優先でまずはタイマスタート */
            func_mset_g_timer5_couter_set( (u16)1 );            /* タイマ レジスタクリア ※オーバーフロー割り込み仕様するので、リセット時に割り込みはいらないよう１をセットする */
            func_mset_g_timer5_onoff( ON );                     /* タイマ カウントスタート */

#if 0
            for( u8_loopcnt = (u8)0; u8_loopcnt < RC_DUTY_CH_IDX_MAX ; u8_loopcnt++ )
            {
                func_rc_s_ioc_state_read_back( &u8_ioc_flag_buff, &u8_ioc_port_buff ,u8_loopcnt );

                if( ( u8_ioc_flag_buff == SET ) &&
                    ( u8_ioc_port_buff == RC_DUTY_PULSE_ON_LOGIC ) )
                { /* いずれかのチャネルで、ON区間開始を検知：連続測定開始 */
                    u8_rc_g_duty_judge_sequence = RC_SEQ_DUTY_SAMPLING;
                    u8_rc_s_duty_get_complete = CLEAR;                          /* メインループ側でのタイマ値取得完了フラグ クリア */
                }
                else
                { /* ON区間終了を検知 */
                    /* 初回がON区間終了から始めるのはおかしいので、タイマ停止。シーケンスそのまま */
                    func_mset_g_timer5_couter_set( (u16)1 );            /* タイマ レジスタクリア ※オーバーフロー割り込み仕様するので、リセット時に割り込みはいらないよう１をセットする */
                    func_mset_g_timer5_onoff( OFF );                     /* タイマ カウント停止 */
                    rc_duty_get_by_int_tbl[u8_loopcnt].u16_ch_duty_cnt_start = (u16)0;
                    rc_duty_get_by_int_tbl[u8_loopcnt].u16_ch_duty_cnt_end = (u16)0;
                    rc_duty_get_by_int_tbl[u8_loopcnt].u8_start_capture_complete_flag = CLEAR;
                    rc_duty_get_by_int_tbl[u8_loopcnt].u8_end_capture_complete_flag = CLEAR;
                }

                func_rc_s_ioc_flag_erase( u8_loopcnt );             /* IOCフラグクリア */
            }
#else
                /* 最後のチャネルのサンプル終了後、次にCH0のIOC割り込みが入るまで若干猶予がある。 */
                /* このタイミングでループタスクで値回収したいので、開始はCH0に固定する */
                u8_loopcnt = RC_DUTY_CH_THROTTLE;
                func_rc_s_ioc_state_read_back( &u8_ioc_flag_buff, &u8_ioc_port_buff ,u8_loopcnt );

                if( ( u8_ioc_flag_buff == SET ) &&
                    ( u8_ioc_port_buff == RC_DUTY_PULSE_ON_LOGIC ) )
                { /* いずれかのチャネルで、ON区間開始を検知：連続測定開始 */
                    u8_rc_g_duty_judge_sequence = RC_SEQ_DUTY_SAMPLING;
                    u8_rc_s_duty_get_complete = CLEAR;                          /* メインループ側でのタイマ値取得完了フラグ クリア */
                }
                else
                { /* ON区間終了を検知 */
                    /* 初回がON区間終了から始めるのはおかしいので、タイマ停止。シーケンスそのまま */
                    func_mset_g_timer5_onoff( OFF );                     /* タイマ カウント停止 */
                    func_mset_g_timer5_couter_set( (u16)1 );            /* タイマ レジスタクリア ※オーバーフロー割り込み仕様するので、リセット時に割り込みはいらないよう１をセットする */
                    rc_duty_get_by_int_tbl[u8_loopcnt].u16_ch_duty_cnt_start = (u16)0;
                    rc_duty_get_by_int_tbl[u8_loopcnt].u16_ch_duty_cnt_end = (u16)0;
                    rc_duty_get_by_int_tbl[u8_loopcnt].u8_start_capture_complete_flag = CLEAR;
                    rc_duty_get_by_int_tbl[u8_loopcnt].u8_end_capture_complete_flag = CLEAR;
                }

                for( u8_loopcnt = (u8)0; u8_loopcnt < RC_DUTY_CH_IDX_MAX ; u8_loopcnt++ )
                { /* フラグは全部クリアする */
                    func_rc_s_ioc_flag_erase( u8_loopcnt );             /* IOCフラグクリア */
                }          
            
#endif

            break;
            
        case RC_SEQ_DUTY_SAMPLING:
            u16_timer_register_buff = func_mset_g_timer5_read();        /* 割り込みが来た時点でいったんタイマレジスタの値を保持しておく */
            
            /* 順番にチャネル判定 */
#if 0
            /* 1チャンネルでの動作確認用 */
            func_rc_s_ioc_state_read_back( &u8_ioc_flag_buff, &u8_ioc_port_buff ,RC_DUTY_CH_REV_LIMIT );

            if( u8_ioc_flag_buff == SET )
            {
                if( u8_rc_s_debug_flag == SET )
                {
                    u8_rc_s_debug_flag = CLEAR;
                }
                else
                {
                    u8_rc_s_debug_flag = SET;
                }
                
                GPIO_OUT_DEBUG = u8_rc_s_debug_flag;    
            }


            for( u8_loopcnt = (u8)0; u8_loopcnt < RC_DUTY_CH_IDX_MAX ; u8_loopcnt++ )
            { /* フラグは全部クリアする */
                func_rc_s_ioc_flag_erase( u8_loopcnt );             /* IOCフラグクリア */
            }  

#else
            /* IOC割り込みフラグが立ってるポートを確認 */
            for( u8_loopcnt = (u8)0; u8_loopcnt < RC_DUTY_CH_IDX_MAX ; u8_loopcnt++ )
            {
                func_rc_s_ioc_state_read_back( &u8_ioc_flag_buff, &u8_ioc_port_buff ,u8_loopcnt );
            
                if( u8_ioc_flag_buff == SET )
                { /* IOC割り込み発生あり */
                    if( u8_ioc_port_buff != RC_DUTY_PULSE_ON_LOGIC )
                    { /* ON区間終わりの時 */
                        rc_duty_get_by_int_tbl[u8_loopcnt].u16_ch_duty_cnt_end = u16_timer_register_buff;        /* IOC割り込み発生時点でのタイマ値を取得 ※PWM仕様上、複数チャネルが同時に切り替わることはないはず。 */
                        rc_duty_get_by_int_tbl[u8_loopcnt].u8_end_capture_complete_flag = SET;                   /* 対象のチャネルのパルス幅測定 完了 */
                    }
                    else
                    { /* ON区間始まりの時：開始キャプチャとして保存する ※基本連続しているので、1つ後のチャネルでここに来る */
                        rc_duty_get_by_int_tbl[u8_loopcnt].u16_ch_duty_cnt_start = u16_timer_register_buff;      /* IOC割り込み発生時点でのタイマ値を取得 ※PWM仕様上、複数チャネルが同時に切り替わることはないはず。 */
                        rc_duty_get_by_int_tbl[u8_loopcnt].u8_start_capture_complete_flag = SET;                 /* 対象のチャネルのパルス幅測定 完了 */
                    }

                    func_rc_s_ioc_flag_erase( u8_loopcnt );             /* IOCフラグクリア */
                }

                if( rc_duty_get_by_int_tbl[u8_loopcnt].u8_end_capture_complete_flag == CLEAR )
                { /* まだON区間測定が終了してないチャネルがある */
                    u8_ch_sample_not_complete_flag = SET;
                }
            }

            
            /* 全チャネル パルス幅測定 完了判定 */
            if( u8_ch_sample_not_complete_flag != SET )
            { /* 全チャネルサンプリング完了 */
                u8_rc_g_duty_judge_sequence = RC_SEQ_DUTY_SAMPLING_WAIT;            /* 待機状態へ戻す　※待機状態中にループタスクで値回収 */
                func_mset_g_timer5_onoff( OFF );
                func_mset_g_timer5_couter_set( (u16)1 );

                for( u8_loopcnt = (u8)0; u8_loopcnt < RC_DUTY_CH_IDX_MAX ; u8_loopcnt++ )
                { /* ループが2回連続で走るが、すでに全CHサンプル完了後なのでOK */
                    rc_duty_get_by_int_tbl[u8_loopcnt].u8_start_capture_complete_flag = CLEAR;
                    rc_duty_get_by_int_tbl[u8_loopcnt].u8_end_capture_complete_flag = CLEAR;
                }
            }
#endif

            break;
            
        case RC_SEQ_DUTY_SAMPLING_TIMEOUT:
            /* タイムアウト発生 */
            /* CH0のIOC割り込み発生後、8CHすべて取得できる時間以上経過したのにパルスが来なかった後、IOC割り込み発生するとここに来る */
            /* パルス幅の取得タイミングがずれたときにCH0開始に戻すために設けた */
            /* 完全にパルスが止まった場合はIOC割り込みがなく、ここに来ることができないので、ループタスク側でもタイムアウト処理を作ったほうが良いかも */
            if( u8_rc_s_duty_judge_timeout_cnt < U8_MAX )
            {
                u8_rc_s_duty_judge_timeout_cnt++;
            }
            
            if( u8_rc_s_duty_judge_timeout_cnt > RC_DUTY_JUDGE_TIMEOUT_CNT )
            { /* 何度もタイムアウトが発生している */
                u8_rc_g_duty_judge_sequence = RC_SEQ_DUTY_SAMPLING_FAILE;
            }
            else
            {
                u8_rc_g_duty_judge_sequence = RC_SEQ_DUTY_SAMPLING_WAIT;
            }

            func_mset_g_timer5_onoff( OFF );                    /* タイマ カウント停止 */
            func_mset_g_timer5_couter_set( (u16)1 );            /* タイマ レジスタクリア ※オーバーフロー割り込み仕様するので、リセット時に割り込みはいらないよう１をセットする */
            
            for( u8_loopcnt = (u8)0; u8_loopcnt < RC_DUTY_CH_IDX_MAX ; u8_loopcnt++ )
            { /* フラグは全部クリアする */
                func_rc_s_ioc_flag_erase( u8_loopcnt );             /* IOCフラグクリア */
            }
        
            break;
            
        case RC_SEQ_DUTY_SAMPLING_FAILE:
            /* パルス幅測定失敗 */
            /* ここに来た時点で設計ミスなので、デバッグ時点ではあえて抜けるないようにしておく。 */
            for( u8_loopcnt = (u8)0; u8_loopcnt < RC_DUTY_CH_IDX_MAX ; u8_loopcnt++ )
            {
                // *(rc_duty_get_register_combi_tbl[u8_loopcnt].port_addr) != (u8)0;                     /* 読み取り専用なので特に意味なし */
                func_rc_s_ioc_flag_erase( u8_loopcnt );                                                  /* IOCフラグクリア */
                rc_duty_get_by_int_tbl[u8_loopcnt].u16_ch_duty_cnt_start = (u16)0;
                rc_duty_get_by_int_tbl[u8_loopcnt].u16_ch_duty_cnt_end = (u16)0;
                rc_duty_get_by_int_tbl[u8_loopcnt].u8_start_capture_complete_flag = CLEAR;
                rc_duty_get_by_int_tbl[u8_loopcnt].u8_end_capture_complete_flag = CLEAR;
            }
              
            u8_rc_s_duty_judge_timeout_cnt = (u8)0;
            
            /* @@何度もここに来るので、ここに関数記述するのは微妙かもしれない。実害ないのでいったん保留。 */
            func_mset_g_timer5_onoff( OFF );                    /* タイマ カウント停止 */
            func_mset_g_timer5_couter_set( (u16)1 );            /* タイマ レジスタクリア ※オーバーフロー割り込み仕様するので、リセット時に割り込みはいらないよう１をセットする */
        
            break;
        
        default:
            break;
    }
}

/**************************************************************/
/*  Function:                                                 */
/*  入力パルスのduty識別・決定関数                              */
/*  IOC割り込みがあったのにパルス幅取得を完了できなかった場合の処理 */
/**************************************************************/
void func_rc_g_duty_detect_timeout( void )
{
    //u8_rc_s_duty_judge_timeout_flag = SET;
}


/**************************************************************/
/*  Function:                                                 */
/* *u8_flagbuff：引数/変数 IOCフラグ状態                          */
/* *u8_portbuff：引数/変数 IOC割り当てのポート状態                */
/* u8_ioc_ch：引数 IOCのチャネル指定                             */
/**************************************************************/
static void func_rc_s_ioc_state_read_back( u8 *u8_flag_buff, u8 *u8_port_buff ,u8 u8_ioc_ch )
{
    u8 u8_iocxf_flag;
    u8 u8_iocxf_port_state;

    u8_iocxf_flag = *(rc_duty_get_register_combi_tbl[ u8_ioc_ch ].flag_addr);                             /* IOCフラグレジスタの値を取得 */
    u8_iocxf_flag = u8_iocxf_flag & (rc_duty_get_register_combi_tbl[ u8_ioc_ch ].flag_bitmask);        /* ビットマスク */
    
    if( u8_iocxf_flag != (u8)0 )
    { /* IOCフラグレジスタの該当ビットが0ではない */
        *u8_flag_buff = SET;         /* のちの処理でビット位置気にしないよう、この時点で上書きする */
    }
    else
    {
        *u8_flag_buff = CLEAR;
    }
    
    /* IOCポートの確認 */
    u8_iocxf_port_state = *(rc_duty_get_register_combi_tbl[ u8_ioc_ch ].port_addr);                       /* IOCに該当のポート状態を取得 */
    u8_iocxf_port_state = u8_iocxf_port_state & (rc_duty_get_register_combi_tbl[ u8_ioc_ch ].port_bitmask);     /* ビットマスク */
    
    if( u8_iocxf_port_state != (u8)0 )
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
    
    *( rc_duty_get_register_combi_tbl[ u8_ioc_ch ].flag_addr ) &= u8_ioc_flag_buff;
}



/**************************************************************/
/*  Function:                                                 */
/*  入力パルス幅の一時保存値を回収                              */
/*  タイマの計算値を0~100の1byte内のdutyへ変換する              */
/*  インバータ電圧、角度ともに100段程度分解能があれば困らないはず  */
/*  @@除算が激重でこの処理が一番処理負荷高いので、処理時間問題ないかチェックする */
/**************************************************************/
static void func_rc_s_convert_tmrcount_to_duty( void )
{
    u8 u8_loopcnt;
    u8 u8_duty_calc_disable_flag;
    u16 u16_pulse_on_time_cnt_buff;
    u16 u16_calc_buff;
    u16 u16_calc_buff2;
    u32 u32_calc_buff;
    u16 u16_tmr_cnt_base;
    u16 u16_tmr_cnt_gap;
    

    /* ローカル変数初期化 */
    u8_loopcnt = (u8)0;
    u16_pulse_on_time_cnt_buff = (u16)0;
    u16_calc_buff = (u16)0;
    u16_calc_buff2 = (u16)0;
    u8_duty_calc_disable_flag = CLEAR;
    
    /* グローバル変数へ移植 */
    /* 割り込み更新がないタイミングでデータを取り出す */
    if( ( u8_rc_s_duty_get_complete == CLEAR ) &&
        ( u8_rc_g_duty_judge_sequence == RC_SEQ_DUTY_SAMPLING_WAIT ) )
    { /* 次のパルス開始タイミングまで待機中 (duty更新なし中) */
        /* 基準カウント数の算出 */
        u16_tmr_cnt_base = RC_DUTY_100P_CNT - RC_DUTY_0P_CNT;

        for( u8_loopcnt = (u8)0; u8_loopcnt < RC_DUTY_CH_IDX_MAX; u8_loopcnt++ )
        {
            u16_calc_buff = rc_duty_get_by_int_tbl[ u8_loopcnt ].u16_ch_duty_cnt_start;         /* パルスON開始時のタイマ・カウント */
            u16_calc_buff2 = rc_duty_get_by_int_tbl[ u8_loopcnt ].u16_ch_duty_cnt_end;          /* パルスON終了時のタイマ・カウント */

            if( u16_calc_buff < u16_calc_buff2 )
            { /* 正常：後ろのタイミングのタイマカウントのほうが大きい */
                u16_calc_buff = u16_calc_buff2 - u16_calc_buff;

                /* 上下限クリップ処理 */
                if( u16_calc_buff >= RC_DUTY_0P_CNT )
                {
                    u16_tmr_cnt_gap = u16_calc_buff - RC_DUTY_0P_CNT;

                    if( u16_tmr_cnt_gap < RC_DUTY_CONVERSION_ROUND)
                    { /* ほぼ0% */
                        u16_tmr_cnt_gap = (u16)0;                       /* 0%に丸め込む */
                    }

                    if( u16_tmr_cnt_gap > u16_tmr_cnt_base )
                    { /* 100%より大きい */
                        u16_tmr_cnt_gap = u16_tmr_cnt_base;
                    }
                    else
                    {
                        u16_calc_buff = u16_tmr_cnt_base - u16_tmr_cnt_gap;
                        if( u16_calc_buff < RC_DUTY_CONVERSION_ROUND )
                        { /* ほぼ100%のパルス幅になっている */
                            u16_tmr_cnt_gap = u16_tmr_cnt_base;         /* 100%に丸め込む */
                        }
                    }
                }
                else
                {
                    u16_tmr_cnt_gap = (u16)0;
                }
            }
            else
            { /* ラジコンのPWMの性質上、パルス幅がゼロになることはない */
                u8_duty_calc_disable_flag = SET;            /* 異常時：計算しない*/
            }


           
            if( u8_duty_calc_disable_flag == CLEAR )
            { /* duty計算許可がある */
                if( rc_duty_get_register_combi_tbl[ u8_loopcnt ].duty_direction == RC_DUTY_REVERSE_PROP )
                { /* duty増減方向を反転させる場合 */
                    u16_tmr_cnt_gap = u16_tmr_cnt_base - u16_tmr_cnt_gap;
                }
#if 1
                /* タイマ値 -> %変換 */
                u32_calc_buff = func_ud_g_calcmul_2x2_byte( u16_tmr_cnt_gap, (u16)RC_CH_DUTY_100P );
                u32_calc_buff = func_ud_g_calcdiv_4x4_byte( u32_calc_buff, (u32)u16_tmr_cnt_base );
                u8_rc_g_ch_duty_tbl[ u8_loopcnt ] = (u8)u32_calc_buff;
#else
                /* タイマ値 -> %変換 */
                /* こっちのほうが重い */
                u32_calc_buff = func_ud_g_calcdiv_4x4_byte( RC_DUTY_100P_CNT, RC_CH_DUTY_100P );
                u32_calc_buff = func_ud_g_calcdiv_4x4_byte( (u32)u16_pulse_on_time_cnt_buff, u32_calc_buff );
                u8_rc_g_ch_duty_tbl[ u8_loopcnt ] = (u8)u32_calc_buff;
#endif
            }
        }
        u8_rc_s_duty_get_complete = SET;
    }
}

/**************************************************************/
/*  Function:                                                 */
/*  パルス幅取得の動作確認                                      */
/**************************************************************/
static void func_rc_s_debug_pulseout( void )
{
#if 1
    if( u8_rc_s_duty_cnt_debug < U8_MAX )
    {
        u8_rc_s_duty_cnt_debug++;
    }

    if( u8_rc_s_duty_cnt_debug >= u8_rc_g_ch_duty_tbl[ RC_DUTY_CH_THROTTLE ] )
    {
        u8_rc_s_duty_cnt_debug = (u8)0;

        if( GPIO_OUT_DEBUG == CLEAR )
        {
            GPIO_OUT_DEBUG = SET;
        }
        else
        {
            GPIO_OUT_DEBUG = CLEAR;
        }
    }
#endif
}



