/*
 * File:   speedsens.c
 * Author: ICE_MEGANE
 *
 * Created on 2026/4/2, 22:26
 */

#include <xc.h>
#include "../userdefine.h"

#include "../mcufunc/gpio.h"
#include "../mcufunc/mcu_setup.h"
#include "../mcufunc/timer_driver.h"
#include "speedsens.h"
#include "../mcufunc/gpio.h"
#include "../mcufunc/dac.h"


/* パラメータ */
#define SPEEDSENS_CAPTURE_MTR_NUM         ((u8)4)                     /* 2,4,8,16 より選択可能。　※2のn倍のみ設定可能：モータ回転数のキャプチャ保存数 */
#define SPEEDSENS_CAPTURE_1STGEAR_NUM     ((u8)4)                     /* 2,4,8,16 より選択可能。　※2のn倍のみ設定可能：1次側ギヤ回転数のキャプチャ保存数 */

#define SPEEDSENS_LOWSPEED_JUDGE_CNT_NUM  ((u8)5)
#define SPEEDSENS_LOWSPEED_JUDGE_RPM_NUM  ((u8)6)


typedef struct speed_status_def_by_mainloop
{
    u16 u16_capture_cnt_save;    /* 前回保存したキャプチャカウント */
    u16 u16_capture_cnt_now;        /* 現在のキャプチャカウント */
    u16 u16_speed;                  /* 回転数の現在値 */        /* 平均はいらない気がする もともとごく低速では平均取れないはず */
}ts_speed_status_by_mainloop;


/*  */
u16 u16_speedsens_g_rpm_ary[ SPEEDSENS_CH_NUM ];
ts_speed_status_by_capture speedsens_status[ SPEEDSENS_CH_NUM ];                    /* キャプチャによる回転数計算 ステータス */
static ts_speed_status_by_mainloop low_speedsens_status[ SPEEDSENS_CH_NUM ];        /* ループタスクによる回転数計算 ステータス */

static u16 u16_speedsens_s_capture_ary_mtr[ SPEEDSENS_CAPTURE_MTR_NUM ];
static u16 u16_speedsens_s_capture_ary_1stgear[ SPEEDSENS_CAPTURE_1STGEAR_NUM ];
static const u8 u8_speedsens_s_capture_buff_num_ary[ SPEEDSENS_CH_NUM ] =
{
    SPEEDSENS_CAPTURE_MTR_NUM,
    SPEEDSENS_CAPTURE_1STGEAR_NUM
};

/* @@注意：u16のバッファに2byte単位で保存するので、ポインタ指定するときインデックス操作で2byteずつ進んでいくよう、 */
/* 2byte管理のポインタ配列として定義する */
static const u16 *u16_speedsens_s_capture_ary_pointer[ SPEEDSENS_CH_NUM ] =
{
#if 0
    &u16_speedsens_s_capture_ary_mtr[(u8)0],
    &u16_speedsens_s_capture_ary_1stgear[(u8)0]
#else
    /* こっちの書き方のほうがシンプル */
    u16_speedsens_s_capture_ary_mtr,
    u16_speedsens_s_capture_ary_1stgear
#endif
};


/* 低速検出関連 */
static u16 u16_speedsens_s_lowrpm_ary[ SPEEDSENS_CH_NUM ];
static u16 u16_speedsens_s_lowrpm_calc_cnt[ SPEEDSENS_CH_NUM ];

static const u16 u16_speedsens_low_speed_judge_cnt_ary[ SPEEDSENS_LOWSPEED_JUDGE_CNT_NUM ] =
{ /* カウント数,   回転数 */
/* 0 */     (u16)120,     /* 範囲外の定義 */
/* 1 */     (u16)150,     /* 高速側 */
/* 2 */     (u16)200,
/* 3 */     (u16)300,
/* 4 */     (u16)600      /* 低速側 */
};

static const u16 u16_speedsens_low_speed_judge_rpm_ary[ SPEEDSENS_LOWSPEED_JUDGE_RPM_NUM ] =
{
/* 0 */     (u16)U16_MAX,     /* 実際は500rpmだが、範囲外の判定のために意図的にでかくしておく */
/* 1 */     (u16)400,
/* 2 */     (u16)300,
/* 3 */     (u16)200,
/* 4 */     (u16)100,
/* 5 */     (u16)0
};


/* 関数プロトタイプ宣言 */
static void func_speedsens_s_capture_ave_update( ts_speed_status_by_capture *sts );
static u16 func_speedsens_s_calc_speed( ts_speed_status_by_capture *sts );
static u16 func_speedsens_s_calc_low_speed( u8 u8_speedsens_ch, gpio_in gpio_in_reset_source );
static void func_speedsens_s_speed_update( void );


/**************************************************************/
/*  Function:                                                 */
/*  ループタスク                                               */
/**************************************************************/
void func_speedsens_g_main( void )
{
    func_speedsens_s_speed_update();
}



/**************************************************************/
/*  Function:                                                 */
/*  初期化処理                                                 */
/**************************************************************/
void func_speedsens_g_init( void )
{
    u8 u8_loopcnt;

    for( u8_loopcnt = (u8)0; u8_loopcnt < SPEEDSENS_CH_NUM; u8_loopcnt++ )
    {
        /* 回転数検出 ステータス初期化 */
        speedsens_status[ u8_loopcnt ].u8_buff_num = u8_speedsens_s_capture_buff_num_ary[ u8_loopcnt ];               /* 固定値 バッファ数指定 */
        speedsens_status[ u8_loopcnt ].u16_p_capture_buff = u16_speedsens_s_capture_ary_pointer[ u8_loopcnt ];        /* 固定値 バッファへのポインタ指定 */
        func_speedsens_g_reset_capture_sts( &speedsens_status[ u8_loopcnt ] );

        /* 回転数検出 回転数保存バッファ 初期化 */
        u16_speedsens_g_rpm_ary[ u8_loopcnt ]         = (u16)0;           /* 回転数計算用バッファ：初期化 */
        u16_speedsens_s_lowrpm_ary[ u8_loopcnt ]      = (u16)0;
        u16_speedsens_s_lowrpm_calc_cnt[ u8_loopcnt ] = (u16)0;

        /* ループタスク回転数検出 初期化 */
        low_speedsens_status[ u8_loopcnt ].u16_capture_cnt_now = U16_MAX;
        low_speedsens_status[ u8_loopcnt ].u16_capture_cnt_save = U16_MAX;
        low_speedsens_status[ u8_loopcnt ].u16_speed       = (u16)0;
    }
}


/**************************************************************/
/*  Function:                                                 */
/*  キャプチャ値を回収する                                      */
/**************************************************************/
void func_speedsens_g_collect_capture( u16 u16_capture, ts_speed_status_by_capture *sts )
{
    /* キャプチャ保存処理 */
    if( sts->u8_cap_timer_reload == SET )
    { /* ゲート クローズ時点でタイマのリロードが発生していた */
        /* 値が信用できないので低速側で上書きする */
        sts->u16_p_capture_buff[ (sts->u8_buff_idx) ] = U16_MAX;            /* 最大値を設定 */
        sts->u8_cap_timer_reload = CLEAR;                                   /* リロードフラグ クリア */
    }
    else
    { /* リロードなし：適切な回転数帯 */
        sts->u16_p_capture_buff[ (sts->u8_buff_idx) ] = u16_capture;        /* タイマの値をそのまま取得する */
    }

    
    if( sts->u8_buff_idx < sts->u8_buff_num )
    {
        (sts->u8_buff_idx)++;
        
        if( (sts->u8_buff_idx) ==  (sts->u8_buff_num) )
        {
            sts->u8_buff_idx = (u8)0;
            sts->u8_buffer_filled = SET;       /* キャプチャバッファが全て埋まった */
        }
    }

    /* 平均化処理 */
    /* 割り込み処理としては若干負荷が高いが、ビットシフト演算で除算しているので問題ない・・・はず。 */
    /* バッファが更新される前に平均化を行ったほうが、正確な回転数平均が出せると思われるため、ここに実装 */
    func_speedsens_s_capture_ave_update( sts );
}

/**************************************************************/
/*  Function:                                                 */
/*  キャプチャ値回収処理の関連変数をリセットする                  */
/*  基本的にリセットは低速側に寄せる                             */
/**************************************************************/
void func_speedsens_g_reset_capture_sts( ts_speed_status_by_capture *sts )
{
    u8 u8_buff_cnt;
    u8 u8_buff_num;

    sts->u8_buffer_filled      = CLEAR;     /* バッファ埋め：未完了 */
    sts->u8_cap_timer_reload   = CLEAR;     /* タイマリロード：発生していない */    /* ※※このフラグはOVF発生後再度セットされる!※※ */
    sts->u8_buff_idx = (u8)0;               /* キャプチャバッファ：0コ目にリセット */
    sts->u16_capture_ave = U16_MAX;         /* キャプチャ平均：最低速側に設定 */
    sts->u16_speed_ave = (u16)0;            /* 回転数：最低速側に設定 */


    u8_buff_num = sts->u8_buff_num; 
    for( u8_buff_cnt = (u8)0; u8_buff_cnt < u8_buff_num; u8_buff_cnt++ )
    { /* バッファの数に応じて順次初期化する */
        sts->u16_p_capture_buff[u8_buff_cnt] = (u16)U16_MAX;      /* 低速側で初期化 */
    }
}

/**************************************************************/
/*  Function:                                                 */
/*  キャプチャ平均処理                                          */
/* 一応ループタスク側で平均化する */
/**************************************************************/
static void func_speedsens_s_capture_ave_update( ts_speed_status_by_capture *sts )
{
    u8 u8_loopcnt;
    u32 u32_capture_sum;

    u8_loopcnt = (u8)0;
    u32_capture_sum = (u32)0;

    sts->u16_capture_ave = (u16)0;

    for( u8_loopcnt = (u8)0; u8_loopcnt < (sts->u8_buff_num); u8_loopcnt++ )
    { /* バッファの合計値をすべて合算 */
        u32_capture_sum += (u32)(sts->u16_p_capture_buff[ u8_loopcnt ]);
    }    
    
    if( sts->u8_buffer_filled == SET )
    {
        if( sts->u8_buff_num == (u8)2 )
        {
            sts->u16_capture_ave = (u16)( u32_capture_sum >> 1U );
        }
        else if( sts->u8_buff_num == (u8)4 )
        {
            sts->u16_capture_ave = (u16)( u32_capture_sum >> 2U );
        }
        else if( sts->u8_buff_num == (u8)8 )
        {
            sts->u16_capture_ave = (u16)( u32_capture_sum >> 3U );
        }
        else if( sts->u8_buff_num == (u8)16 )
        {
            sts->u16_capture_ave = (u16)( u32_capture_sum >> 4U );
        }
        else
        { /* これ以外の場合は設定ミスとする。ソフトとしてはとりあえず動くようにしておく。 */
            sts->u16_capture_ave = sts->u16_p_capture_buff[ u8_loopcnt ];
        }
    }
    else
    { /* バッファ埋まるまでは最低速度としておく */
        sts->u16_capture_ave = U16_MAX;
    }
}

/**************************************************************/
/*  Function:                                                 */
/* 回転数更新処理                                              */
/**************************************************************/
static void func_speedsens_s_speed_update( void )
{  
    u8 u8_loopcnt;
    u16 u16_speed_by_capture[ SPEEDSENS_CH_NUM ];           /* ループで処理したかったが、低速側のパルスカウントのリセットソースをポインタ指定するのが面倒なので、あとまわし */
    u16 u16_speed_by_mainloop[ SPEEDSENS_CH_NUM ];

    /*========*/
    /* 初期化 */
    /*========*/
    u8_loopcnt = (u8)0;
    for( u8_loopcnt = (u8)0; u8_loopcnt < SPEEDSENS_CH_NUM; u8_loopcnt++ )
    {
        u16_speed_by_capture[ u8_loopcnt ] = (u16)0;
        u16_speed_by_mainloop[ u8_loopcnt ] = (u16)0;
    }

    /*===================*/
    /* モータ側回転数計算 */
    /*===================*/
    u16_speed_by_capture[ SPEEDSENS_CH_MTR ]      = func_speedsens_s_calc_speed( &speedsens_status[ SPEEDSENS_CH_MTR ] );
    u16_speed_by_mainloop[ SPEEDSENS_CH_MTR ]     = func_speedsens_s_calc_low_speed( SPEEDSENS_CH_MTR, gpio_g_speed_mtr_sw );
    if( u16_speed_by_capture[ SPEEDSENS_CH_MTR ] < u16_speed_by_mainloop[ SPEEDSENS_CH_MTR ] )
    { /* 高速側の回転数の方が確度がある */
        u16_speedsens_g_rpm_ary[ SPEEDSENS_CH_MTR ] = u16_speed_by_capture[ SPEEDSENS_CH_MTR ];
    }
    else
    { /* 低速側の回転数の方が確度がある */
        u16_speedsens_g_rpm_ary[ SPEEDSENS_CH_MTR ] = u16_speed_by_mainloop[ SPEEDSENS_CH_MTR ];
    }

    /*===================*/
    /* 1次側ギヤ回転数計算 */
    /*===================*/
    u16_speed_by_capture[ SPEEDSENS_CH_1STGEAR ]  = func_speedsens_s_calc_speed( &speedsens_status[ SPEEDSENS_CH_1STGEAR ] );
    u16_speed_by_mainloop[ SPEEDSENS_CH_1STGEAR ] = func_speedsens_s_calc_low_speed( SPEEDSENS_CH_1STGEAR, gpio_g_speed_1stgear_sw );
    if( u16_speed_by_capture[ SPEEDSENS_CH_1STGEAR ] < u16_speed_by_mainloop[ SPEEDSENS_CH_1STGEAR ] )
    { /* 高速側の回転数の方が確度がある */
        u16_speedsens_g_rpm_ary[ SPEEDSENS_CH_1STGEAR ] = u16_speed_by_capture[ SPEEDSENS_CH_1STGEAR ];
    }
    else
    { /* 低速側の回転数の方が確度がある */
        u16_speedsens_g_rpm_ary[ SPEEDSENS_CH_1STGEAR ] = u16_speed_by_mainloop[ SPEEDSENS_CH_1STGEAR ];
    }

}

/**************************************************************/
/*  Function:                                                 */
/* 回転数計算処理                                              */
/* 取得したキャプチャの平均値から現在の回転数を算出する            */
/**************************************************************/
static u16 func_speedsens_s_calc_speed( ts_speed_status_by_capture *sts )
{
    u16 u16_result;
    u32 u32_speed;

    u16_result = (u16)0;
    u32_speed = (u32)0;
    
    /* 0割防止 */
    if( sts->u16_capture_ave == (u16)0 )
    {
        sts->u16_capture_ave = (u16)1;
    }

    u32_speed = (u32)SPEEDSENS_MAX_SPEED_AT_1_CAPTURE / (u32)(sts->u16_capture_ave);        /* キャプチャ1あたりの回転数 / キャプチャ値：キャプチャがx倍になれば、回転数は1/xになる関係 */

    if( u32_speed > U16_MAX )
    { /* 回転数が速すぎる */
        u16_result = U16_MAX;
    }
    else if( u32_speed < (u32)SPEEDSENS_MIN_SPEED_DETECTABLE )
    { /* 検出可能回転数より下は、0rpmにならずに止まってしまうため、0rpmへ上書きする */
        u16_result = (u16)0;
    }
    else
    {
        u16_result = (u16)u32_speed;
    }

    return u16_result;
}

/****************************************************************/
/* Function:                                                    */
/* 低速回転数計算処理                                             */
/* タイマゲートクローズ中に回転数更新がなかった場合の回転数初期化処理 */
/* タイマゲートクローズ中はタイマカウントが進まないため、ループタスク */
/* 側で回転が止まってるかどうか判断する                            */
/* ※低速側の回転数更新も兼ねる                                    */
/* 線形補完までは不要と判断したので、数段のテーブルで実装した。      */
/****************************************************************/
static u16 func_speedsens_s_calc_low_speed( u8 u8_speedsens_ch, gpio_in gpio_in_reset_source )
{
    u8 u8_loopcnt;
    u16 u16_result;

    u8_loopcnt = (u8)0;
    u16_result = (u16)0;

    /*====================*/
    /* 回転数判断用カウント */
    /*====================*/
    if( low_speedsens_status[ u8_speedsens_ch ].u16_capture_cnt_now < U16_MAX )
    {
        low_speedsens_status[ u8_speedsens_ch ].u16_capture_cnt_now++;
    }

    if( ( gpio_in_reset_source.u8_state == SET ) &&             /* フィルタ分遅れるが、全体的に位相ずれするだけなら問題ないはず??? */
        ( gpio_in_reset_source.u8_state_bf != CLEAR ) )
    { /* エッジを検知した */
        low_speedsens_status[ u8_speedsens_ch ].u16_capture_cnt_save = low_speedsens_status[ u8_speedsens_ch ].u16_capture_cnt_now;      /* 現在のキャプチャ値を保存 */
        low_speedsens_status[ u8_speedsens_ch ].u16_capture_cnt_now = (u16)0;       /* キャプチャカウントクリア */
    }
    else
    { /* エッジが来ない */
        if( low_speedsens_status[ u8_speedsens_ch ].u16_capture_cnt_save < low_speedsens_status[ u8_speedsens_ch ].u16_capture_cnt_now )
        { /* 前回保存したカウント数を超えたのにまだエッジが来ない */
          /* ->停止している可能性があるので回転数計算用のキャプチャカウント数を超えた時点で最新値をもとに計算を始める */
            low_speedsens_status[ u8_speedsens_ch ].u16_capture_cnt_save = low_speedsens_status[ u8_speedsens_ch ].u16_capture_cnt_now;
        }
        else
        { /*  */
            ;       /* 前回のキャプチャカウント数より小さい値でエッジが来た(回転が上がった)場合は、上の処理に入る。それまではここでは何もしない。 */
        }
    }

    if( u8_speedsens_ch == SPEEDSENS_CH_1STGEAR )
    {
        func_dac_s_debug_out( (u32)low_speedsens_status[ u8_speedsens_ch ].u16_capture_cnt_save, (u32)600 );
    }
    

    /*===============*/
    /* 低速回転数計算 */
    /*===============*/
    /* 現在の回転数位置の検索 */
    for( u8_loopcnt = (u8)0; u8_loopcnt < SPEEDSENS_LOWSPEED_JUDGE_CNT_NUM; u8_loopcnt++ )
    {
        if( low_speedsens_status[ u8_speedsens_ch ].u16_capture_cnt_save < u16_speedsens_low_speed_judge_cnt_ary[ u8_loopcnt ] )
        { /* 保存されたキャプチャカウント数をもとに計算する(前回の値を利用する) */
            break;      /* 抜けるときはidxは大きいほうで抜ける = 回転数は低い側に丸め込む */
        }
    }

    /* 回転数更新 */
    u16_result = u16_speedsens_low_speed_judge_rpm_ary[ u8_loopcnt ];

    /*===========*/
    /* 返り値更新 */
    /*===========*/
    return u16_result;
}




