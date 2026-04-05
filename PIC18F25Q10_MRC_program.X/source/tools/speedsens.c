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


/* パラメータ */
#define SPEEDSENS_CAPTURE_MTR_NUM         ((u8)4)                     /* モータ回転数のキャプチャ保存数 */
#define SPEEDSENS_CAPTURE_1STGEAR_NUM     ((u8)4)                     /* 1次側ギヤ回転数のキャプチャ保存数 */

ts_speed_status speedsens_status[ SPEEDSENS_CH_NUM ];
u16 u16_speedsens_g_speed_ave_mtr;
u16 u16_speedsens_g_speed_ave_1stgear;

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

/* 関数プロトタイプ宣言 */
static void func_speedsens_s_capture_ave_update( ts_speed_status *sts );
static u16 func_speedsens_s_calc_speed( ts_speed_status *sts );


/**************************************************************/
/*  Function:                                                 */
/*  ループタスク                                               */
/**************************************************************/
void func_speedsens_g_main( void )
{
    /* モータ 回転数更新 */
    u16_speedsens_g_speed_ave_mtr = func_speedsens_s_calc_speed( &speedsens_status[ SPEEDSENS_CH_MTR ] );

    /* 1次ギヤ 回転数更新 */
    u16_speedsens_g_speed_ave_1stgear = func_speedsens_s_calc_speed( &speedsens_status[ SPEEDSENS_CH_1STGEAR ] );
}



/**************************************************************/
/*  Function:                                                 */
/*  初期化処理                                                 */
/**************************************************************/
void func_speedsens_g_init( void )
{
    u8 u8_loopcnt;

    /* 回転数検出 ステータス初期化 */
    for( u8_loopcnt = (u8)0; u8_loopcnt < SPEEDSENS_CH_NUM; u8_loopcnt++ )
    {
        speedsens_status[ u8_loopcnt ].u8_buff_num = u8_speedsens_s_capture_buff_num_ary[ u8_loopcnt ];               /* 固定値 バッファ数指定 */
        speedsens_status[ u8_loopcnt ].u16_p_capture_buff = u16_speedsens_s_capture_ary_pointer[ u8_loopcnt ];        /* 固定値 バッファへのポインタ指定 */
        func_speedsens_g_reset_capture_sts( &speedsens_status[ u8_loopcnt ] );
    }

    u16_speedsens_g_speed_ave_mtr = (u16)0;
    u16_speedsens_g_speed_ave_1stgear = (u16)0;
}


/**************************************************************/
/*  Function:                                                 */
/*  キャプチャ値を回収する                                      */
/**************************************************************/
void func_speedsens_g_collect_capture( u16 u16_capture, ts_speed_status *sts )
{
    /* キャプチャ保存処理 */
    sts->u16_p_capture_buff[ (sts->u8_buff_idx) ] = u16_capture;
    
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
/**************************************************************/
void func_speedsens_g_reset_capture_sts( ts_speed_status *sts )
{
    u8 u8_buff_cnt;
    u8 u8_buff_num;

    sts->u8_buffer_filled = CLEAR;
    sts->u8_cap_timer_reload = CLEAR;
    sts->u8_buff_idx = (u8)0;
    sts->u16_capture_ave = (u16)0;
    sts->u16_speed_ave = (u16)0;

    u8_buff_num = sts->u8_buff_num;
    for( u8_buff_cnt = (u8)0; u8_buff_cnt < u8_buff_num; u8_buff_cnt++ )
    {
        sts->u16_p_capture_buff[u8_buff_cnt] = (u16)U16_MAX;      /* 低速側で初期化 */
    }
}

/**************************************************************/
/*  Function:                                                 */
/*  キャプチャ平均処理                                          */
/* 一応ループタスク側で平均化する */
/**************************************************************/
static void func_speedsens_s_capture_ave_update( ts_speed_status *sts )
{
    u8 u8_loopcnt;
    u8 u8_buff_idx_num;
    u32 u32_capture_sum;

    u8_loopcnt = (u8)0;
    u8_buff_idx_num = (u8)0;
    u32_capture_sum = (u32)0;

    sts->u16_capture_ave = (u16)0;

    u8_buff_idx_num = sts->u8_buff_num;

    for( u8_loopcnt = (u8)0; u8_loopcnt < u8_buff_idx_num; u8_loopcnt++ )
    { /* バッファの合計値をすべて合算 */
        u32_capture_sum += (u32)(sts->u16_p_capture_buff[ u8_loopcnt ]);
    }    
    
    sts->u16_capture_ave = (u16)( u32_capture_sum / sts->u8_buff_num );
}

/**************************************************************/
/*  Function:                                                 */
/* 回転数更新処理                                              */
/**************************************************************/
static u16 func_speedsens_s_calc_speed( ts_speed_status *sts )
{
    u16 u16_result;
    u32 u32_speed;

    u16_result = (u16)0;
    u32_speed = (u32)0;
    
    if( ( sts->u8_buffer_filled == SET ) &&
        ( sts->u8_cap_timer_reload == CLEAR ) )
    { /* バッファの更新が発生している & タイマの折り返しがない */
        u32_speed = (u32)SPEEDSENS_MAX_SPEED_AT_1_CAPTURE / (u32)(sts->u16_capture_ave);        /* キャプチャ1あたりの回転数 / キャプチャ値：キャプチャがx倍になれば、回転数は1/xになる関係 */

        if( u32_speed > U16_MAX )
        { /* 回転数が速すぎる */
            u16_result = U16_MAX;
        }
        else
        {
            u16_result = (u16)u32_speed;
        }
    }
    else
    { /* 回転開始直後 or ゲート割り込みが来てない状態が継続中 */
        u16_result = (u16)0;         /* 実質、起動後初期値となる。 とりあえず低速側に設定しておく */
    }

    return u16_result;
}