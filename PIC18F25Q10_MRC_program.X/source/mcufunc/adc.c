
/*
 * File:   adc.c
 * Author: ICE_MEGANE
 *
 * Created on 2020/03/5, 22:26
 */

#include <xc.h>
#include "../userdefine.h"

#include "adc.h"



#define ADC_CH_SERVO_POSI_ADJ           ADC_AN2


#define ADC_RESULT_AVE_CYCLE            ((u8)4)
#define ADC_RESULT_AVE_SHIFT_BIT        ((u8)2)     /* 下位2bitシフト　= /4になる */

#define ADC_RESULT_ID_AVE_CALC_SUM      ((u8)0)
#define ADC_RESULT_ID_AVE               ((u8)1)
#define ADC_RESULT_ID_NUM               ( ADC_RESULT_ID_AVE + (u8)1 )



/* AD変換チャネル定義 */
#define ADC_AN0     ((u8)0)
#define ADC_AN1     ((u8)1)
#define ADC_AN2     ((u8)2)
#define ADC_AN3     ((u8)3)
#define ADC_AN4     ((u8)4)
#define ADC_AN5     ((u8)5)
#define ADC_AN6     ((u8)6)
#define ADC_AN7     ((u8)7)
#define ADC_AN8     ((u8)8)
#define ADC_AN9     ((u8)9)
#define ADC_AN10    ((u8)10)
#define ADC_AN11    ((u8)11)
#define ADC_CH_NUM  (ADC_AN11+(u8)1)

#define ADC_REQ     ((u8)0)
#define ADC_RESULT  ((u8)1)
#define ADC_DATA_SELECT (ADC_RESULT + (u8)1)


/* 変換時間 */
#define ADC_ACQUISION_TIME      ((u8)20)



 
/* 関数プロトタイプ宣言 */
static void func_adc_s_convert_start(u8 adc_ch);                                /* AD変換 開始処理        */
static void func_adc_s_calc_data_average( u8 adc_ch );      /* AD変換結果 4回平均処理 */
static void func_adc_s_result_assign( void );                                   /* AD変換結果 割り当て処理 */


static u8 u8_adc_s_result_certain_cnt;
u16 u16_adc_g_ad_result_ave____servo_posi_adj;


/* AD変換要求 保持用配列 */
static u8 u8_adc_s_adc_req_status_tbl[ ADC_CH_NUM ] =
{ /* AD変換チャネルが数値と1対1なので、配列の要素指定にそのまま使える */
    (u8)0, (u8)0,  (u8)0,  (u8)0,  (u8)0,  (u8)0,  (u8)0,  (u8)0,  (u8)0,  (u8)0,  (u8)0,  (u8)0
};

/* AD変換結果 保持用配列 */
static u16 u16_adc_s_adc_result_tbl[ ADC_CH_NUM ] =
{
    (u16)0, (u16)0, (u16)0, (u16)0,  (u16)0, (u16)0, (u16)0, (u16)0,  (u16)0, (u16)0, (u16)0, (u16)0
};

static u16 u16_adc_s_adc_result_ave_tbl[ ADC_CH_NUM ][ ADC_RESULT_ID_NUM ] =
{
    {(u16)0, (u16)0 },
    {(u16)0, (u16)0 },
    {(u16)0, (u16)0 },
    {(u16)0, (u16)0 },

    {(u16)0, (u16)0 },
    {(u16)0, (u16)0 },
    {(u16)0, (u16)0 },
    {(u16)0, (u16)0 },
    
    {(u16)0, (u16)0 },
    {(u16)0, (u16)0 },
    {(u16)0, (u16)0 },
    {(u16)0, (u16)0 },
};






/* グローバル関数 */
/**************************************************************/
/*  Function:                                                 */
/*  AD変換実行処理　                                           */
/*                                                            */
/**************************************************************/
void func_adc_g_main( void )
{
    /* AD変換結果取得 */
    func_adc_s_calc_data_average( ADC_CH_SERVO_POSI_ADJ );
    func_adc_s_result_assign();
    
    /* 次のAD変換開始処理 */
    func_adc_s_convert_start( ADC_CH_SERVO_POSI_ADJ );
}


/**************************************************************/
/*  Function:                                                 */
/*  AD変換実行処理　                                           */
/*                                                            */
/**************************************************************/
void func_adc_g_init( void )
{
    u8 u8_loopcnt;
    
    for( u8_loopcnt = (u8)0; u8_loopcnt < ADC_CH_NUM; u8_loopcnt++ )
    {
        u8_adc_s_adc_req_status_tbl[ u8_loopcnt ]  = (u8)0;         /* 初期化 */
        u16_adc_s_adc_result_tbl[ u8_loopcnt ]     = (u16)0;        /* 初期化 */

        u16_adc_s_adc_result_ave_tbl[ u8_loopcnt ][ ADC_RESULT_ID_AVE_CALC_SUM ] = (u16)0;        /* あまり良い書き方ではないが、ついでに初期化... */
        u16_adc_s_adc_result_ave_tbl[ u8_loopcnt ][ ADC_RESULT_ID_AVE ] = (u16)0;                 /* 初期化 */
    }

    u8_adc_s_result_certain_cnt = (u8)0;                            /* 初期化 */
    u16_adc_g_ad_result_ave____servo_posi_adj = (u16)0;             /* 初期化 */
}




/**************************************************************/
/*  Function:                                                 */
/*  AD変換 値取得処理                                          */
/*                                                            */
/**************************************************************/
void func_adc_g_adc_data_get( void )
{
    u8 u8_adc_ch;
    u8 u8_loopcnt;
    u16 u16_adc_result;

    /* 現在のAD変換対象の取得(この処理自体が割り込みで呼ばれる=初回の変換は完了している地点からスタート) */
    u8_adc_ch = ADCON0;
    u8_adc_ch = u8_adc_ch & (u8)0x7C;       /* ビットマスク:0b0111-1100 */
    u8_adc_ch = u8_adc_ch >> 2U;            /* CHSのデータを2bit下位シフトして数値として使えるようにする */

    /* AD変換結果の取得 */
    u16_adc_result = (u16)ADRESH;
    u16_adc_result = u16_adc_result << 8U;              /* 上位データを8ビット上へ移動　※データはLSB詰めで取得している */
    u16_adc_result += (u16)ADRESL;                           /* 下位データはそのまま加算する */

    u16_adc_s_adc_result_tbl[ u8_adc_ch ] = u16_adc_result;    /* AD変換結果取得 */
    u8_adc_s_adc_req_status_tbl[ u8_adc_ch ] = CLEAR;          /* 変換要求クリア */

    /* AD変換未完了CHの検索 */
    for( u8_loopcnt = (u8)0; u8_loopcnt < ADC_CH_NUM; u8_loopcnt++ )
    { /* AD変換要求が連続して発生 = 渋滞している場合への対応 */
        if( u8_adc_s_adc_req_status_tbl[ u8_loopcnt ] == SET )
        { /* AD変換要求があるチャネルを発見 */
            break;
        }
    }

    /* 次のAD変換実行 */
    if( u8_loopcnt < ADC_CH_NUM )
    { /* 少なくとも1chの未完了-変換要求が残っている。 */
        func_adc_s_convert_start( u8_loopcnt );
    }
}


/**************************************************************/
/*  Function:                                                 */
/*  AD変換開始要求処理                                         */
/*                                                            */
/**************************************************************/
static void func_adc_s_convert_start(u8 adc_ch)
{
    u8 u8_adc_wait;
    u8 u8_setting_buff;
    u8 u8_adc_now_process;

    u8_adc_s_adc_req_status_tbl[ adc_ch ] = SET;       /* AD変換要求を格納する */

    u8_adc_now_process = ADCON0;
    
    if( ( u8_adc_now_process & (u8)0x02 ) == (u8)0 )
    { /* 現在実行中のAD変換がない = 最初の1回目の変換待ち状態 */
        u8_setting_buff = (u8)( adc_ch << 2U );     /* CHSは6~2bit目なので、下2bit分シフトする */
        ADCON0 &= ((u8)~0x7C);              /* CHSビットをいったんすべてクリア */
        ADCON0 |= u8_setting_buff;          /* 今回変換したい */

        /* AD変換の対象チャネル切り替え後はアクイジョン・タイムを設ける */
        /* 短すぎるとサンプルホールド用コンデンサの充電が足りず、AD変換結果が期待値より下がるので注意 */
        u8_adc_wait = ADC_ACQUISION_TIME;
        while( u8_adc_wait > (u8)0 )
        {
            u8_adc_wait--;
        }

        ADCON0 |= (u8)0x02;     /* GO_nDONE = SET：ADC開始 (完了後は自動クリアされる) */
    }

    /* ここで変換要求を出して以降、割り込み内でこの関数を再度呼び出し、自動で連続処理する */
}


/**************************************************************/
/*  Function:                                                 */
/*  AD変換結果 平均化処理                                       */
/*                                                            */
/**************************************************************/
static void func_adc_s_calc_data_average( u8 adc_ch )
{
    /* 最新の取得値を加算 */
    u16_adc_s_adc_result_ave_tbl[ adc_ch ][ ADC_RESULT_ID_AVE_CALC_SUM ] += u16_adc_s_adc_result_tbl[ adc_ch ];             /* 平均用合計値保存バッファに、最新値を１コ追加する */


    if( u8_adc_s_result_certain_cnt < U8_MAX )
    {
        u8_adc_s_result_certain_cnt++;
    }

    if( u8_adc_s_result_certain_cnt > ADC_RESULT_AVE_CYCLE )
    {
        /* 平均値を出す */
        if( u16_adc_s_adc_result_ave_tbl[ adc_ch ][ ADC_RESULT_ID_AVE_CALC_SUM ] > u16_adc_s_adc_result_ave_tbl[ adc_ch ][ ADC_RESULT_ID_AVE ] )
        {
            u16_adc_s_adc_result_ave_tbl[ adc_ch ][ ADC_RESULT_ID_AVE_CALC_SUM ] -= u16_adc_s_adc_result_ave_tbl[ adc_ch ][ ADC_RESULT_ID_AVE ];            /* 合計バッファから、前回までの1平均分を引く */
        }
        
        u16_adc_s_adc_result_ave_tbl[ adc_ch ][ ADC_RESULT_ID_AVE ] = u16_adc_s_adc_result_ave_tbl[ adc_ch ][ ADC_RESULT_ID_AVE_CALC_SUM ] >> ADC_RESULT_AVE_SHIFT_BIT;
    }
}



/**************************************************************/
/*  Function:                                                 */
/*  AD変換結果 割り当て処理                                     */
/*                                                            */
/**************************************************************/
static void func_adc_s_result_assign( void )
{
    u16_adc_g_ad_result_ave____servo_posi_adj = u16_adc_s_adc_result_ave_tbl[ ADC_CH_SERVO_POSI_ADJ ][ ADC_RESULT_ID_AVE ];         /* 1変数にして引き渡す */
}













