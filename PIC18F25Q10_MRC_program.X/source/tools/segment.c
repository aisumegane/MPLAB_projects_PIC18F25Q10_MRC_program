#include <xc.h>
#include "../userdefine.h"


#include "../mcufunc/gpio.h"

#include "segment.h"


/* 関数プロトタイプ宣言 */


/**************************************************************/
/*  Function:                                                 */
/*  7セグ制御 初期化処理                                       */
/*                                                            */
/**************************************************************/
void func_segment_g_init( void )
{
    ;
}


/**************************************************************/
/*  Function:                                                 */
/*  tc4511BP ICを使用する場合の出力関数                         */
/*  1桁表示用                                                  */
/**************************************************************/
void func_segment_g_tc4511bp_data_set( u8 disp_num )
{
    /* 最大出力マスク */
    if( disp_num > (u8)9 )
    { /* tc4511BPは最大9　最大値以上はマスクする */
        disp_num = (u8)9;
    }

    /* ==== 2進数でのビット判定 ==== */
    if( ( disp_num & (u8)0x01 ) == (u8)0x01 )
    {
        u8_gpio_g_out_7seg_led_data_a = SET;
    }

    if( ( disp_num & (u8)0x02 ) == (u8)0x02 )
    {
        u8_gpio_g_out_7seg_led_data_b = SET;
    }

    if( ( disp_num & (u8)0x04 ) == (u8)0x04 )
    {
        u8_gpio_g_out_7seg_led_data_c = SET;
    }

    if( ( disp_num & (u8)0x08 ) == (u8)0x08 )
    {
        u8_gpio_g_out_7seg_led_data_d = SET;
    }
}