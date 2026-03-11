/*
 * File:   main.c
 * Author: ICE_MEGANE
 *
 * Created on 2020/03/5, 22:26
 */

#include <xc.h>
#include "pic16F1827.h"
#include "userdefine.h"


#include "./mcufunc/gpio.h"

#include "./tools/segment.h"


#include "indicate.h"
#include "shift.h"


/* シフトチェンジの制御を担当 */

/* プロトタイプ宣言 */
static void func_indicate_s_shift_posi_disp( void );


/* グローバル変数 */
/**************************************************************/
/*  Function:                                                 */
/*  main関数                                                   */
/*                                                            */
/*                                                            */
/**************************************************************/
void func_indicate_g_main( void )
{
    func_indicate_s_shift_posi_disp();
}


/**************************************************************/
/*  Function:                                                 */
/*  初期化関数                                                 */
/*                                                            */
/*                                                            */
/**************************************************************/
void func_indicate_g_init( void )
{
    ;
}


/**************************************************************/
/*  Function:                                                 */
/* シフト位置を表示する                                         */
/*                                                            */
/**************************************************************/
static void func_indicate_s_shift_posi_disp( void )
{
    u8 u8_shift_buff;
    
    u8_shift_buff = (u8)0;
    
    /* 0~7の計8位置。附番とずれてるのは若干ややこしいかも? */
    if( ts_gpio_g_in_shift_0.u8_state == SET )
    {
        u8_shift_buff += (u8)0x01;
    }
    if( ts_gpio_g_in_shift_1.u8_state == SET )
    {
        u8_shift_buff += (u8)0x02;
    }
    if( ts_gpio_g_in_shift_2.u8_state == SET )
    {
        u8_shift_buff += (u8)0x04;
    }
    
    /* 1速始まりにしたいので1加算 */
    u8_shift_buff += (u8)1;
    
    func_segment_g_tc4511bp_data_set( u8_shift_buff );
}

