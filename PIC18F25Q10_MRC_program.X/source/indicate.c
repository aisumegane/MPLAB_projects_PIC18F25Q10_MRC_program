/*
 * File:   main.c
 * Author: ICE_MEGANE
 *
 * Created on 2020/03/5, 22:26
 */

#include <xc.h>
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

}

