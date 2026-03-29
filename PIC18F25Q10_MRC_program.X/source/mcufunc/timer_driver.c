/*
 * File:   main.c
 * Author: ICE_MEGANE
 *
 * Created on 2020/03/5, 22:26
 */

#include <xc.h>
#include "../userdefine.h"


#include "timer_driver.h"

/* タイマ5 */
/**************************************************************/
/*  Function:                                                 */
/*  タイマ5 動作停止/開始関数                                   */
/**************************************************************/
void func_mset_g_timer5_onoff( u8 u8_state )
{
    if( u8_state == ON )
    {
        TMR5IF = CLEAR;
        TMR5ON = SET;
    }
    else
    {
        TMR5IF = CLEAR;
        TMR5ON = OFF;
    }
}

/**************************************************************/
/*  Function:                                                 */
/*  タイマ5 カウンタレジスタクリア関数                           */
/**************************************************************/
void func_mset_g_timer5_couter_set( u16 u16_val )
{
    TMR5H = (u8)( u16_val >> 8U );
    TMR5L = (u8)u16_val;
}

/**************************************************************/
/*  Function:                                                 */
/*  タイマ5 カウンタレジス読み取り関数                           */
/**************************************************************/
u16 func_mset_g_timer5_read( void )
{   
   return   TMR5;               /* ※16bit読み出し可能 */ 
}

/*---------------------------------------------------------------------------------------------------------------------------------------------------------*/

/**************************************************************/
/*  Function:                                                 */
/*  CCP1を使ったPWMのduty設定                                  */
/*                                                            */
/**************************************************************/
void td_g_ccp1_pwm_duty_set( u16 u16_duty )
{
    u8 u8_duty_upper8bit;
    u8 u8_duty_lower2bit;

    /* なんでこんなレジスタ構成なんや。 */
    /* CCPRxL は10bitのdutyの上位8ビットを指定する */
    u8_duty_upper8bit = (u8)( u16_duty >> 2U );        /* 0b0011-1111-1111 -> 0b0000-1111-1111-(11) */

    /* DCxB は10bitのdutyの下位2bitを指定する */
    u16_duty &= (u16)0x0003;                /* 0b0011-1111-1111 -> 0b0000-0000-0011 */ /* 下位2bit以外を削除 */
    u8_duty_lower2bit = (u8)( u16_duty << 4U );        /* 0b0011-1111-1111 -> 0b0000-0011-0000 */ /* DCxB bitがある<5:4>bit目まで位置をシフト */
    
    /* duty更新 */
    CCPR1L = u8_duty_upper8bit;                        /* duty 上位8bit 代入       */

    CCP1CON &= (u8)0xCF;                               /* DCxB いったん現在値クリア */
    CCP1CON |= u8_duty_lower2bit;                      /* duty 下位2bit 代入       */
}


/**************************************************************/
/*  Function:                                                 */
/*  CCP2を使ったPWMのduty設定                                  */
/*                                                            */
/**************************************************************/
void td_g_ccp2_pwm_duty_set( u16 u16_duty )
{
    u8 u8_duty_upper8bit;
    u8 u8_duty_lower2bit;

    /* なんでこんなレジスタ構成なんや。 */
    /* CCPRxL は10bitのdutyの上位8ビットを指定する */
    u8_duty_upper8bit = (u8)( u16_duty >> 2U );        /* 0b0011-1111-1111 -> 0b0000-1111-1111-(11) */

    /* DCxB は10bitのdutyの下位2bitを指定する */
    u16_duty &= (u16)0x0003;                /* 0b0011-1111-1111 -> 0b0000-0000-0011 */ /* 下位2bit以外を削除 */
    u8_duty_lower2bit = (u8)( u16_duty << 4U );        /* 0b0011-1111-1111 -> 0b0000-0011-0000 */ /* DCxB bitがある<5:4>bit目まで位置をシフト */
    
    /* duty更新 */
    CCPR2L = u8_duty_upper8bit;                        /* duty 上位8bit 代入       */

    CCP2CON &= (u8)0xCF;                               /* DCxB いったん現在値クリア */
    CCP2CON |= u8_duty_lower2bit;                      /* duty 下位2bit 代入       */
}


/**************************************************************/
/*  Function:                                                 */
/*  CCP3を使ったPWMのduty設定                                  */
/*                                                            */
/**************************************************************/
void td_g_ccp3_pwm_duty_set( u16 u16_duty )
{
    u8 u8_duty_upper8bit;
    u8 u8_duty_lower2bit;

    /* なんでこんなレジスタ構成なんや。 */
    /* CCPRxL は10bitのdutyの上位8ビットを指定する */
    u8_duty_upper8bit = (u8)( u16_duty >> 2U );        /* 0b0011-1111-1111 -> 0b0000-1111-1111-(11) */

    /* DCxB は10bitのdutyの下位2bitを指定する */
    u16_duty &= (u16)0x0003;                /* 0b0011-1111-1111 -> 0b0000-0000-0011 */ /* 下位2bit以外を削除 */
    u8_duty_lower2bit = (u8)( u16_duty << 4U );        /* 0b0011-1111-1111 -> 0b0000-0011-0000 */ /* DCxB bitがある<5:4>bit目まで位置をシフト */
    
    /* duty更新 */
    //CCPR3L = u8_duty_upper8bit;                        /* duty 上位8bit 代入       */

    //CCP3CON &= (u8)0xCF;                               /* DCxB いったん現在値クリア */
    //CCP3CON |= u8_duty_lower2bit;                      /* duty 下位2bit 代入       */
}


