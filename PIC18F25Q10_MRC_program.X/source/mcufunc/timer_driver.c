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
/*  PWM3 PWM モード duty設定関数                               */
/*  PWM3はCCPから独立してPWM専用となっている                     */
/**************************************************************/
void td_g_pwm3_pwm_duty_set( u16 u16_duty )
{
    u8 u8_duty_upper8bit;
    u8 u8_duty_lower2bit;

    /* なんでこんなレジスタ構成なんや。 */
    /* CCPRxL は10bitのdutyの上位8ビットを指定する */
    u8_duty_upper8bit = (u8)( u16_duty >> 2U );        /* 0b0011-1111-1111 -> 0b0000-1111-1111-(11) */

    /* DCxB は10bitのdutyの下位2bitを指定する */
    u16_duty &= (u16)0x0003;                /* 0b0011-1111-1111 -> 0b0000-0000-0011 */ /* 下位2bit以外を削除 */
    u8_duty_lower2bit = (u8)( u16_duty << 6U );        /* 0b0011-1111-1111 -> 0b0000-0011-0000 */ /* DCxB bitがある<7:6>bit目 まで位置をシフト */
    
    /* duty更新 */
    PWM3DCH = u8_duty_upper8bit;                /* duty 上位8bit 代入       */
    PWM3DCL = u8_duty_lower2bit;                /* duty 下位2bit 代入       */
}

/**************************************************************/
/*  Function:                                                 */
/*  PWM4 PWM モード duty設定関数                               */
/*  PWM4はCCPから独立してPWM専用となっている                     */
/**************************************************************/
void td_g_pwm4_pwm_duty_set( u16 u16_duty )
{
    u8 u8_duty_upper8bit;
    u8 u8_duty_lower2bit;

    /* なんでこんなレジスタ構成なんや。 */
    /* CCPRxL は10bitのdutyの上位8ビットを指定する */
    u8_duty_upper8bit = (u8)( u16_duty >> 2U );        /* 0b0011-1111-1111 -> 0b0000-1111-1111-(11) */

    /* DCxB は10bitのdutyの下位2bitを指定する */
    u16_duty &= (u16)0x0003;                /* 0b0011-1111-1111 -> 0b0000-0000-0011 */ /* 下位2bit以外を削除 */
    u8_duty_lower2bit = (u8)( u16_duty << 6U );        /* 0b0011-1111-1111 -> 0b0000-0011-0000 */ /* DCxB bitがある<7:6>bit目 まで位置をシフト */
    
    /* duty更新 */
    PWM4DCH = u8_duty_upper8bit;                /* duty 上位8bit 代入       */
    PWM4DCL = u8_duty_lower2bit;                /* duty 下位2bit 代入       */
}


/**************************************************************/
/*  Function:                                                 */
/*  CWG1 Full-Bridge モード 動作方向設定関数                    */
/**************************************************************/
void td_g_cwg1_mode_full_bridge_dir_set( u8 u8_dir_req )
{
    u8 u8_data_buff;
    
    u8_data_buff = CWG1CON0;
    u8_data_buff &= (u8)0x08;       /* MODE<2:0> ビットクリア */
    
    if( u8_dir_req == FORWARD ) /* SET */
    {
        u8_data_buff += (u8)0x02;       /* 0b010 */
    }
    else if( u8_dir_req == BACKWARD )   /* CLEAR */
    {
        u8_data_buff += (u8)0x03;       /* 0b011 */
    }
    else
    {
        u8_data_buff += (u8)0x02;       /* 0b010 */     /* ひとまず正転*/
    }
    
    /* レジスタ更新 */
    CWG1CON0 = u8_data_buff;    
}



#if ( UNUSED_FUNCTION_HIDE_SETTING == SET )
/**************************************************************/
/*  Function:                                                 */
/*  CCP1 PWMモード duty設定関数                                */
/*                                                            */
/**************************************************************/
void td_g_ccp1_mode_pwm_duty_set( u16 u16_duty )
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
/*  CCP1 PWMモード duty設定関数                                */
/*                                                            */
/**************************************************************/
void td_g_ccp2_mode_pwm_duty_set( u16 u16_duty )
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
/*  PWM3 PWM モード duty設定関数                               */
/*  PWM3はCCPから独立してPWM専用となっている                     */
/**************************************************************/
void td_g_pwm4_pwm_duty_set( u16 u16_duty )
{
    u8 u8_duty_upper8bit;
    u8 u8_duty_lower2bit;

    /* なんでこんなレジスタ構成なんや。 */
    /* CCPRxL は10bitのdutyの上位8ビットを指定する */
    u8_duty_upper8bit = (u8)( u16_duty >> 2U );        /* 0b0011-1111-1111 -> 0b0000-1111-1111-(11) */

    /* DCxB は10bitのdutyの下位2bitを指定する */
    u16_duty &= (u16)0x0003;                /* 0b0011-1111-1111 -> 0b0000-0000-0011 */ /* 下位2bit以外を削除 */
    u8_duty_lower2bit = (u8)( u16_duty << 6U );        /* 0b0011-1111-1111 -> 0b0000-0011-0000 */ /* DCxB bitがある<7:6>bit目 まで位置をシフト */
    
    /* duty更新 */
    PWM4DCH = u8_duty_upper8bit;                /* duty 上位8bit 代入       */
    PWM4DCH = u8_duty_lower2bit;                /* duty 下位2bit 代入       */
}
#endif

