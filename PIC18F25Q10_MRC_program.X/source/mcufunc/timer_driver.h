/* 
 * File:   mcu_setup.h
 * Author: shunt
 *
 * Created on 2026/03/04, 22:56
 */

#ifndef TIMER_DRIVER_H
#define	TIMER_DRIVER_H

#ifdef	__cplusplus
extern "C" {
#endif



/* タイマ5 */
extern void func_mset_g_timer5_onoff( u8 u8_state );
extern void func_mset_g_timer5_couter_set( u16 u16_val );
extern u16 func_mset_g_timer5_read( void );


/* CCP機能 */


/* PWM機能 */
extern void td_g_pwm3_pwm_duty_set( u16 u16_duty );

/* CWG機能 */
extern void td_g_cwg1_mode_full_bridge_dir_set( u8 u8_dir_req );


/* 未使用関数群 */
#if( UNUSED_FUNCTION_HIDE_SETTING == SET )
extern void td_g_ccp1_mode_pwm_duty_set( u16 u16_duty );                /* CCP1 Mode:PWM duty設定関数 */
extern void td_g_ccp2_mode_pwm_duty_set( u16 u16_duty );                /* CCP2 Mode:PWM duty設定関数 */
extern void td_g_pwm4_pwm_duty_set( u16 u16_duty );                     /* PWM4 PWM duty設定関数      */
#endif

#ifdef	__cplusplus
}
#endif

#endif	/* TIMER_DRIVER_H */

