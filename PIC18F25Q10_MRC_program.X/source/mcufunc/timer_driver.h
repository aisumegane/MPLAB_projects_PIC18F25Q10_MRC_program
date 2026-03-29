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
extern void td_g_ccp1_pwm_duty_set( u16 u16_duty );
extern void td_g_ccp2_pwm_duty_set( u16 u16_duty );
extern void td_g_ccp3_pwm_duty_set( u16 u16_duty );



#ifdef	__cplusplus
}
#endif

#endif	/* TIMER_DRIVER_H */

