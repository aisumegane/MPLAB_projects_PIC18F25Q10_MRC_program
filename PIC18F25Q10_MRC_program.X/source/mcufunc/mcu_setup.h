/* 
 * File:   mcu_setup.h
 * Author: shunt
 *
 * Created on 2026/03/04, 22:56
 */

#ifndef MCU_SETUP_H
#define	MCU_SETUP_H

#ifdef	__cplusplus
extern "C" {
#endif

#define _XTAL_FREQ  64000000     /* 16MHz */        /* ここで定義するのは微妙かも？  @@要件等 */

/* 共通 */
extern void func_mset_g_mcu_start_condition( void );
extern void func_mset_g_init( void );
extern void func_mset_g_init( void );

/* タイマ5操作 */
extern void func_mset_g_timer5_onoff( u8 u8_state );
extern void func_mset_g_timer5_clear( void );
extern u16 func_mset_g_timer5_read( void );

#ifdef	__cplusplus
}
#endif

#endif	/* MCU_SETUP_H */

