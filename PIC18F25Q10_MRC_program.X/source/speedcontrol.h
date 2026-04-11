/* 
 * File:   speedcontrol.h
 * Author: ICE_MEGANE
 *
 * Created on 2026/04/05, 22:40
 */

#ifndef SPEEDCONTROL_H
#define	SPEEDCONTROL_H

#ifdef	__cplusplus
extern "C" {
#endif

#define SC_THROTTLE_DIR_NONE            ((u8)0)                 /* 中立 */
#define SC_THROTTLE_DIR_FORWARD         ((u8)1)                 /* 前進 */
#define SC_THROTTLE_DIR_BACKWARD        ((u8)2)                 /* 後退 */

extern void func_speedcontrol_g_main( void );
extern void func_speedcontrol_g_init( void );

extern u8 u8_sc_s_throttle_dir;
extern u8 u8_sc_g_throttle_rc_ch_duty_target;

#ifdef	__cplusplus
}
#endif

#endif	/* SPEEDCONTROL_H */