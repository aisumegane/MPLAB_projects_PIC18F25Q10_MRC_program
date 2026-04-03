/* 
 * File:   int.h
 * Author: shunt
 *
 * Created on 2026/04/02, 22:40
 */

#ifndef SPEEDSENS_H
#define	SPEEDSENS_H

#ifdef	__cplusplus
extern "C" {
#endif


extern void func_speedsens_g_main( void );
extern void func_speedsens_g_init( void );

extern void func_speedsens_g_collect_mtr_capture( u16 u16_capture );
extern void func_speedsens_g_collect_1stgear_capture( u16 u16_capture );
extern u16 func_speedsens_g_calc_speed( u16 u16_capture_ave );

#ifdef	__cplusplus
}
#endif

#endif	/* SPEEDSENS_H */

