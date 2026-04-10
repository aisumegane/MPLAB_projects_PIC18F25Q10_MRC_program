/* 
 * File:   shift.h
 * Author: shunt
 *
 * Created on 2026/03/04, 22:40
 */

#ifndef SHIFT_H
#define	SHIFT_H

#ifdef	__cplusplus
extern "C" {
#endif

#define SHIFT_MODE_MANUAL           ((u8)0)       /* 手動でシフトアップ */
#define SHIFT_MODE_AUTOMATIC        ((u8)1)       /* 自動でシフトアップ */

#define SHIFT_POSI_0                ((u8)0)
#define SHIFT_POSI_1                ((u8)1)
#define SHIFT_POSI_2                ((u8)2)
#define SHIFT_POSI_3                ((u8)3)
#define SHIFT_POSI_4                ((u8)4)
#define SHIFT_POSI_5                ((u8)5)
#define SHIFT_POSI_6                ((u8)6)
#define SHIFT_POSI_7                ((u8)7)
#define SHIFT_POSI_NUM              (SHIFT_POSI_7 + (u8)1)


extern void func_shift_g_main( void );
extern void func_shift_g_init( void );


#ifdef	__cplusplus
}
#endif

#endif	/* SHIFT_H */