/* 
 * File:   userdefine.h
 * Author: ICE_MEGANE
 *
 * Created on 2026/03/04, 22:26
 */

#ifndef USERDEFINE_H
#define	USERDEFINE_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "./mcufunc/pic18f25q10.h"

/* 共通マクロ設定 */
#define     CLEAR   0U
#define     OFF     0U
#define     LOW     0U
#define     DOWN    0U

#define     SET     1U
#define     ON      1U
#define     HI      1U
#define     UP      1U

#define     MID     3U

/* 共通型名称 */
typedef  unsigned char   u8;
typedef  unsigned short  u16;
typedef  unsigned long   u32;

#define     U8_MAX     ((u8)0xFF)
#define     U16_MAX    ((u16)0xFFFF)
#define     U32_MAX    ((u32)0xFFFFFFFF)



/* 全ファイル共通関数 */
extern u32 func_ud_g_calcmul_2x2_byte( u16 u16_arg1, u16 u16_arg2 );
extern u32 func_ud_g_calcdiv_4x4_byte( u32 u32_arg1, u32 u32_arg2 );

#ifdef	__cplusplus
}
#endif

#endif	/* USERDEFINE_H */
