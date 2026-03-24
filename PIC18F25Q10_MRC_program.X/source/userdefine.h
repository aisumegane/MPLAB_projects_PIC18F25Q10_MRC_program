/* 
 * File:   adc.h
 * Author: shunt
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

#define     SET     1U
#define     ON      1U
#define     HI      1U


/* 共通型名称 */
typedef  unsigned char   u8;
typedef  unsigned short  u16;
typedef  unsigned long   u32;


#define     U8_MAX     ((u8)0xFF)
#define     U16_MAX     ((u8)0xFFFF)
#define     U32_MAX     ((u8)0xFFFFFFFF)


#ifdef	__cplusplus
}
#endif

#endif	/* USERDEFINE_H */
