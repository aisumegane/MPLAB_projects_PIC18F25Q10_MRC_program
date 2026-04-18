/*
 * File:   inverter.c
 * Author: ICE_MEGANE
 *
 * Created on 2026/04/12, 14:38
 */

#ifndef INVERTER_H
#define	INVERTER_H

#ifdef	__cplusplus
extern "C" {
#endif


/* 出力状態 定義 (外部の関数で指定する方 シーケンスではない) */
#define HBRIDGE_OUTPUT_BRAKE        ((u8)0)
#define HBRIDGE_OUTPUT_CW           ((u8)1)
#define HBRIDGE_OUTPUT_CCW          ((u8)2)


/* duty定義 */
#define HBRIDGE_DUTY_MAX_CNT        ((u16)0x3FF)
#define HBRIDGE_DUTY_MAX_DEFINE     ((u16)1000)        /* 0.1%指定 */

#define HBRIDGE_DUTY_0P             ((u16)( ((u32)0   * (u32)HBRIDGE_DUTY_MAX_CNT ) / (u32)HBRIDGE_DUTY_MAX_DEFINE ))
#define HBRIDGE_DUTY_1P             ((u16)( ((u32)10  * (u32)HBRIDGE_DUTY_MAX_CNT ) / (u32)HBRIDGE_DUTY_MAX_DEFINE ))
#define HBRIDGE_DUTY_2P             ((u16)( ((u32)20  * (u32)HBRIDGE_DUTY_MAX_CNT ) / (u32)HBRIDGE_DUTY_MAX_DEFINE ))
#define HBRIDGE_DUTY_3P             ((u16)( ((u32)30  * (u32)HBRIDGE_DUTY_MAX_CNT ) / (u32)HBRIDGE_DUTY_MAX_DEFINE ))
#define HBRIDGE_DUTY_4P             ((u16)( ((u32)40  * (u32)HBRIDGE_DUTY_MAX_CNT ) / (u32)HBRIDGE_DUTY_MAX_DEFINE ))
#define HBRIDGE_DUTY_5P             ((u16)( ((u32)50  * (u32)HBRIDGE_DUTY_MAX_CNT ) / (u32)HBRIDGE_DUTY_MAX_DEFINE ))
#define HBRIDGE_DUTY_6P             ((u16)( ((u32)60  * (u32)HBRIDGE_DUTY_MAX_CNT ) / (u32)HBRIDGE_DUTY_MAX_DEFINE ))
#define HBRIDGE_DUTY_7P             ((u16)( ((u32)70  * (u32)HBRIDGE_DUTY_MAX_CNT ) / (u32)HBRIDGE_DUTY_MAX_DEFINE ))
#define HBRIDGE_DUTY_8P             ((u16)( ((u32)80  * (u32)HBRIDGE_DUTY_MAX_CNT ) / (u32)HBRIDGE_DUTY_MAX_DEFINE ))
#define HBRIDGE_DUTY_9P             ((u16)( ((u32)90  * (u32)HBRIDGE_DUTY_MAX_CNT ) / (u32)HBRIDGE_DUTY_MAX_DEFINE ))
#define HBRIDGE_DUTY_10P            ((u16)( ((u32)100 * (u32)HBRIDGE_DUTY_MAX_CNT ) / (u32)HBRIDGE_DUTY_MAX_DEFINE ))

#define HBRIDGE_DUTY_20P            ((u16)( ((u32)200 * (u32)HBRIDGE_DUTY_MAX_CNT ) / (u32)HBRIDGE_DUTY_MAX_DEFINE ))
#define HBRIDGE_DUTY_30P            ((u16)( ((u32)300 * (u32)HBRIDGE_DUTY_MAX_CNT ) / (u32)HBRIDGE_DUTY_MAX_DEFINE ))
#define HBRIDGE_DUTY_40P            ((u16)( ((u32)400 * (u32)HBRIDGE_DUTY_MAX_CNT ) / (u32)HBRIDGE_DUTY_MAX_DEFINE ))
#define HBRIDGE_DUTY_50P            ((u16)( ((u32)500 * (u32)HBRIDGE_DUTY_MAX_CNT ) / (u32)HBRIDGE_DUTY_MAX_DEFINE ))
#define HBRIDGE_DUTY_60P            ((u16)( ((u32)600 * (u32)HBRIDGE_DUTY_MAX_CNT ) / (u32)HBRIDGE_DUTY_MAX_DEFINE ))
#define HBRIDGE_DUTY_70P            ((u16)( ((u32)700 * (u32)HBRIDGE_DUTY_MAX_CNT ) / (u32)HBRIDGE_DUTY_MAX_DEFINE ))
#define HBRIDGE_DUTY_80P            ((u16)( ((u32)800 * (u32)HBRIDGE_DUTY_MAX_CNT ) / (u32)HBRIDGE_DUTY_MAX_DEFINE ))
#define HBRIDGE_DUTY_90P            ((u16)( ((u32)900 * (u32)HBRIDGE_DUTY_MAX_CNT ) / (u32)HBRIDGE_DUTY_MAX_DEFINE ))
#define HBRIDGE_DUTY_100P           ((u16)( ((u32)1000 * (u32)HBRIDGE_DUTY_MAX_CNT ) / (u32)HBRIDGE_DUTY_MAX_DEFINE ))








extern void func_inverter_g_init( void );
extern void func_hbridge_g_main( void );

extern void func_hbridge_control_set( u8 u8_state_req, u16 u16_duty_req );


extern u8 u8_hbridge_g_output_state_request;
extern u16 u16_hbridge_g_output_duty;

#ifdef	__cplusplus
}
#endif

#endif	/* INVERTER_H */

