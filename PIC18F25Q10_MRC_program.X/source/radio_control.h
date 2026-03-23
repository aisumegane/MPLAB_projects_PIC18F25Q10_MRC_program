/* 
 * File:   indicate.h
 * Author: HGS01
 *
 * Created on 2026/03/23, 20:40
 */

#ifndef RADIO_CONTROL_H
#define	RADIO_CONTROL_H

#ifdef	__cplusplus
extern "C" {
#endif

/* パルス幅測定 シーケンス定義 */
#define RC_SEQ_DUTY_JUDGE_INIT           ((u8)0)        /* duty測定 準備中 */
#define RC_SEQ_DUTY_JUDGE_START          ((u8)1)        /* duty測定 開始 */
#define RC_SEQ_DUTY_JUDGE_END            ((u8)2)        /* 全チャネルのパルス幅測定完了 */
#define RC_SEQ_DUTY_JUDGE_TIMEOUT        ((u8)3)        /* duty測定 タイムアウト判定 */
#define RC_SEQ_DUTY_JUDGE_FAILE          ((u8)4)        /* duty測定 失敗判定 */

u8 u8_rc_g_duty_judge_sequence;


extern void func_rc_g_main( void );
extern void func_rc_g_init( void );
extern void func_rc_g_duty_detection( void );

#ifdef	__cplusplus
}
#endif

#endif	/* RADIO_CONTROL_H */