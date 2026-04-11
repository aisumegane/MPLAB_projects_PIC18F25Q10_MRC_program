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

/* 変速モード */
#define SHIFT_MODE_MANUAL           ((u8)0)       /* 手動でシフトアップ */
#define SHIFT_MODE_AUTOMATIC        ((u8)1)       /* 自動でシフトアップ */

/* シフトポジション */
#define SHIFT_POSI_0                ((u8)0)
#define SHIFT_POSI_1                ((u8)1)
#define SHIFT_POSI_2                ((u8)2)
#define SHIFT_POSI_3                ((u8)3)
#define SHIFT_POSI_4                ((u8)4)
#define SHIFT_POSI_5                ((u8)5)
#define SHIFT_POSI_6                ((u8)6)
#define SHIFT_POSI_7                ((u8)7)
#define SHIFT_POSI_NUM              (SHIFT_POSI_7 + (u8)1)

/* 変速シーケンス */
#define SHIFT_SEQ_CLUTCH_OFF_STOP           ((u8)0)      /* 停止状態でクラッチOFF */
#define SHIFT_SEQ_CLUTCH_MEETING            ((u8)1)
#define SHIFT_SEQ_CLUTCH_ON_DRIVE           ((u8)2)
#define SHIFT_SEQ_CLUTCH_OFF_BFORE_CHG      ((u8)3)
#define SHIFT_SEQ_CLUTCH_OFF_SHIFT_CHG      ((u8)4)
#define SHIFT_SEQ_CLUTCH_OFF_AFTER_CHG      ((u8)5)
#define SHIFT_SEQ_CLUTCH_OFF_KICK           ((u8)6)


extern void func_shift_g_main( void );
extern void func_shift_g_init( void );


extern u8 u8_shift_g_shifting_sequence;
extern u8 u8_shift_g_shifting_sequence_before;
extern const u16 u16_shift_s_gear_ratio_ary[ SHIFT_POSI_NUM ];


#ifdef	__cplusplus
}
#endif

#endif	/* SHIFT_H */