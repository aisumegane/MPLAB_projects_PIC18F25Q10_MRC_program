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


/* duty定義 */
#define RC_CH_DUTY_0P                   ((u8)0)
#define RC_CH_DUTY_1P                   ((u8)1)
#define RC_CH_DUTY_2P                   ((u8)2)
#define RC_CH_DUTY_3P                   ((u8)3)
#define RC_CH_DUTY_4P                   ((u8)4)
#define RC_CH_DUTY_5P                   ((u8)5)
#define RC_CH_DUTY_6P                   ((u8)6)
#define RC_CH_DUTY_7P                   ((u8)7)
#define RC_CH_DUTY_8P                   ((u8)8)
#define RC_CH_DUTY_9P                   ((u8)9)
#define RC_CH_DUTY_10P                  ((u8)10)

#define RC_CH_DUTY_11P                  ((u8)11)
#define RC_CH_DUTY_12P                  ((u8)12)
#define RC_CH_DUTY_13P                  ((u8)13)
#define RC_CH_DUTY_14P                  ((u8)14)
#define RC_CH_DUTY_15P                  ((u8)15)
#define RC_CH_DUTY_16P                  ((u8)16)
#define RC_CH_DUTY_17P                  ((u8)17)
#define RC_CH_DUTY_18P                  ((u8)18)
#define RC_CH_DUTY_19P                  ((u8)19)
#define RC_CH_DUTY_20P                  ((u8)20)

#define RC_CH_DUTY_21P                  ((u8)21)
#define RC_CH_DUTY_22P                  ((u8)22)
#define RC_CH_DUTY_23P                  ((u8)23)
#define RC_CH_DUTY_24P                  ((u8)24)
#define RC_CH_DUTY_25P                  ((u8)25)
#define RC_CH_DUTY_26P                  ((u8)26)
#define RC_CH_DUTY_27P                  ((u8)27)
#define RC_CH_DUTY_28P                  ((u8)28)
#define RC_CH_DUTY_29P                  ((u8)29)
#define RC_CH_DUTY_30P                  ((u8)30)

#define RC_CH_DUTY_31P                  ((u8)31)
#define RC_CH_DUTY_32P                  ((u8)32)
#define RC_CH_DUTY_33P                  ((u8)33)
#define RC_CH_DUTY_34P                  ((u8)34)
#define RC_CH_DUTY_35P                  ((u8)35)
#define RC_CH_DUTY_36P                  ((u8)36)
#define RC_CH_DUTY_37P                  ((u8)37)
#define RC_CH_DUTY_38P                  ((u8)38)
#define RC_CH_DUTY_39P                  ((u8)39)
#define RC_CH_DUTY_40P                  ((u8)40)

#define RC_CH_DUTY_41P                  ((u8)41)
#define RC_CH_DUTY_42P                  ((u8)42)
#define RC_CH_DUTY_43P                  ((u8)43)
#define RC_CH_DUTY_44P                  ((u8)44)
#define RC_CH_DUTY_45P                  ((u8)45)
#define RC_CH_DUTY_46P                  ((u8)46)
#define RC_CH_DUTY_47P                  ((u8)47)
#define RC_CH_DUTY_48P                  ((u8)48)
#define RC_CH_DUTY_49P                  ((u8)49)
#define RC_CH_DUTY_50P                  ((u8)50)

#define RC_CH_DUTY_51P                  ((u8)51)
#define RC_CH_DUTY_52P                  ((u8)52)
#define RC_CH_DUTY_53P                  ((u8)53)
#define RC_CH_DUTY_54P                  ((u8)54)
#define RC_CH_DUTY_55P                  ((u8)55)
#define RC_CH_DUTY_56P                  ((u8)56)
#define RC_CH_DUTY_57P                  ((u8)57)
#define RC_CH_DUTY_58P                  ((u8)58)
#define RC_CH_DUTY_59P                  ((u8)59)
#define RC_CH_DUTY_60P                  ((u8)60)

#define RC_CH_DUTY_61P                  ((u8)61)
#define RC_CH_DUTY_62P                  ((u8)62)
#define RC_CH_DUTY_63P                  ((u8)63)
#define RC_CH_DUTY_64P                  ((u8)64)
#define RC_CH_DUTY_65P                  ((u8)65)
#define RC_CH_DUTY_66P                  ((u8)66)
#define RC_CH_DUTY_67P                  ((u8)67)
#define RC_CH_DUTY_68P                  ((u8)68)
#define RC_CH_DUTY_69P                  ((u8)69)
#define RC_CH_DUTY_70P                  ((u8)70)

#define RC_CH_DUTY_71P                  ((u8)71)
#define RC_CH_DUTY_72P                  ((u8)72)
#define RC_CH_DUTY_73P                  ((u8)73)
#define RC_CH_DUTY_74P                  ((u8)74)
#define RC_CH_DUTY_75P                  ((u8)75)
#define RC_CH_DUTY_76P                  ((u8)76)
#define RC_CH_DUTY_77P                  ((u8)77)
#define RC_CH_DUTY_78P                  ((u8)78)
#define RC_CH_DUTY_79P                  ((u8)79)
#define RC_CH_DUTY_80P                  ((u8)80)

#define RC_CH_DUTY_81P                  ((u8)81)
#define RC_CH_DUTY_82P                  ((u8)82)
#define RC_CH_DUTY_83P                  ((u8)83)
#define RC_CH_DUTY_84P                  ((u8)84)
#define RC_CH_DUTY_85P                  ((u8)85)
#define RC_CH_DUTY_86P                  ((u8)86)
#define RC_CH_DUTY_87P                  ((u8)87)
#define RC_CH_DUTY_88P                  ((u8)88)
#define RC_CH_DUTY_89P                  ((u8)89)
#define RC_CH_DUTY_90P                  ((u8)90)

#define RC_CH_DUTY_91P                  ((u8)91)
#define RC_CH_DUTY_92P                  ((u8)92)
#define RC_CH_DUTY_93P                  ((u8)93)
#define RC_CH_DUTY_94P                  ((u8)94)
#define RC_CH_DUTY_95P                  ((u8)95)
#define RC_CH_DUTY_96P                  ((u8)96)
#define RC_CH_DUTY_97P                  ((u8)97)
#define RC_CH_DUTY_98P                  ((u8)98)
#define RC_CH_DUTY_99P                  ((u8)99)
#define RC_CH_DUTY_100P                 ((u8)100)


extern u8 u8_rc_g_duty_judge_sequence;

extern u8 u8_rc_g_duty_speed;
extern u8 u8_rc_g_duty_shift_mode;
extern u8 u8_rc_g_duty_speed_gain;
extern u8 u8_rc_g_duty_shift_updown;
extern u8 u8_rc_g_duty_rev_limit;

extern void func_rc_g_main( void );
extern void func_rc_g_init( void );
extern void func_rc_g_duty_detection( void );

#ifdef	__cplusplus
}
#endif

#endif	/* RADIO_CONTROL_H */