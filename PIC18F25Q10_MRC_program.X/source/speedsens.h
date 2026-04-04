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

#define SPEEDSENS_MAX_SPEED_AT_1_CAPTURE   ((u32)30000000)               /* キャプチャ1あたりの 回転数 */

#define SPEEDSENS_CH_MTR                   ((u8)0)
#define SPEEDSENS_CH_1STGEAR               ((u8)1)
#define SPEEDSENS_CH_NUM                   ((u8)SPEEDSENS_CH_1STGEAR+(u8)1)

typedef struct speed_status_def
{
    u8 u8_buffer_filled;        /* キャプチャバッファすべて埋まったFLAG */
    u8 u8_cap_timer_reload;     /* キャプチャタイマ オーバーフロー発生FLAG (=キャプチャ値に確度がなくなるので回転数を代入したくない) */
    u8 u8_buff_num;             /* キャプチャバッファの総数 */
    u8 u8_buff_idx;             /* 現在のキャプチャバッファのIDX */
    u16 *u16_p_capture_buff;      /* キャプチャバッファの先頭要素へのポインタ */        /* @@キャプチャ保存の配列の型サイズと合わせないと、ポインタ指定で代入するとき上位ビットが消えるので注意 */
    u16 u16_capture_ave;        /* キャプチャバッファの平均値 */
    u16 u16_speed_ave;          /* 回転数の平均値 */
}ts_speed_status;

extern void func_speedsens_g_main( void );
extern void func_speedsens_g_init( void );

extern void func_speedsens_g_collect_capture( u16 u16_capture, ts_speed_status *sts );
extern void func_speedsens_g_reset_capture_sts( ts_speed_status *sts );

extern ts_speed_status speedsens_status[ SPEEDSENS_CH_NUM ];
extern u16 u16_speedsens_g_speed_ave_mtr;
extern u16 u16_speedsens_g_speed_ave_1stgear;

#ifdef	__cplusplus
}
#endif

#endif	/* SPEEDSENS_H */

