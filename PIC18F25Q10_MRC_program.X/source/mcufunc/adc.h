/* 
 * File:   adc.h
 * Author: shunt
 *
 * Created on 2026/03/04, 22:26
 */

#ifndef ADC_H
#define	ADC_H

#ifdef	__cplusplus
extern "C" {
#endif

#define ADC_10BIT_MAX   ((u16)0x3FF)



/* 関数宣言 */
extern void func_adc_g_main( void );                       /* AD変換処理 メインループ用 */
extern void func_adc_g_init( void );                       /* AD変換処理 変数初期化 */
extern void func_adc_g_adc_data_get( void );               /* AD変換結果 取得処理 割り込み処理用 */

u16 u16_adc_g_ad_result_ave____servo_posi_adj;             /* サーボ位置調整用AD変換結果 */



#ifdef	__cplusplus
}
#endif

#endif	/* ADC_H */

