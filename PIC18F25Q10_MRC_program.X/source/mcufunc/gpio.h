/* 
 * File:   int.h
 * Author: unko
 *
 * Created on 2026/03/05, 22:40
 */

#ifndef GPIO_H
#define	GPIO_H

#ifdef	__cplusplus
extern "C" {
#endif

/* 入力ポート割り当て定義 */
#define GPIO_IN_RC_CH_SPEED                 RA0
#define GPIO_IN_RC_CH_SHIFT_MODE            RA1
#define GPIO_IN_RC_CH_SPEED_GAIN            RA2
#define GPIO_IN_RC_CH_SHIFT_UPDOWN          RA3
#define GPIO_IN_RC_CH_SHIFT_REV_LIMIT       RA4

/* 出力ポート割り当て定義 */
#define GPIO_OUT_SHIFT_NEUTRAL              LATB4
#define GPIO_OUT_SHIFT_0                    LATB3
#define GPIO_OUT_SHIFT_1                    LATB2
#define GPIO_OUT_SHIFT_2                    LATB1
#define GPIO_OUT_DEBUG                      LATB0



typedef struct gpio_in_define
{
    u8 u8_judge_cnt;
    u8 u8_buff;
    u8 u8_state;
    u8 u8_state_bf;
}gpio_in;


extern void func_gpio_g_main( void );
extern void func_gpio_g_init( void );


extern gpio_in gpio_g_paddle_shift_sw;
extern gpio_in gpio_g_shift_mode_sw;


extern u8 U8_GPIO_G_OUT_NEUTRAL;
extern u8 U8_GPIO_G_OUT_SHIFT_0;
extern u8 U8_GPIO_G_OUT_SHIFT_1;
extern u8 U8_GPIO_G_OUT_SHIFT_2;

extern u8 U8_GPIO_G_OUT_DEBUG;



#ifdef	__cplusplus
}
#endif

#endif	/* GPIO_H */