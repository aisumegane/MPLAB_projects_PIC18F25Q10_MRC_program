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

typedef struct gpio_in
{
    u8 u8_judge_cnt;
    u8 u8_buff;
    u8 u8_state;
}ts_gpio_in_def;

extern void func_gpio_g_main( void );
extern void func_gpio_g_init( void );


extern ts_gpio_in_def ts_gpio_g_in_shift_0;
extern ts_gpio_in_def ts_gpio_g_in_shift_1;
extern ts_gpio_in_def ts_gpio_g_in_shift_2;
extern ts_gpio_in_def ts_gpio_g_in_neutral;

extern u8 U8_GPIO_G_OUT_TASK_CHK;



#ifdef	__cplusplus
}
#endif

#endif	/* GPIO_H */