/*
 * File:   servo.c
 * Author: ICE_MEGANE
 *
 * Created on 2020/03/5, 22:26
 */

#include <xc.h>
#include "../userdefine.h"


#include "../mcufunc/timer_driver.h"

#include "servo.h"


/* ファイル内定義 */
/* PIC16シリーズは割り算ができない(超遅い)。あらかじめ角度に相当するパルス幅をROM定義して配列参照でパッと呼び出す。 */
/* ↓↓↓この値を変更する場合は合わせてタイマのクロック、PWM設定を全部見直す */
#define SERVO_PWM_CNT_MAX            (u16)600U        /* サーボ角度:180度に相当する PWM-duty設定 */  /* Ton_servo/((1/Tosc)*prescaler) = 0.0024/((1/16MHz)*64) */
#define SERVO_PWM_CNT_MIN            (u16)125U        /* サーボ角度:  0度に相当する PWM-duty設定 */  /* Ton_servo/((1/Tosc)*prescaler) = 0.0005/((1/16MHz)*64) */

#define SERVO_ANGLE_MAX              (u16)180U        /* サーボ 最大角度 */
#define SERVO_ANGLE_MIN              (u16)0U          /* サーボ 最小角度 */

/* サーボの最大-最小角度に応じて、PWM-dutyのカウント数を線形補完して定義する */
#define SERVO_ANGLE___0DEG      (u16)((((u32)0 )*( (u32)SERVO_PWM_CNT_MAX - (u32)SERVO_PWM_CNT_MIN )) / ( (u32)SERVO_ANGLE_MAX ) + (u32)SERVO_PWM_CNT_MIN)
#define SERVO_ANGLE___5DEG      (u16)((((u32)5 )*( (u32)SERVO_PWM_CNT_MAX - (u32)SERVO_PWM_CNT_MIN )) / ( (u32)SERVO_ANGLE_MAX ) + (u32)SERVO_PWM_CNT_MIN)
#define SERVO_ANGLE__10DEG      (u16)((((u32)10)*( (u32)SERVO_PWM_CNT_MAX - (u32)SERVO_PWM_CNT_MIN )) / ( (u32)SERVO_ANGLE_MAX ) + (u32)SERVO_PWM_CNT_MIN)
#define SERVO_ANGLE__15DEG      (u16)((((u32)15)*( (u32)SERVO_PWM_CNT_MAX - (u32)SERVO_PWM_CNT_MIN )) / ( (u32)SERVO_ANGLE_MAX ) + (u32)SERVO_PWM_CNT_MIN)
#define SERVO_ANGLE__20DEG      (u16)((((u32)20)*( (u32)SERVO_PWM_CNT_MAX - (u32)SERVO_PWM_CNT_MIN )) / ( (u32)SERVO_ANGLE_MAX ) + (u32)SERVO_PWM_CNT_MIN)
#define SERVO_ANGLE__25DEG      (u16)((((u32)25)*( (u32)SERVO_PWM_CNT_MAX - (u32)SERVO_PWM_CNT_MIN )) / ( (u32)SERVO_ANGLE_MAX ) + (u32)SERVO_PWM_CNT_MIN)
#define SERVO_ANGLE__30DEG      (u16)((((u32)30)*( (u32)SERVO_PWM_CNT_MAX - (u32)SERVO_PWM_CNT_MIN )) / ( (u32)SERVO_ANGLE_MAX ) + (u32)SERVO_PWM_CNT_MIN)
#define SERVO_ANGLE__35DEG      (u16)((((u32)35)*( (u32)SERVO_PWM_CNT_MAX - (u32)SERVO_PWM_CNT_MIN )) / ( (u32)SERVO_ANGLE_MAX ) + (u32)SERVO_PWM_CNT_MIN)
#define SERVO_ANGLE__40DEG      (u16)((((u32)40)*( (u32)SERVO_PWM_CNT_MAX - (u32)SERVO_PWM_CNT_MIN )) / ( (u32)SERVO_ANGLE_MAX ) + (u32)SERVO_PWM_CNT_MIN)
#define SERVO_ANGLE__45DEG      (u16)((((u32)45)*( (u32)SERVO_PWM_CNT_MAX - (u32)SERVO_PWM_CNT_MIN )) / ( (u32)SERVO_ANGLE_MAX ) + (u32)SERVO_PWM_CNT_MIN)
#define SERVO_ANGLE__50DEG      (u16)((((u32)50)*( (u32)SERVO_PWM_CNT_MAX - (u32)SERVO_PWM_CNT_MIN )) / ( (u32)SERVO_ANGLE_MAX ) + (u32)SERVO_PWM_CNT_MIN)
#define SERVO_ANGLE__55DEG      (u16)((((u32)55)*( (u32)SERVO_PWM_CNT_MAX - (u32)SERVO_PWM_CNT_MIN )) / ( (u32)SERVO_ANGLE_MAX ) + (u32)SERVO_PWM_CNT_MIN)
#define SERVO_ANGLE__60DEG      (u16)((((u32)60)*( (u32)SERVO_PWM_CNT_MAX - (u32)SERVO_PWM_CNT_MIN )) / ( (u32)SERVO_ANGLE_MAX ) + (u32)SERVO_PWM_CNT_MIN)
#define SERVO_ANGLE__65DEG      (u16)((((u32)65)*( (u32)SERVO_PWM_CNT_MAX - (u32)SERVO_PWM_CNT_MIN )) / ( (u32)SERVO_ANGLE_MAX ) + (u32)SERVO_PWM_CNT_MIN)
#define SERVO_ANGLE__70DEG      (u16)((((u32)70)*( (u32)SERVO_PWM_CNT_MAX - (u32)SERVO_PWM_CNT_MIN )) / ( (u32)SERVO_ANGLE_MAX ) + (u32)SERVO_PWM_CNT_MIN)
#define SERVO_ANGLE__75DEG      (u16)((((u32)75)*( (u32)SERVO_PWM_CNT_MAX - (u32)SERVO_PWM_CNT_MIN )) / ( (u32)SERVO_ANGLE_MAX ) + (u32)SERVO_PWM_CNT_MIN)
#define SERVO_ANGLE__80DEG      (u16)((((u32)80)*( (u32)SERVO_PWM_CNT_MAX - (u32)SERVO_PWM_CNT_MIN )) / ( (u32)SERVO_ANGLE_MAX ) + (u32)SERVO_PWM_CNT_MIN)
#define SERVO_ANGLE__85DEG      (u16)((((u32)85)*( (u32)SERVO_PWM_CNT_MAX - (u32)SERVO_PWM_CNT_MIN )) / ( (u32)SERVO_ANGLE_MAX ) + (u32)SERVO_PWM_CNT_MIN)
#define SERVO_ANGLE__90DEG      (u16)((((u32)90)*( (u32)SERVO_PWM_CNT_MAX - (u32)SERVO_PWM_CNT_MIN )) / ( (u32)SERVO_ANGLE_MAX ) + (u32)SERVO_PWM_CNT_MIN)
#define SERVO_ANGLE__95DEG      (u16)((((u32)95)*( (u32)SERVO_PWM_CNT_MAX - (u32)SERVO_PWM_CNT_MIN )) / ( (u32)SERVO_ANGLE_MAX ) + (u32)SERVO_PWM_CNT_MIN)
#define SERVO_ANGLE__100DEG     (u16)((((u32)100)*( (u32)SERVO_PWM_CNT_MAX - (u32)SERVO_PWM_CNT_MIN )) / ( (u32)SERVO_ANGLE_MAX ) + (u32)SERVO_PWM_CNT_MIN)
#define SERVO_ANGLE__105DEG     (u16)((((u32)105)*( (u32)SERVO_PWM_CNT_MAX - (u32)SERVO_PWM_CNT_MIN )) / ( (u32)SERVO_ANGLE_MAX ) + (u32)SERVO_PWM_CNT_MIN)
#define SERVO_ANGLE__110DEG     (u16)((((u32)110)*( (u32)SERVO_PWM_CNT_MAX - (u32)SERVO_PWM_CNT_MIN )) / ( (u32)SERVO_ANGLE_MAX ) + (u32)SERVO_PWM_CNT_MIN)
#define SERVO_ANGLE__115DEG     (u16)((((u32)115)*( (u32)SERVO_PWM_CNT_MAX - (u32)SERVO_PWM_CNT_MIN )) / ( (u32)SERVO_ANGLE_MAX ) + (u32)SERVO_PWM_CNT_MIN)
#define SERVO_ANGLE__120DEG     (u16)((((u32)120)*( (u32)SERVO_PWM_CNT_MAX - (u32)SERVO_PWM_CNT_MIN )) / ( (u32)SERVO_ANGLE_MAX ) + (u32)SERVO_PWM_CNT_MIN)
#define SERVO_ANGLE__125DEG     (u16)((((u32)125)*( (u32)SERVO_PWM_CNT_MAX - (u32)SERVO_PWM_CNT_MIN )) / ( (u32)SERVO_ANGLE_MAX ) + (u32)SERVO_PWM_CNT_MIN)
#define SERVO_ANGLE__130DEG     (u16)((((u32)130)*( (u32)SERVO_PWM_CNT_MAX - (u32)SERVO_PWM_CNT_MIN )) / ( (u32)SERVO_ANGLE_MAX ) + (u32)SERVO_PWM_CNT_MIN)
#define SERVO_ANGLE__135DEG     (u16)((((u32)135)*( (u32)SERVO_PWM_CNT_MAX - (u32)SERVO_PWM_CNT_MIN )) / ( (u32)SERVO_ANGLE_MAX ) + (u32)SERVO_PWM_CNT_MIN)
#define SERVO_ANGLE__140DEG     (u16)((((u32)140)*( (u32)SERVO_PWM_CNT_MAX - (u32)SERVO_PWM_CNT_MIN )) / ( (u32)SERVO_ANGLE_MAX ) + (u32)SERVO_PWM_CNT_MIN)
#define SERVO_ANGLE__145DEG     (u16)((((u32)145)*( (u32)SERVO_PWM_CNT_MAX - (u32)SERVO_PWM_CNT_MIN )) / ( (u32)SERVO_ANGLE_MAX ) + (u32)SERVO_PWM_CNT_MIN)
#define SERVO_ANGLE__150DEG     (u16)((((u32)150)*( (u32)SERVO_PWM_CNT_MAX - (u32)SERVO_PWM_CNT_MIN )) / ( (u32)SERVO_ANGLE_MAX ) + (u32)SERVO_PWM_CNT_MIN)
#define SERVO_ANGLE__155DEG     (u16)((((u32)155)*( (u32)SERVO_PWM_CNT_MAX - (u32)SERVO_PWM_CNT_MIN )) / ( (u32)SERVO_ANGLE_MAX ) + (u32)SERVO_PWM_CNT_MIN)
#define SERVO_ANGLE__160DEG     (u16)((((u32)160)*( (u32)SERVO_PWM_CNT_MAX - (u32)SERVO_PWM_CNT_MIN )) / ( (u32)SERVO_ANGLE_MAX ) + (u32)SERVO_PWM_CNT_MIN)
#define SERVO_ANGLE__165DEG     (u16)((((u32)165)*( (u32)SERVO_PWM_CNT_MAX - (u32)SERVO_PWM_CNT_MIN )) / ( (u32)SERVO_ANGLE_MAX ) + (u32)SERVO_PWM_CNT_MIN)
#define SERVO_ANGLE__170DEG     (u16)((((u32)170)*( (u32)SERVO_PWM_CNT_MAX - (u32)SERVO_PWM_CNT_MIN )) / ( (u32)SERVO_ANGLE_MAX ) + (u32)SERVO_PWM_CNT_MIN)
#define SERVO_ANGLE__175DEG     (u16)((((u32)175)*( (u32)SERVO_PWM_CNT_MAX - (u32)SERVO_PWM_CNT_MIN )) / ( (u32)SERVO_ANGLE_MAX ) + (u32)SERVO_PWM_CNT_MIN)
#define SERVO_ANGLE__180DEG     (u16)((((u32)180)*( (u32)SERVO_PWM_CNT_MAX - (u32)SERVO_PWM_CNT_MIN )) / ( (u32)SERVO_ANGLE_MAX ) + (u32)SERVO_PWM_CNT_MIN)
/* 角度と数値の割り当て具合： 1degree ~= 2 : 0DEG = 125, 1DEG = 127, ... */


/* サーボ角度定義 */
/* 分解能は5度にした。この程度でも十分に滑らかに角度調節できる。 */
/* forループ検索したりしても良いかも。 @@要件等 */
const static u16 u16_servo_s_angle_set_array[ SERVO_ANGLE_NUM ] = 
{
    SERVO_ANGLE___0DEG,
    SERVO_ANGLE___5DEG,
    SERVO_ANGLE__10DEG,
    SERVO_ANGLE__15DEG,
    SERVO_ANGLE__20DEG,
    SERVO_ANGLE__25DEG,
    SERVO_ANGLE__30DEG,
    SERVO_ANGLE__35DEG,
    SERVO_ANGLE__40DEG,
    SERVO_ANGLE__45DEG,
    SERVO_ANGLE__50DEG,
    SERVO_ANGLE__55DEG,
    SERVO_ANGLE__60DEG,
    SERVO_ANGLE__65DEG,
    SERVO_ANGLE__70DEG,
    SERVO_ANGLE__75DEG,
    SERVO_ANGLE__80DEG,
    SERVO_ANGLE__85DEG,
    SERVO_ANGLE__90DEG,
    SERVO_ANGLE__95DEG,
    SERVO_ANGLE__100DEG,
    SERVO_ANGLE__105DEG,
    SERVO_ANGLE__110DEG,
    SERVO_ANGLE__115DEG,
    SERVO_ANGLE__120DEG,
    SERVO_ANGLE__125DEG,
    SERVO_ANGLE__130DEG,
    SERVO_ANGLE__135DEG,
    SERVO_ANGLE__140DEG,
    SERVO_ANGLE__145DEG,
    SERVO_ANGLE__150DEG,
    SERVO_ANGLE__155DEG,
    SERVO_ANGLE__160DEG,
    SERVO_ANGLE__165DEG,
    SERVO_ANGLE__170DEG,
    SERVO_ANGLE__175DEG,
    SERVO_ANGLE__180DEG
};


/* 関数プロトタイプ宣言 */

/**************************************************************/
/*  Function:                                                 */
/*  サーボ関連 初期化関数                                       */
/*                                                            */
/**************************************************************/
void func_servo_g_init( void )
{
    ;
}


/*  */
/**************************************************************/
/*  Function:                                                 */
/*  サーボ角度の指定関数                                        */
/*                                                            */
/**************************************************************/
void servo_s_angle_set( u8 u8_angle_idx, u8 servo_num )
{
    u16 u16_angle_duty;

    if( u8_angle_idx >= SERVO_ANGLE_NUM )
    { /* 基本、定義したインデックス以外は使わない予定だが、一応範囲外参照は避けておく。 */
        u8_angle_idx = SERVO_ANGLE_NUM - (u8)1;
    }

    u16_angle_duty = u16_servo_s_angle_set_array[ u8_angle_idx ];


    /* この関数内で使用しているCCPモジュールを指定するのはなんとも微妙な感じ・・・ */
    if( servo_num == SERVO_SHIFT_0 )
    {
        td_g_ccp1_pwm_duty_set( u16_angle_duty );
    }
    else if( servo_num == SERVO_SHIFT_1 )
    {
        td_g_ccp2_pwm_duty_set( u16_angle_duty );
    }
    else if( servo_num == SERVO_SHIFT_2 )
    {
        td_g_ccp3_pwm_duty_set( u16_angle_duty );
    }
    else
    {
        ;       /* ここには来ない予定 */
    }
}



