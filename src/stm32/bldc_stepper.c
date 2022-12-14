// Handling of stepper drivers.
//
// Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
// BLDC mod by arsi@arsi.sk
// code based on 
// http://www.berryjam.eu/2015/04/driving-bldc-gimbals-at-super-slow-speeds-with-arduino/
// Nucleo 32F401 PINS
//     Nucleo - Port - Klipper
//     D3 - PB3 - 19   Defaul port PB1 cannot be used because the timer used for PWM is used by Klipper 
//     D5 - PB4 - 20
//     D6 - PB10 - 26
// EN  D8 - PA9 - 9

#include "autoconf.h" // CONFIG_*
#include "basecmd.h" // oid_alloc
#include "board/gpio.h" // gpio_out_write
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_is_before
#include "command.h" // DECL_COMMAND
#include "sched.h" // struct timer
#include "stepper.h" // stepper_event
#include "trsync.h"
#include "stm32/gpio.h" // trsync_add_signal
#include "board/misc.h" // output

#if CONFIG_INLINE_STEPPER_HACK && CONFIG_HAVE_STEPPER_BOTH_EDGE
#define HAVE_SINGLE_SCHEDULE 1
#define HAVE_EDGE_OPTIMIZATION 1
#define HAVE_AVR_OPTIMIZATION 0
DECL_CONSTANT("STEPPER_BOTH_EDGE", 1);
#elif CONFIG_INLINE_STEPPER_HACK && CONFIG_MACH_AVR
#define HAVE_SINGLE_SCHEDULE 1
#define HAVE_EDGE_OPTIMIZATION 0
#define HAVE_AVR_OPTIMIZATION 1
#else
#define HAVE_SINGLE_SCHEDULE 0
#define HAVE_EDGE_OPTIMIZATION 0
#define HAVE_AVR_OPTIMIZATION 0
#endif



#define PWM_VALUE  84000000 / (CONFIG_BLDC_FREQ*1000)



struct stepper_move {
    struct move_node node;
    uint32_t interval;
    int16_t add;
    uint16_t count;
    uint8_t flags;
};

enum { MF_DIR=1<<0 };

struct stepper {
    struct timer time;
    uint32_t interval;
    int16_t add;
    uint32_t count;
    uint32_t next_step_time, step_pulse_ticks;
    struct gpio_out step_pin, dir_pin;
    uint32_t position;
    struct move_queue_head mq;
    struct trsync_signal stop_signal;
    // gcc (pre v6) does better optimization when uint8_t are bitfields
    uint8_t flags : 8;
    uint8_t dir;
    struct gpio_pwm pinU;
    struct gpio_pwm pinV;
    struct gpio_pwm pinW;
    uint32_t pin;
    signed int currentStepA;
    signed int currentStepB;
    signed int currentStepC;
    signed int increment;
    int power_full;
    int power_holding;
};

signed int sineArraySize;

enum { POSITION_BIAS=0x40000000 };

enum {
    SF_LAST_DIR=1<<0, SF_NEXT_DIR=1<<1, SF_INVERT_STEP=1<<2, SF_NEED_RESET=1<<3,
    SF_SINGLE_SCHED=1<<4, SF_HAVE_ADD=1<<5
};


// 2° electrical table, 120 elements
//const int pwmSin[]={2048,2155,2262,2369,2475,2581,2687,2791,2895,2997,3098,3199,3297,3394,3490,3584,3614,3642,3668,3692,3714,3734,3752,3769,3782,3794,3804,3811,3817,3820,3821,3820,3817,3811,3804,3794,3782,3769,3752,3734,3714,3692,3668,3642,3614,3584,3614,3642,3668,3692,3714,3734,3752,3769,3782,3794,3804,3811,3817,3820,3821,3820,3817,3811,3804,3794,3782,3769,3752,3734,3714,3692,3668,3642,3614,3584,3490,3394,3297,3199,3098,2997,2895,2791,2687,2581,2475,2369,2262,2155,2048,1941,1834,1727,1621,1515,1409,1305,1201,1099,998,897,799,702,606,512,482,454,428,404,382,362,344,327,314,302,292,285,279,276,275,276,279,285,292,302,314,327,344,362,382,404,428,454,482,512,482,454,428,404,382,362,344,327,314,302,292,285,279,276,275,276,279,285,292,302,314,327,344,362,382,404,428,454,482,512,606,702,799,897,998,1099,1201,1305,1409,1515,1621,1727,1834,1941,};
// 1° electrical table, 360 elements
const int pwmSin[] = {2048, 2102, 2155, 2209, 2262, 2316, 2369, 2422, 2475, 2528, 2581, 2634, 2687, 2739, 2791, 2843, 2895, 2946, 2997, 3048, 3098, 3149, 3199, 3248, 3297, 3346, 3394, 3442, 3490, 3537, 3584, 3599, 3614, 3628, 3642, 3655, 3668, 3680, 3692, 3703, 3714, 3725, 3734, 3744, 3752, 3761, 3769, 3776, 3782, 3789, 3794, 3799, 3804, 3808, 3811, 3814, 3817, 3819, 3820, 3821, 3821, 3821, 3820, 3819, 3817, 3814, 3811, 3808, 3804, 3799, 3794, 3789, 3782, 3776, 3769, 3761, 3752, 3744, 3734, 3725, 3714, 3703, 3692, 3680, 3668, 3655, 3642, 3628, 3614, 3599, 3584, 3599, 3614, 3628, 3642, 3655, 3668, 3680, 3692, 3703, 3714, 3725, 3734, 3744, 3752, 3761, 3769, 3776, 3782, 3789, 3794, 3799, 3804, 3808, 3811, 3814, 3817, 3819, 3820, 3821, 3821, 3821, 3820, 3819, 3817, 3814, 3811, 3808, 3804, 3799, 3794, 3789, 3782, 3776, 3769, 3761, 3752, 3744, 3734, 3725, 3714, 3703, 3692, 3680, 3668, 3655, 3642, 3628, 3614, 3599, 3584, 3537, 3490, 3442, 3394, 3346, 3297, 3248, 3199, 3149, 3098, 3048, 2997, 2946, 2895, 2843, 2791, 2739, 2687, 2634, 2581, 2528, 2475, 2422, 2369, 2316, 2262, 2209, 2155, 2102, 2048, 1994, 1941, 1887, 1834, 1780, 1727, 1674, 1621, 1568, 1515, 1462, 1409, 1357, 1305, 1253, 1201, 1150, 1099, 1048, 998, 947, 897, 848, 799, 750, 702, 654, 606, 559, 512, 497, 482, 468, 454, 441, 428, 416, 404, 393, 382, 371, 362, 352, 344, 335, 327, 320, 314, 307, 302, 297, 292, 288, 285, 282, 279, 277, 276, 275, 275, 275, 276, 277, 279, 282, 285, 288, 292, 297, 302, 307, 314, 320, 327, 335, 344, 352, 362, 371, 382, 393, 404, 416, 428, 441, 454, 468, 482, 497, 512, 497, 482, 468, 454, 441, 428, 416, 404, 393, 382, 371, 362, 352, 344, 335, 327, 320, 314, 307, 302, 297, 292, 288, 285, 282, 279, 277, 276, 275, 275, 275, 276, 277, 279, 282, 285, 288, 292, 297, 302, 307, 314, 320, 327, 335, 344, 352, 362, 371, 382, 393, 404, 416, 428, 441, 454, 468, 482, 497, 512, 559, 606, 654, 702, 750, 799, 848, 897, 947, 998, 1048, 1099, 1150, 1201, 1253, 1305, 1357, 1409, 1462, 1515, 1568, 1621, 1674, 1727, 1780, 1834, 1887, 1941, 1994};
// 0.5° electrical table, 720 elements
//const int pwmSin[]={2048,2075,2102,2128,2155,2182,2209,2235,2262,2289,2316,2342,2369,2396,2422,2449,2475,2502,2528,2555,2581,2608,2634,2660,2687,2713,2739,2765,2791,2817,2843,2869,2895,2920,2946,2972,2997,3023,3048,3073,3098,3124,3149,3174,3199,3223,3248,3273,3297,3322,3346,3370,3394,3418,3442,3466,3490,3513,3537,3560,3584,3591,3599,3606,3614,3621,3628,3635,3642,3648,3655,3662,3668,3674,3680,3686,3692,3698,3703,3709,3714,3719,3725,3730,3734,3739,3744,3748,3752,3757,3761,3765,3769,3772,3776,3779,3782,3786,3789,3791,3794,3797,3799,3802,3804,3806,3808,3810,3811,3813,3814,3816,3817,3818,3819,3819,3820,3821,3821,3821,3821,3821,3821,3821,3820,3819,3819,3818,3817,3816,3814,3813,3811,3810,3808,3806,3804,3802,3799,3797,3794,3791,3789,3786,3782,3779,3776,3772,3769,3765,3761,3757,3752,3748,3744,3739,3734,3730,3725,3719,3714,3709,3703,3698,3692,3686,3680,3674,3668,3662,3655,3648,3642,3635,3628,3621,3614,3606,3599,3591,3584,3591,3599,3606,3614,3621,3628,3635,3642,3648,3655,3662,3668,3674,3680,3686,3692,3698,3703,3709,3714,3719,3725,3730,3734,3739,3744,3748,3752,3757,3761,3765,3769,3772,3776,3779,3782,3786,3789,3791,3794,3797,3799,3802,3804,3806,3808,3810,3811,3813,3814,3816,3817,3818,3819,3819,3820,3821,3821,3821,3821,3821,3821,3821,3820,3819,3819,3818,3817,3816,3814,3813,3811,3810,3808,3806,3804,3802,3799,3797,3794,3791,3789,3786,3782,3779,3776,3772,3769,3765,3761,3757,3752,3748,3744,3739,3734,3730,3725,3719,3714,3709,3703,3698,3692,3686,3680,3674,3668,3662,3655,3648,3642,3635,3628,3621,3614,3606,3599,3591,3584,3560,3537,3513,3490,3466,3442,3418,3394,3370,3346,3322,3297,3273,3248,3223,3199,3174,3149,3124,3098,3073,3048,3023,2997,2972,2946,2920,2895,2869,2843,2817,2791,2765,2739,2713,2687,2660,2634,2608,2581,2555,2528,2502,2475,2449,2422,2396,2369,2342,2316,2289,2262,2235,2209,2182,2155,2128,2102,2075,2048,2021,1994,1968,1941,1914,1887,1861,1834,1807,1780,1754,1727,1700,1674,1647,1621,1594,1568,1541,1515,1488,1462,1436,1409,1383,1357,1331,1305,1279,1253,1227,1201,1176,1150,1124,1099,1073,1048,1023,998,972,947,922,897,873,848,823,799,774,750,726,702,678,654,630,606,583,559,536,512,505,497,490,482,475,468,461,454,448,441,434,428,422,416,410,404,398,393,387,382,377,371,366,362,357,352,348,344,339,335,331,327,324,320,317,314,310,307,305,302,299,297,294,292,290,288,286,285,283,282,280,279,278,277,277,276,275,275,275,275,275,275,275,276,277,277,278,279,280,282,283,285,286,288,290,292,294,297,299,302,305,307,310,314,317,320,324,327,331,335,339,344,348,352,357,362,366,371,377,382,387,393,398,404,410,416,422,428,434,441,448,454,461,468,475,482,490,497,505,512,505,497,490,482,475,468,461,454,448,441,434,428,422,416,410,404,398,393,387,382,377,371,366,362,357,352,348,344,339,335,331,327,324,320,317,314,310,307,305,302,299,297,294,292,290,288,286,285,283,282,280,279,278,277,277,276,275,275,275,275,275,275,275,276,277,277,278,279,280,282,283,285,286,288,290,292,294,297,299,302,305,307,310,314,317,320,324,327,331,335,339,344,348,352,357,362,366,371,377,382,387,393,398,404,410,416,422,428,434,441,448,454,461,468,475,482,490,497,505,512,536,559,583,606,630,654,678,702,726,750,774,799,823,848,873,897,922,947,972,998,1023,1048,1073,1099,1124,1150,1176,1201,1227,1253,1279,1305,1331,1357,1383,1409,1436,1462,1488,1515,1541,1568,1594,1621,1647,1674,1700,1727,1754,1780,1807,1834,1861,1887,1914,1941,1968,1994,2021};
static uint16_t pwmSinOut0[sizeof(pwmSin)/sizeof(int)];
static uint16_t pwmSinOut1[sizeof(pwmSin)/sizeof(int)];
static uint16_t pwmSinOut2[sizeof(pwmSin)/sizeof(int)];
//const int pwmSin[] = {128, 132, 136, 140, 143, 147, 151, 155, 159, 162, 166, 170, 174, 178, 181, 185, 189, 192, 196, 200, 203, 207, 211, 214, 218, 221, 225, 228, 232, 235, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 248, 249, 250, 250, 251, 252, 252, 253, 253, 253, 254, 254, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 254, 254, 253, 253, 253, 252, 252, 251, 250, 250, 249, 248, 248, 247, 246, 245, 244, 243, 242, 241, 240, 239, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 248, 249, 250, 250, 251, 252, 252, 253, 253, 253, 254, 254, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 254, 254, 253, 253, 253, 252, 252, 251, 250, 250, 249, 248, 248, 247, 246, 245, 244, 243, 242, 241, 240, 239, 238, 235, 232, 228, 225, 221, 218, 214, 211, 207, 203, 200, 196, 192, 189, 185, 181, 178, 174, 170, 166, 162, 159, 155, 151, 147, 143, 140, 136, 132, 128, 124, 120, 116, 113, 109, 105, 101, 97, 94, 90, 86, 82, 78, 75, 71, 67, 64, 60, 56, 53, 49, 45, 42, 38, 35, 31, 28, 24, 21, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 8, 7, 6, 6, 5, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 6, 6, 7, 8, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 8, 7, 6, 6, 5, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 6, 6, 7, 8, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 21, 24, 28, 31, 35, 38, 42, 45, 49, 53, 56, 60, 64, 67, 71, 75, 78, 82, 86, 90, 94, 97, 101, 105, 109, 113, 116, 120, 124};
// 5° electrical table, 72 elements
//const int pwmSin[] = {128, 147, 166, 185, 203, 221, 238, 243, 248, 251, 253, 255, 255, 255, 253, 251, 248, 243, 238, 243, 248, 251, 253, 255, 255, 255, 253, 251, 248, 243, 238, 221, 203, 185, 166, 147, 128, 109, 90, 71, 53, 35, 18, 13, 8, 5, 3, 1, 1, 1, 3, 5, 8, 13, 18, 13, 8, 5, 3, 1, 1, 1, 3, 5, 8, 13, 18, 35, 53, 71, 90, 109};

extern void gpio_pwm_start(uint8_t pin1, uint8_t pin2, uint8_t pin3);

void bldc_init(struct stepper *s, uint32_t pin) {
    s->increment = 0;
    if (pin == CONFIG_BLDC_PWM0A) {
        s->pin = CONFIG_BLDC_PWM0A;
        s->pinU = gpio_pwm_setup(CONFIG_BLDC_PWM0A, PWM_VALUE, 0);
        s->pinV = gpio_pwm_setup(CONFIG_BLDC_PWM0B, PWM_VALUE, 0);
        s->pinW = gpio_pwm_setup(CONFIG_BLDC_PWM0C, PWM_VALUE, 0);
        s->power_full = (float) (CONFIG_BLDC_POWER0 * 2.55);
        s->power_holding = (float) (CONFIG_BLDC_POWER0_HOLDING * 2.55);
        int i;
        for (i = 0; i < sizeof(pwmSin)/sizeof(int); i++) {
            float f = (float)(pwmSin[i] * (s->power_full / 255.0));
           pwmSinOut0[i] = f;
        }

        gpio_pwm_start(CONFIG_BLDC_PWM0A, CONFIG_BLDC_PWM0B, CONFIG_BLDC_PWM0C);
    } else if (pin == CONFIG_BLDC_PWM1A) {
        s->pin = CONFIG_BLDC_PWM1A;
        s->pinU = gpio_pwm_setup(CONFIG_BLDC_PWM1A, PWM_VALUE, 0);
        s->pinV = gpio_pwm_setup(CONFIG_BLDC_PWM1B, PWM_VALUE, 0);
        s->pinW = gpio_pwm_setup(CONFIG_BLDC_PWM1C, PWM_VALUE, 0);
        s->power_full = (float) (CONFIG_BLDC_POWER1 * 2.55);
        s->power_holding = (float) (CONFIG_BLDC_POWER1_HOLDING * 2.55);
        int i;
        for (i = 0; i < sizeof(pwmSin)/sizeof(int); i++) {
            float f = (float)(pwmSin[i] * (s->power_full / 255.0));
           pwmSinOut1[i] = f;
        }
        gpio_pwm_start(CONFIG_BLDC_PWM1A, CONFIG_BLDC_PWM1B, CONFIG_BLDC_PWM1C);
    } else if (pin == CONFIG_BLDC_PWM2A) {
        s->pin = CONFIG_BLDC_PWM2A;
        s->pinU = gpio_pwm_setup(CONFIG_BLDC_PWM2A, PWM_VALUE, 0);
        s->pinV = gpio_pwm_setup(CONFIG_BLDC_PWM2B, PWM_VALUE, 0);
        s->pinW = gpio_pwm_setup(CONFIG_BLDC_PWM2C, PWM_VALUE, 0);
        s->power_full = (float) (CONFIG_BLDC_POWER1 * 2.55);
        s->power_holding = (float) (CONFIG_BLDC_POWER1_HOLDING * 2.55);
        int i;
        for (i = 0; i < sizeof(pwmSin)/sizeof(int); i++) {
            float f = (float)(pwmSin[i] * (s->power_full / 255.0));
           pwmSinOut2[i] = f;
        }
        gpio_pwm_start(CONFIG_BLDC_PWM2A, CONFIG_BLDC_PWM2B, CONFIG_BLDC_PWM2C);
    } else {
        shutdown("BLDC stepper pin not found in PWM A pins");
    }

}

// Setup a stepper for the next move in its queue
static uint_fast8_t
stepper_load_next(struct stepper *s)
{
    if (move_queue_empty(&s->mq)) {
        // There is no next move - the queue is empty
        s->count = 0;
        return SF_DONE;
    }

    // Load next 'struct stepper_move' into 'struct stepper'
    struct move_node *mn = move_queue_pop(&s->mq);
    struct stepper_move *m = container_of(mn, struct stepper_move, node);
    s->add = m->add;
    s->interval = m->interval + m->add;
    if (HAVE_SINGLE_SCHEDULE && s->flags & SF_SINGLE_SCHED) {
        s->time.waketime += m->interval;
        if (HAVE_AVR_OPTIMIZATION)
            s->flags = m->add ? s->flags|SF_HAVE_ADD : s->flags & ~SF_HAVE_ADD;
        s->count = m->count;
    } else {
        // It is necessary to schedule unstep events and so there are
        // twice as many events.
        s->next_step_time += m->interval;
        s->time.waketime = s->next_step_time;
        s->count = (uint32_t)m->count * 2;
    }
    // Add all steps to s->position (stepper_get_position() can calc mid-move)
    if (m->flags & MF_DIR) {
        s->position = -s->position + m->count;
        // gpio_out_toggle_noirq(s->dir_pin);
        s->dir=!s->dir;
    } else {
        s->position += m->count;
    }

    move_free(m);
    return SF_RESCHEDULE;
}


#define AVR_STEP_INSNS 40 // minimum instructions between step gpio pulses

void setMotorPosition(struct stepper *s,float power){
    gpio_pwm_write(s->pinU,(float) (pwmSin[s->currentStepA]* (power / 255.0)));
    gpio_pwm_write(s->pinV, (float) (pwmSin[s->currentStepB]* (power / 255.0)));
    gpio_pwm_write(s->pinW, (float) (pwmSin[s->currentStepC]* (power / 255.0)));
}

void setMotorPositionFull(struct stepper *s) {
    if (s->pin == CONFIG_BLDC_PWM0A) {
        gpio_pwm_write(s->pinU, pwmSinOut0[s->currentStepA]);
        gpio_pwm_write(s->pinV, pwmSinOut0[s->currentStepB]);
        gpio_pwm_write(s->pinW, pwmSinOut0[s->currentStepC]);
    } else if (s->pin == CONFIG_BLDC_PWM1A) {
        gpio_pwm_write(s->pinU, pwmSinOut1[s->currentStepA]);
        gpio_pwm_write(s->pinV, pwmSinOut1[s->currentStepB]);
        gpio_pwm_write(s->pinW, pwmSinOut1[s->currentStepC]);
    } else if (s->pin == CONFIG_BLDC_PWM2A) {
        gpio_pwm_write(s->pinU, pwmSinOut2[s->currentStepA]);
        gpio_pwm_write(s->pinV, pwmSinOut2[s->currentStepB]);
        gpio_pwm_write(s->pinW, pwmSinOut2[s->currentStepC]);
    }
}


// AVR optimized step function
inline static uint_fast8_t
stepper_event_avr(struct timer *t) {
    struct stepper *s = container_of(t, struct stepper, time);
    if (likely(s->dir == 1)) s->increment = 1;
    else s->increment = -1;

    s->currentStepA = s->currentStepA + s->increment;
    s->currentStepB = s->currentStepB + s->increment;
    s->currentStepC = s->currentStepC + s->increment;

    //Check for lookup table overflow and return to opposite end if necessary
    if (likely(s->currentStepA > sineArraySize)) s->currentStepA = 0;
    if (likely(s->currentStepA < 0)) s->currentStepA = sineArraySize;

    if (likely(s->currentStepB > sineArraySize)) s->currentStepB = 0;
    if (likely(s->currentStepB < 0)) s->currentStepB = sineArraySize;

    if (likely(s->currentStepC > sineArraySize)) s->currentStepC = 0;
    if (likely(s->currentStepC < 0)) s->currentStepC = sineArraySize;
    setMotorPositionFull(s);
        uint16_t *pcount = (void*)&s->count, count = *pcount - 1;
    if (likely(count)) {
        *pcount = count;
        s->time.waketime += s->interval;
        //  gpio_out_toggle_noirq(s->step_pin);
        if (s->flags & SF_HAVE_ADD)
            s->interval += s->add;
        return SF_RESCHEDULE;
    }
    uint_fast8_t ret = stepper_load_next(s);
    if(ret==SF_DONE){
        setMotorPosition(s,s->power_holding);
    }
    return ret;
}



// Optimized entry point for step function (may be inlined into sched.c code)
inline uint_fast8_t
stepper_event(struct timer *t)
{
    return stepper_event_avr(t);
}



void
command_config_stepper(uint32_t *args)
{
    struct stepper *s = oid_alloc(args[0], command_config_stepper, sizeof(*s));
    int_fast8_t invert_step = args[3];
    s->flags = invert_step > 0 ? SF_INVERT_STEP : 0;
    bldc_init(s,args[1]);
    //    s->step_pin = gpio_out_setup(args[1], s->flags & SF_INVERT_STEP);
    //   s->dir_pin = gpio_out_setup(args[2], 0);
    s->dir=0;
    s->position = -POSITION_BIAS;
    s->step_pulse_ticks = args[4];
    move_queue_setup(&s->mq, sizeof(struct stepper_move));
    s->flags |= SF_SINGLE_SCHED;
    sineArraySize = sizeof (pwmSin) / sizeof (int); // Find lookup table size
    int phaseShift = sineArraySize / 3; // Find phase shift and initial A, B C phase values
    s->currentStepA = 0;
    s->currentStepB = s->currentStepA + phaseShift;
    s->currentStepC = s->currentStepB + phaseShift;
    sineArraySize--;
}
DECL_COMMAND(command_config_stepper, "config_stepper oid=%c step_pin=%c"
        " dir_pin=%c invert_step=%c step_pulse_ticks=%u");

// Return the 'struct stepper' for a given stepper oid
static struct stepper *
stepper_oid_lookup(uint8_t oid)
{
    return oid_lookup(oid, command_config_stepper);
}

// Schedule a set of steps with a given timing
void
command_queue_step(uint32_t *args)
{
    struct stepper *s = stepper_oid_lookup(args[0]);
    struct stepper_move *m = move_alloc();
    m->interval = args[1];
    m->count = args[2];
    if (!m->count)
        shutdown("Invalid count parameter");
    m->add = args[3];
    m->flags = 0;

    irq_disable();
    uint8_t flags = s->flags;
    if (!!(flags & SF_LAST_DIR) != !!(flags & SF_NEXT_DIR)) {
        flags ^= SF_LAST_DIR;
        m->flags |= MF_DIR;
    }
    if (s->count) {
        s->flags = flags;
        move_queue_push(&m->node, &s->mq);
    } else if (flags & SF_NEED_RESET) {
        move_free(m);
    } else {
        s->flags = flags;
        move_queue_push(&m->node, &s->mq);
        stepper_load_next(s);
        sched_add_timer(&s->time);
    }
    irq_enable();
}
DECL_COMMAND(command_queue_step,
        "queue_step oid=%c interval=%u count=%hu add=%hi");

// Set the direction of the next queued step
void
command_set_next_step_dir(uint32_t *args)
{
    struct stepper *s = stepper_oid_lookup(args[0]);
    uint8_t nextdir = args[1] ? SF_NEXT_DIR : 0;
    irq_disable();
    s->flags = (s->flags & ~SF_NEXT_DIR) | nextdir;
    irq_enable();
}
DECL_COMMAND(command_set_next_step_dir, "set_next_step_dir oid=%c dir=%c");

// Set an absolute time that the next step will be relative to
void
command_reset_step_clock(uint32_t *args)
{
    struct stepper *s = stepper_oid_lookup(args[0]);
    uint32_t waketime = args[1];
    irq_disable();
    if (s->count)
        shutdown("Can't reset time when stepper active");
    s->next_step_time = s->time.waketime = waketime;
    s->flags &= ~SF_NEED_RESET;
    irq_enable();
}
DECL_COMMAND(command_reset_step_clock, "reset_step_clock oid=%c clock=%u");

// Return the current stepper position.  Caller must disable irqs.
static uint32_t
stepper_get_position(struct stepper *s)
{
    uint32_t position = s->position;
    // If stepper is mid-move, subtract out steps not yet taken
    if (HAVE_SINGLE_SCHEDULE && s->flags & SF_SINGLE_SCHED)
        position -= s->count;
    else
        position -= s->count / 2;
    // The top bit of s->position is an optimized reverse direction flag
    if (position & 0x80000000)
        return -position;
    return position;
}

// Report the current position of the stepper
void
command_stepper_get_position(uint32_t *args)
{
    uint8_t oid = args[0];
    struct stepper *s = stepper_oid_lookup(oid);
    irq_disable();
    uint32_t position = stepper_get_position(s);
    irq_enable();
    sendf("stepper_position oid=%c pos=%i", oid, position - POSITION_BIAS);
}
DECL_COMMAND(command_stepper_get_position, "stepper_get_position oid=%c");

// Stop all moves for a given stepper (caller must disable IRQs)
static void
stepper_stop(struct trsync_signal *tss, uint8_t reason)
{
    struct stepper *s = container_of(tss, struct stepper, stop_signal);
    sched_del_timer(&s->time);
    s->next_step_time = s->time.waketime = 0;
    s->position = -stepper_get_position(s);
    s->count = 0;
    s->flags = (s->flags & (SF_INVERT_STEP|SF_SINGLE_SCHED)) | SF_NEED_RESET;
    //   gpio_out_write(s->dir_pin, 0);
    gpio_pwm_write(s->pinU,0);
    gpio_pwm_write(s->pinV,0);
    gpio_pwm_write(s->pinW,0);
    s->dir=0;
    //    if (!(HAVE_EDGE_OPTIMIZATION && s->flags & SF_SINGLE_SCHED))
    //        gpio_out_write(s->step_pin, s->flags & SF_INVERT_STEP);
    while (!move_queue_empty(&s->mq)) {
        struct move_node *mn = move_queue_pop(&s->mq);
        struct stepper_move *m = container_of(mn, struct stepper_move, node);
        move_free(m);
    }
}

// Set the stepper to stop on a "trigger event" (used in homing)
void
command_stepper_stop_on_trigger(uint32_t *args)
{
    struct stepper *s = stepper_oid_lookup(args[0]);
    struct trsync *ts = trsync_oid_lookup(args[1]);
    trsync_add_signal(ts, &s->stop_signal, stepper_stop);
}
DECL_COMMAND(command_stepper_stop_on_trigger,
        "stepper_stop_on_trigger oid=%c trsync_oid=%c");

void
stepper_shutdown(void)
{
    uint8_t i;
    struct stepper *s;
    foreach_oid(i, s, command_config_stepper) {
        move_queue_clear(&s->mq);
        stepper_stop(&s->stop_signal, 0);
    }
}
DECL_SHUTDOWN(stepper_shutdown);
