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
#include "rp2040/gpio.h" // trsync_add_signal
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



#define PWM_VALUE  125000000 / (CONFIG_BLDC_FREQ*1000)



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


// // 1° electrical table, 360 elements,3174 range, 19.691kHz
const int pwmSin[] = {1701,1745,1789,1834,1878,1923,1967,2011,2055,2099,2143,2187,2231,2274,2317,2360,2403,2446,2488,2531,2573,2614,2656,2697,2738,2778,2818,2858,2898,2937,2976,2988,3000,3012,3024,3035,3045,3056,3066,3075,3084,3093,3101,3108,3116,3123,3129,3135,3141,3146,3150,3155,3158,3162,3165,3167,3169,3171,3172,3173,3173,3173,3172,3171,3169,3167,3165,3162,3158,3155,3150,3146,3141,3135,3129,3123,3116,3108,3101,3093,3084,3075,3066,3056,3045,3035,3024,3012,3000,2988,2976,2988,3000,3012,3024,3035,3045,3056,3066,3075,3084,3093,3101,3108,3116,3123,3129,3135,3141,3146,3150,3155,3158,3162,3165,3167,3169,3171,3172,3173,3173,3173,3172,3171,3169,3167,3165,3162,3158,3155,3150,3146,3141,3135,3129,3123,3116,3108,3101,3093,3084,3075,3066,3056,3045,3035,3024,3012,3000,2988,2976,2937,2898,2858,2818,2778,2738,2697,2656,2614,2573,2531,2488,2446,2403,2360,2317,2274,2231,2187,2143,2099,2055,2011,1967,1923,1878,1834,1789,1745,1701,1656,1612,1567,1523,1478,1434,1390,1346,1302,1258,1214,1170,1127,1084,1041,998,955,913,870,828,787,745,704,663,623,583,543,503,464,426,413,401,389,377,366,356,345,335,326,317,308,300,293,285,278,272,266,260,255,251,246,243,239,236,234,232,230,229,228,228,228,229,230,232,234,236,239,243,246,251,255,260,266,272,278,285,293,300,308,317,326,335,345,356,366,377,389,401,413,426,413,401,389,377,366,356,345,335,326,317,308,300,293,285,278,272,266,260,255,251,246,243,239,236,234,232,230,229,228,228,228,229,230,232,234,236,239,243,246,251,255,260,266,272,278,285,293,300,308,317,326,335,345,356,366,377,389,401,413,426,464,503,543,583,623,663,704,745,787,828,870,913,955,998,1041,1084,1127,1170,1214,1258,1302,1346,1390,1434,1478,1523,1567,1612,1656,};
static uint16_t pwmSinOut0[sizeof(pwmSin)/sizeof(int)];

extern struct gpio_pwm gpio_pwm_setup_bldc(uint8_t pin);
extern void gpio_pwm_start(uint8_t pin1, uint8_t pin2, uint8_t pin3);

void bldc_init(struct stepper *s, uint32_t pin) {
    s->increment = 0;
    if (pin == CONFIG_BLDC_PWM0A) {
        s->pin = CONFIG_BLDC_PWM0A;
        s->pinU = gpio_pwm_setup_bldc(CONFIG_BLDC_PWM0A);
        s->pinV = gpio_pwm_setup_bldc(CONFIG_BLDC_PWM0B);
        s->pinW = gpio_pwm_setup_bldc(CONFIG_BLDC_PWM0C);
        s->power_full = (float) (CONFIG_BLDC_POWER0 * 2.55);
        s->power_holding = (float) (CONFIG_BLDC_POWER0_HOLDING * 2.55);
        int i;
        for (i = 0; i < sizeof(pwmSin)/sizeof(int); i++) {
            float f = (float)(pwmSin[i] * (s->power_full / 255.0));
           pwmSinOut0[i] = f;
        }

        gpio_pwm_start(CONFIG_BLDC_PWM0A, CONFIG_BLDC_PWM0B, CONFIG_BLDC_PWM0C);
    
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
