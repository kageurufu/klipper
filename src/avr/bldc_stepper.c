// Handling of stepper drivers.
//
// Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
// BLDC mod by arsi@arsi.sk
// code based on 
// http://www.berryjam.eu/2015/04/driving-bldc-gimbals-at-super-slow-speeds-with-arduino/
// Arduino UNO Pins
//     Arduino - Port - Klipper
//     D9 - PB1 - 9  >> D3 - PD3 - 27   Defaul port PB1 cannot be used because the timer used for PWM is used by Klipper 
//     D5 - PD5 - 29
//     D6 - PD6 - 30
// EN  D8 - PB0 - 8

#include "autoconf.h" // CONFIG_*
#include "basecmd.h" // oid_alloc
#include "board/gpio.h" // gpio_out_write
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_is_before
#include "command.h" // DECL_COMMAND
#include "sched.h" // struct timer
#include "stepper.h" // stepper_event
#include "trsync.h"
#include "avr/gpio.h" // trsync_add_signal

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

struct gpio_pwm pinU;
struct gpio_pwm pinV;
struct gpio_pwm pinW;

void bldc_init(void) {
    pinU = gpio_pwm_setup(27, 0, 0);
    pinV = gpio_pwm_setup(29, 0, 0);
    pinW = gpio_pwm_setup(30, 0, 0);
}

struct stepper_move {
    struct move_node node;
    uint32_t interval;
    int16_t add;
    uint16_t count;
    uint8_t flags;
};

enum {
    MF_DIR = 1 << 0
};

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
};

enum {
    POSITION_BIAS = 0x40000000
};

enum {
    SF_LAST_DIR = 1 << 0, SF_NEXT_DIR = 1 << 1, SF_INVERT_STEP = 1 << 2, SF_NEED_RESET = 1 << 3,
    SF_SINGLE_SCHED = 1 << 4, SF_HAVE_ADD = 1 << 5
};


// 1° electrical table, 360 elements
const int pwmSin[] = {128, 132, 136, 140, 143, 147, 151, 155, 159, 162, 166, 170, 174, 178, 181, 185, 189, 192, 196, 200, 203, 207, 211, 214, 218, 221, 225, 228, 232, 235, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 248, 249, 250, 250, 251, 252, 252, 253, 253, 253, 254, 254, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 254, 254, 253, 253, 253, 252, 252, 251, 250, 250, 249, 248, 248, 247, 246, 245, 244, 243, 242, 241, 240, 239, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 248, 249, 250, 250, 251, 252, 252, 253, 253, 253, 254, 254, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 254, 254, 253, 253, 253, 252, 252, 251, 250, 250, 249, 248, 248, 247, 246, 245, 244, 243, 242, 241, 240, 239, 238, 235, 232, 228, 225, 221, 218, 214, 211, 207, 203, 200, 196, 192, 189, 185, 181, 178, 174, 170, 166, 162, 159, 155, 151, 147, 143, 140, 136, 132, 128, 124, 120, 116, 113, 109, 105, 101, 97, 94, 90, 86, 82, 78, 75, 71, 67, 64, 60, 56, 53, 49, 45, 42, 38, 35, 31, 28, 24, 21, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 8, 7, 6, 6, 5, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 6, 6, 7, 8, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 8, 7, 6, 6, 5, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 6, 6, 7, 8, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 21, 24, 28, 31, 35, 38, 42, 45, 49, 53, 56, 60, 64, 67, 71, 75, 78, 82, 86, 90, 94, 97, 101, 105, 109, 113, 116, 120, 124};
// 5° electrical table, 72 elements
//const int pwmSin[] = {128, 147, 166, 185, 203, 221, 238, 243, 248, 251, 253, 255, 255, 255, 253, 251, 248, 243, 238, 243, 248, 251, 253, 255, 255, 255, 253, 251, 248, 243, 238, 221, 203, 185, 166, 147, 128, 109, 90, 71, 53, 35, 18, 13, 8, 5, 3, 1, 1, 1, 3, 5, 8, 13, 18, 13, 8, 5, 3, 1, 1, 1, 3, 5, 8, 13, 18, 35, 53, 71, 90, 109};

signed int currentStepA;
signed int currentStepB;
signed int currentStepC;

signed int sineArraySize;
signed int increment = 0;

// Setup a stepper for the next move in its queue

static uint_fast8_t
stepper_load_next(struct stepper *s) {
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
            s->flags = m->add ? s->flags | SF_HAVE_ADD : s->flags & ~SF_HAVE_ADD;
        s->count = m->count;
    } else {
        // It is necessary to schedule unstep events and so there are
        // twice as many events.
        s->next_step_time += m->interval;
        s->time.waketime = s->next_step_time;
        s->count = (uint32_t) m->count * 2;
    }
    // Add all steps to s->position (stepper_get_position() can calc mid-move)
    if (m->flags & MF_DIR) {
        s->position = -s->position + m->count;
        // gpio_out_toggle_noirq(s->dir_pin);
        s->dir = !s->dir;
    } else {
        s->position += m->count;
    }

    move_free(m);
    return SF_RESCHEDULE;
}


#define AVR_STEP_INSNS 40 // minimum instructions between step gpio pulses

// AVR optimized step function

inline static uint_fast8_t
stepper_event_avr(struct timer *t) {
    struct stepper *s = container_of(t, struct stepper, time);


    if (likely(s->dir == 1)) increment = 1;
    else increment = -1;

    currentStepA = currentStepA + increment;
    currentStepB = currentStepB + increment;
    currentStepC = currentStepC + increment;

    //Check for lookup table overflow and return to opposite end if necessary
    if (likely(currentStepA > sineArraySize)) currentStepA = 0;
    if (likely(currentStepA < 0)) currentStepA = sineArraySize;

    if (likely(currentStepB > sineArraySize)) currentStepB = 0;
    if (likely(currentStepB < 0)) currentStepB = sineArraySize;

    if (likely(currentStepC > sineArraySize)) currentStepC = 0;
    if (likely(currentStepC < 0)) currentStepC = sineArraySize;
    gpio_pwm_write(pinU, pwmSin[currentStepA]);
    gpio_pwm_write(pinV, pwmSin[currentStepB]);
    gpio_pwm_write(pinW, pwmSin[currentStepC]);
    //    gpio_out_toggle_noirq(s->step_pin);
    uint16_t *pcount = (void*) &s->count, count = *pcount - 1;
    if (likely(count)) {
        *pcount = count;
        s->time.waketime += s->interval;
        //  gpio_out_toggle_noirq(s->step_pin);
        if (s->flags & SF_HAVE_ADD)
            s->interval += s->add;
        return SF_RESCHEDULE;
    }
    uint_fast8_t ret = stepper_load_next(s);
    // gpio_out_toggle_noirq(s->step_pin);
    return ret;
}



// Optimized entry point for step function (may be inlined into sched.c code)

inline uint_fast8_t
stepper_event(struct timer *t) {
    return stepper_event_avr(t);
}

void
command_config_stepper(uint32_t *args) {
    struct stepper *s = oid_alloc(args[0], command_config_stepper, sizeof (*s));
    int_fast8_t invert_step = args[3];
    s->flags = invert_step > 0 ? SF_INVERT_STEP : 0;
    bldc_init();
    //    s->step_pin = gpio_out_setup(args[1], s->flags & SF_INVERT_STEP);
    //   s->dir_pin = gpio_out_setup(args[2], 0);
    s->dir = 0;
    s->position = -POSITION_BIAS;
    s->step_pulse_ticks = args[4];
    move_queue_setup(&s->mq, sizeof (struct stepper_move));
    if (s->step_pulse_ticks <= AVR_STEP_INSNS)
        s->flags |= SF_SINGLE_SCHED;
    else
        s->time.func = stepper_event_full;
    sineArraySize = sizeof (pwmSin) / sizeof (int); // Find lookup table size
    int phaseShift = sineArraySize / 3; // Find phase shift and initial A, B C phase values
    currentStepA = 0;
    currentStepB = currentStepA + phaseShift;
    currentStepC = currentStepB + phaseShift;
    sineArraySize--;
}
DECL_COMMAND(command_config_stepper, "config_stepper oid=%c step_pin=%c"
        " dir_pin=%c invert_step=%c step_pulse_ticks=%u");

// Return the 'struct stepper' for a given stepper oid

static struct stepper *
stepper_oid_lookup(uint8_t oid) {
    return oid_lookup(oid, command_config_stepper);
}

// Schedule a set of steps with a given timing

void
command_queue_step(uint32_t *args) {
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
command_set_next_step_dir(uint32_t *args) {
    struct stepper *s = stepper_oid_lookup(args[0]);
    uint8_t nextdir = args[1] ? SF_NEXT_DIR : 0;
    irq_disable();
    s->flags = (s->flags & ~SF_NEXT_DIR) | nextdir;
    irq_enable();
}
DECL_COMMAND(command_set_next_step_dir, "set_next_step_dir oid=%c dir=%c");

// Set an absolute time that the next step will be relative to

void
command_reset_step_clock(uint32_t *args) {
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
stepper_get_position(struct stepper *s) {
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
command_stepper_get_position(uint32_t *args) {
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
stepper_stop(struct trsync_signal *tss, uint8_t reason) {
    struct stepper *s = container_of(tss, struct stepper, stop_signal);
    sched_del_timer(&s->time);
    s->next_step_time = s->time.waketime = 0;
    s->position = -stepper_get_position(s);
    s->count = 0;
    s->flags = (s->flags & (SF_INVERT_STEP | SF_SINGLE_SCHED)) | SF_NEED_RESET;
    //   gpio_out_write(s->dir_pin, 0);
    gpio_pwm_write(pinU, 0);
    gpio_pwm_write(pinV, 0);
    gpio_pwm_write(pinW, 0);
    s->dir = 0;
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
command_stepper_stop_on_trigger(uint32_t *args) {
    struct stepper *s = stepper_oid_lookup(args[0]);
    struct trsync *ts = trsync_oid_lookup(args[1]);
    trsync_add_signal(ts, &s->stop_signal, stepper_stop);
}
DECL_COMMAND(command_stepper_stop_on_trigger,
        "stepper_stop_on_trigger oid=%c trsync_oid=%c");

void
stepper_shutdown(void) {
    uint8_t i;
    struct stepper *s;

    foreach_oid(i, s, command_config_stepper) {
        move_queue_clear(&s->mq);
        stepper_stop(&s->stop_signal, 0);
    }
}
DECL_SHUTDOWN(stepper_shutdown);
