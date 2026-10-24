// Dual Z bottom switch alignment (Creality K2 main MCU)
//
// Copyright (C) 2026  Jacob10383 <36573259+Jacob10383@users.noreply.github.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "basecmd.h" // oid_alloc
#include "board/gpio.h" // gpio_out_write
#include "board/misc.h" // timer_read_time
#include "command.h" // DECL_COMMAND
#include "compiler.h" // container_of
#include "sched.h" // DECL_TASK
#include "stepper.h" // stepper_sync_dir_pin

// Each Z motor drops onto its own bottom switch, which levels the bed.
// Both sides then back off together and re-approach slowly, twice. A
// re-approach should take exactly the back-off distance; the host reads
// the miss as that switch's repeatability.

#define Z_ALIGN_CHANNELS 2
#define Z_ALIGN_PASSES 2

struct z_align_chan {
    struct gpio_out step, dir;
    struct gpio_in endstop;
    uint32_t pulses, fast, released, slow[Z_ALIGN_PASSES];
    uint16_t filter;
    uint8_t zd_up, zes_untrig, moving, step_level, saved_dir;
};

struct z_align {
    struct timer timer;
    struct z_align_chan chan[Z_ALIGN_CHANNELS];
    uint32_t quick_ticks, slow_ticks, settle_ticks;
    // Distances in halfsteps
    uint32_t retract, safe, fast_max, lift_max;
    uint32_t halfsteps, lift, first_stop;
    uint8_t oid, count, filter_count, phase, pass, stopped, flags;
    uint8_t result_flag, error, result_phase, start_states, end_states;
};

enum {
    ZA_CLEAR = 1, ZA_FAST = 2, ZA_RETRACT = 3, ZA_SLOW = 4,
};

enum {
    ZA_ACTIVE = 1<<0, ZA_REPORT = 1<<1,
};

enum {
    ZA_ERR_NONE = 0,
    ZA_ERR_INVALID_CONFIG = 1,
    ZA_ERR_NOT_RELEASED = 2,
    ZA_ERR_NO_TRIGGER = 3,
    ZA_ERR_TILT = 4,
    ZA_ERR_NO_RETRIGGER = 5,
    ZA_ERR_FORCE_STOP = 6,
};

static struct task_wake z_align_wake;

static uint8_t
z_align_triggered(struct z_align_chan *c)
{
    return gpio_in_read(c->endstop) != c->zes_untrig;
}

static void
z_align_set_dir(struct z_align *za, uint8_t up)
{
    for (uint8_t i = 0; i < za->count; i++) {
        struct z_align_chan *c = &za->chan[i];
        uint8_t val = up ? c->zd_up : !c->zd_up;
        gpio_out_write(c->dir, val);
        stepper_sync_dir_pin(c->dir, val);
    }
}

static void
z_align_step(struct z_align_chan *c)
{
    c->step_level ^= 1;
    gpio_out_write(c->step, c->step_level);
    if (c->step_level)
        c->pulses++;
}

static void
z_align_step_low(struct z_align_chan *c)
{
    c->step_level = 0;
    gpio_out_write(c->step, 0);
}

// End an active run with DIR as the host's step compression last left it
static void
z_align_stop(struct z_align *za)
{
    if (!(za->flags & ZA_ACTIVE))
        return;
    za->flags &= ~ZA_ACTIVE;
    for (uint8_t i = 0; i < za->count; i++) {
        struct z_align_chan *c = &za->chan[i];
        z_align_step_low(c);
        gpio_out_write(c->dir, c->saved_dir);
        stepper_sync_dir_pin(c->dir, c->saved_dir);
    }
}

static void
z_align_report(struct z_align *za, uint8_t flag, uint8_t error)
{
    za->result_phase = za->phase;
    z_align_stop(za);
    za->phase = 0;
    za->result_flag = flag;
    za->error = error;
    za->end_states = 0;
    for (uint8_t i = 0; i < za->count; i++)
        if (z_align_triggered(&za->chan[i]))
            za->end_states |= 1 << i;
    za->flags |= ZA_REPORT;
    sched_wake_task(&z_align_wake);
}

// The first step of a phase comes a settle time after the last stop and
// its DIR change, so the bed is at rest before it moves again
static void
z_align_begin(struct z_align *za, uint8_t phase)
{
    za->phase = phase;
    za->halfsteps = za->stopped = za->first_stop = 0;
    if (phase == ZA_CLEAR)
        za->lift = 0;
    z_align_set_dir(za, phase == ZA_CLEAR || phase == ZA_RETRACT);
    for (uint8_t i = 0; i < za->count; i++) {
        struct z_align_chan *c = &za->chan[i];
        c->pulses = c->filter = 0;
        c->moving = 1;
        if (phase == ZA_RETRACT)
            c->released = 0;
        z_align_step_low(c);
    }
}

// Both sides lift together. Returns the next phase, an error, or zero to
// keep lifting.
static uint8_t
z_align_lift(struct z_align *za, uint8_t *error)
{
    uint8_t all_released = 1;
    for (uint8_t i = 0; i < za->count; i++) {
        struct z_align_chan *c = &za->chan[i];
        if (z_align_triggered(c)) {
            c->filter = 0;
        } else if (c->filter < za->filter_count
                   && ++c->filter >= za->filter_count && !c->released) {
            c->released = za->halfsteps;
        }
        if (c->filter < za->filter_count)
            all_released = 0;
    }
    if (za->phase == ZA_CLEAR) {
        // Lift until every switch releases, then a back-off further
        if (!all_released) {
            za->lift = 0;
            if (za->halfsteps >= za->lift_max) {
                *error = ZA_ERR_NOT_RELEASED;
                return 0;
            }
        } else if (!za->lift) {
            za->lift = za->halfsteps;
        } else if (za->halfsteps - za->lift >= za->retract) {
            return ZA_FAST;
        }
    } else if (za->halfsteps >= za->retract) {
        if (!all_released) {
            *error = ZA_ERR_NOT_RELEASED;
            return 0;
        }
        return ZA_SLOW;
    }
    for (uint8_t i = 0; i < za->count; i++)
        z_align_step(&za->chan[i]);
    za->halfsteps++;
    return 0;
}

// Each side drops until its own switch triggers. Returns nonzero once
// every side has stopped.
static uint8_t
z_align_approach(struct z_align *za, uint8_t *error)
{
    uint8_t moving = 0;
    for (uint8_t i = 0; i < za->count; i++) {
        struct z_align_chan *c = &za->chan[i];
        if (!c->moving)
            continue;
        if (!z_align_triggered(c)) {
            c->filter = 0;
        } else if (++c->filter >= za->filter_count) {
            c->moving = 0;
            z_align_step_low(c);
            if (za->phase == ZA_FAST)
                c->fast = c->pulses;
            else
                c->slow[za->pass] = c->pulses;
            if (!za->stopped++)
                za->first_stop = c->pulses;
            continue;
        }
        moving++;
    }
    if (!moving)
        return 1;
    for (uint8_t i = 0; i < za->count; i++) {
        struct z_align_chan *c = &za->chan[i];
        if (!c->moving)
            continue;
        uint32_t travel = c->pulses * 2;
        if (za->phase == ZA_FAST) {
            if (travel > za->fast_max) {
                *error = ZA_ERR_NO_TRIGGER;
                return 0;
            }
            // A bed that tilted at power off may need one side to drop
            // further, but not without limit
            if (za->stopped && travel - za->first_stop * 2 > za->safe) {
                *error = ZA_ERR_TILT;
                return 0;
            }
        } else if (travel > za->retract * 2) {
            *error = ZA_ERR_NO_RETRIGGER;
            return 0;
        }
        z_align_step(c);
    }
    return 0;
}

static uint_fast8_t
z_align_event(struct timer *timer)
{
    struct z_align *za = container_of(timer, struct z_align, timer);
    if (!(za->flags & ZA_ACTIVE))
        return SF_DONE;

    uint8_t error = ZA_ERR_NONE, next = 0;
    if (za->phase == ZA_CLEAR || za->phase == ZA_RETRACT) {
        next = z_align_lift(za, &error);
    } else if (z_align_approach(za, &error)) {
        if (za->phase == ZA_FAST) {
            za->pass = 0;
            next = ZA_RETRACT;
        } else if (++za->pass < Z_ALIGN_PASSES) {
            next = ZA_RETRACT;
        } else {
            z_align_report(za, 1, ZA_ERR_NONE);
            return SF_DONE;
        }
    }
    if (error) {
        z_align_report(za, 2, error);
        return SF_DONE;
    }
    if (next) {
        z_align_begin(za, next);
        timer->waketime += za->settle_ticks;
        return SF_RESCHEDULE;
    }
    timer->waketime += za->phase == ZA_FAST ? za->quick_ticks : za->slow_ticks;
    return SF_RESCHEDULE;
}

static void
z_align_send_status(struct z_align *za)
{
    struct z_align_chan *c0 = &za->chan[0], *c1 = &za->chan[1];
    sendf("z_align_status oid=%c flag=%c error=%c phase=%c start=%c end=%c"
          " lift=%u fast0=%u fast1=%u released0=%u released1=%u"
          " slow0_0=%u slow0_1=%u slow1_0=%u slow1_1=%u"
          , za->oid, za->result_flag, za->error, za->result_phase
          , za->start_states, za->end_states, za->lift
          , c0->fast, c1->fast, c0->released, c1->released
          , c0->slow[0], c1->slow[0], c0->slow[1], c1->slow[1]);
}

void
command_config_z_align(uint32_t *args)
{
    struct z_align *za = oid_alloc(args[0], command_config_z_align, sizeof(*za));
    za->oid = args[0];
    za->timer.func = z_align_event;
}
DECL_COMMAND(command_config_z_align, "config_z_align oid=%c");

void
command_config_z_align_add(uint32_t *args)
{
    struct z_align *za = oid_lookup(args[0], command_config_z_align);
    uint8_t index = args[1];
    if (index >= Z_ALIGN_CHANNELS)
        shutdown("z_align index too large");
    struct z_align_chan *c = &za->chan[index];
    c->step = gpio_out_setup(args[2], 0);
    c->dir = gpio_out_setup(args[3], args[4]);
    stepper_sync_dir_pin(c->dir, args[4]);
    c->zd_up = args[4];
    c->endstop = gpio_in_setup(args[5], 0);
    c->zes_untrig = args[6];
    if (index >= za->count)
        za->count = index + 1;
}
DECL_COMMAND(command_config_z_align_add,
             "config_z_align_add oid=%c z_indx=%c zs_pin=%u zd_pin=%u"
             " zd_up=%c zes_pin=%u zes_untrig=%c");

void
command_query_z_align(uint32_t *args)
{
    struct z_align *za = oid_lookup(args[0], command_config_z_align);
    sched_del_timer(&za->timer);
    // A resent query restarts the run; snapshot DIR from the host's state
    z_align_stop(za);
    za->flags = 0;
    za->quick_ticks = args[1];
    za->slow_ticks = args[2];
    za->retract = args[3];
    za->safe = args[4];
    za->fast_max = args[5];
    za->lift_max = args[6];
    za->filter_count = args[7];
    za->settle_ticks = args[8] > za->slow_ticks ? args[8] : za->slow_ticks;
    za->lift = za->start_states = za->end_states = za->result_phase = 0;
    za->result_flag = 0;
    za->error = ZA_ERR_NONE;
    for (uint8_t i = 0; i < Z_ALIGN_CHANNELS; i++) {
        struct z_align_chan *c = &za->chan[i];
        c->fast = c->released = c->slow[0] = c->slow[1] = 0;
    }
    if (za->count != Z_ALIGN_CHANNELS || !za->retract || !za->filter_count) {
        z_align_report(za, 2, ZA_ERR_INVALID_CONFIG);
        return;
    }
    for (uint8_t i = 0; i < za->count; i++) {
        struct z_align_chan *c = &za->chan[i];
        c->saved_dir = gpio_out_read(c->dir);
        if (z_align_triggered(c))
            za->start_states |= 1 << i;
    }
    za->flags = ZA_ACTIVE;
    z_align_begin(za, za->start_states ? ZA_CLEAR : ZA_FAST);
    za->timer.waketime = timer_read_time() + za->slow_ticks;
    sched_add_timer(&za->timer);
    z_align_send_status(za);
}
DECL_COMMAND(command_query_z_align,
             "query_z_align oid=%c quick_ticks=%u slow_ticks=%u retract=%u"
             " safe=%u fast_max=%u lift_max=%u filter=%c settle_ticks=%u");

void
command_z_align_force_stop(uint32_t *args)
{
    struct z_align *za = oid_lookup(args[0], command_config_z_align);
    sched_del_timer(&za->timer);
    if (za->flags & ZA_ACTIVE)
        z_align_report(za, 2, ZA_ERR_FORCE_STOP);
}
DECL_COMMAND(command_z_align_force_stop, "z_align_force_stop oid=%c");

void
z_align_task(void)
{
    if (!sched_check_wake(&z_align_wake))
        return;
    uint8_t oid;
    struct z_align *za;
    foreach_oid(oid, za, command_config_z_align) {
        if (!(za->flags & ZA_REPORT))
            continue;
        za->flags &= ~ZA_REPORT;
        z_align_send_status(za);
    }
}
DECL_TASK(z_align_task);

void
z_align_shutdown(void)
{
    uint8_t oid;
    struct z_align *za;
    foreach_oid(oid, za, command_config_z_align) {
        sched_del_timer(&za->timer);
        za->flags = 0;
        for (uint8_t i = 0; i < za->count; i++)
            gpio_out_write(za->chan[i].step, 0);
    }
}
DECL_SHUTDOWN(z_align_shutdown);
