// Forward framed motor traffic over USART3 (Creality K2 nozzle MCU)
//
// Copyright (C) 2026  Jacob10383 <36573259+Jacob10383@users.noreply.github.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h> // memcpy
#include "autoconf.h" // CONFIG_STM32_SERIAL_USART3
#include "basecmd.h" // oid_alloc
#include "board/armcm_boot.h" // armcm_enable_irq
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_from_us
#include "command.h" // DECL_COMMAND
#include "compiler.h" // container_of
#include "sched.h" // DECL_TASK
#include "stm32/internal.h" // USART3

#if CONFIG_STM32_SERIAL_USART3 || CONFIG_STM32_SERIAL_USART3_ALT_PD9_PD8
#error "USART3 is used for the host connection"
#endif

#define TRANSPARENT_BAUD 230400
#define TRANSPARENT_BUF_SIZE 60
#define MOTOR_FRAME_HEAD 0xf7
#define MOTOR_FRAME_LEN_MIN 3
// Longer timeouts are waited out in steps that fit the 32-bit timer
#define TIMEOUT_STEP_MS 1000

struct transparent_s {
    struct timer timer;
    uint32_t timeout_ms;
    uint8_t flags, seq;
    uint8_t write_len, write_pos;
    uint8_t read_len, expect_len;
    uint8_t write[TRANSPARENT_BUF_SIZE];
    uint8_t read[TRANSPARENT_BUF_SIZE];
};

enum {
    TF_ACTIVE = 1<<0, TF_DONE = 1<<1, TF_REPORT = 1<<2, TF_TIMER_ARMED = 1<<3,
};

static struct task_wake transparent_wake;
static struct transparent_s *transparent_active;

static void
transparent_drain_rx(void)
{
    while (USART3->SR & (USART_SR_RXNE | USART_SR_ORE)) {
        USART3->SR;
        USART3->DR;
    }
}

// Complete a transfer (from irq context); the reply is kept until the
// next send so that it can be requested again
static void
transparent_finish_irq(struct transparent_s *t)
{
    if (transparent_active == t)
        transparent_active = NULL;
    USART3->CR1 &= ~USART_CR1_TXEIE;
    t->flags = (t->flags & TF_TIMER_ARMED) | TF_DONE | TF_REPORT;
    sched_wake_task(&transparent_wake);
}

static uint8_t
transparent_rx_frame_done(struct transparent_s *t, uint8_t data)
{
    uint8_t read_len = t->read_len;
    if (!read_len && data != MOTOR_FRAME_HEAD)
        return 0;
    if (read_len >= sizeof(t->read)) {
        t->read_len = t->expect_len = 0;
        return 0;
    }
    t->read[read_len++] = data;
    t->read_len = read_len;
    if (read_len == 3) {
        uint8_t declared_len = t->read[2];
        uint16_t expect_len = declared_len + 3;
        if (declared_len < MOTOR_FRAME_LEN_MIN
            || expect_len > sizeof(t->read)) {
            t->read_len = t->expect_len = 0;
            return 0;
        }
        t->expect_len = expect_len;
    }
    return t->expect_len && read_len >= t->expect_len;
}

void
USART3_IRQHandler(void)
{
    uint32_t sr = USART3->SR;
    struct transparent_s *t = transparent_active;
    if (sr & (USART_SR_RXNE | USART_SR_ORE)) {
        uint8_t data = USART3->DR;
        if (t && (sr & USART_SR_RXNE) && transparent_rx_frame_done(t, data))
            transparent_finish_irq(t);
    }
    if ((sr & USART_SR_TXE) && (USART3->CR1 & USART_CR1_TXEIE)) {
        t = transparent_active;
        if (t && t->write_pos < t->write_len)
            USART3->DR = t->write[t->write_pos++];
        else
            USART3->CR1 &= ~USART_CR1_TXEIE;
    }
}

static uint32_t
transparent_next_wait(struct transparent_s *t)
{
    uint32_t ms = t->timeout_ms;
    if (ms > TIMEOUT_STEP_MS)
        ms = TIMEOUT_STEP_MS;
    t->timeout_ms -= ms;
    return timer_from_us(ms * 1000);
}

static uint_fast8_t
transparent_timeout_event(struct timer *timer)
{
    struct transparent_s *t = container_of(timer, struct transparent_s, timer);
    if (t->timeout_ms && (t->flags & TF_ACTIVE)) {
        t->timer.waketime += transparent_next_wait(t);
        return SF_RESCHEDULE;
    }
    t->flags &= ~TF_TIMER_ARMED;
    if (t->flags & TF_ACTIVE)
        transparent_finish_irq(t);
    return SF_DONE;
}

void
command_config_transparent(uint32_t *args)
{
    // Only the default USART3 pins are supported. The host names them so
    // that its pin checks catch a conflicting use.
    if (args[1] != GPIO('B', 11) || args[2] != GPIO('B', 10))
        shutdown("transparent requires rx_pin=PB11 tx_pin=PB10");
    oid_alloc(args[0], command_config_transparent
              , sizeof(struct transparent_s));
    enable_pclock((uint32_t)USART3);
    gpio_peripheral(args[1], GPIO_INPUT, 0);
    gpio_peripheral(args[2], GPIO_FUNCTION(7), 0);
    uint32_t pclk = get_pclock_frequency((uint32_t)USART3);
    uint32_t div = DIV_ROUND_CLOSEST(pclk, TRANSPARENT_BAUD);
    USART3->CR1 = 0;
    USART3->CR2 = 0;
    USART3->CR3 = 0;
    USART3->BRR = (((div / 16) << USART_BRR_DIV_Mantissa_Pos)
                   | ((div % 16) << USART_BRR_DIV_Fraction_Pos));
    USART3->CR1 = (USART_CR1_UE | USART_CR1_RE | USART_CR1_TE
                   | USART_CR1_RXNEIE);
    armcm_enable_irq(USART3_IRQHandler, USART3_IRQn, 0);
    transparent_drain_rx();
}
DECL_COMMAND(command_config_transparent,
             "config_transparent oid=%c rx_pin=%u tx_pin=%u");

// Send a frame and collect the reply; a send while busy is ignored, and
// the host sees no reply with its sequence number
void
command_transparent_send(uint32_t *args)
{
    struct transparent_s *t = oid_lookup(args[0], command_config_transparent);
    if (transparent_active || (t->flags & (TF_ACTIVE | TF_TIMER_ARMED)))
        return;
    uint8_t write_len = args[2];
    if (write_len > sizeof(t->write))
        shutdown("transparent data too large");
    memcpy(t->write, command_decode_ptr(args[3]), write_len);
    t->seq = args[1];
    t->write_len = write_len;
    t->write_pos = t->read_len = t->expect_len = 0;
    t->timer.func = transparent_timeout_event;
    t->timeout_ms = args[4];
    t->timer.waketime = timer_read_time() + transparent_next_wait(t);
    transparent_drain_rx();

    irq_disable();
    t->flags = TF_ACTIVE | TF_TIMER_ARMED;
    transparent_active = t;
    sched_add_timer(&t->timer);
    if (USART3->SR & USART_SR_TXE)
        USART3->DR = t->write[t->write_pos++];
    if (t->write_pos < t->write_len)
        USART3->CR1 |= USART_CR1_TXEIE;
    irq_enable();
}
DECL_COMMAND(command_transparent_send,
             "transparent_send oid=%c seq=%c write=%*s timeout_ms=%u");

// Send the last reply again; the MCU drops a reply when its transmit
// buffer is full, so the host asks again instead of resending the frame
void
command_transparent_query(uint32_t *args)
{
    struct transparent_s *t = oid_lookup(args[0], command_config_transparent);
    irq_disable();
    if ((t->flags & TF_DONE) && t->seq == args[1]) {
        t->flags |= TF_REPORT;
        sched_wake_task(&transparent_wake);
    }
    irq_enable();
}
DECL_COMMAND(command_transparent_query, "transparent_query oid=%c seq=%c");

void
transparent_task(void)
{
    if (!sched_check_wake(&transparent_wake))
        return;
    uint8_t oid;
    struct transparent_s *t;
    foreach_oid(oid, t, command_config_transparent) {
        irq_disable();
        uint8_t flags = t->flags;
        if (flags & TF_REPORT) {
            if (flags & TF_TIMER_ARMED)
                sched_del_timer(&t->timer);
            t->flags = flags & ~(TF_REPORT | TF_TIMER_ARMED);
        }
        irq_enable();
        if (!(flags & TF_REPORT))
            continue;
        sendf("transparent_response oid=%c seq=%c read=%*s"
              , oid, t->seq, t->read_len, t->read);
    }
}
DECL_TASK(transparent_task);

void
transparent_shutdown(void)
{
    uint8_t oid;
    struct transparent_s *t;
    foreach_oid(oid, t, command_config_transparent) {
        t->flags = 0;
    }
    transparent_active = NULL;
}
DECL_SHUTDOWN(transparent_shutdown);
