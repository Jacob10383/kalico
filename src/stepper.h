#ifndef __STEPPER_H
#define __STEPPER_H

#include <stdint.h> // uint8_t
#include "board/gpio.h" // struct gpio_out

uint_fast8_t stepper_event(struct timer *t);
void stepper_sync_dir_pin(struct gpio_out dir, uint8_t val);

#endif // stepper.h
