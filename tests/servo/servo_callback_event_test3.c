#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include "pigpiod_if2.h"

static uint32_t rise_tick = 0;    // Pulse rise time tick value
static uint32_t pulse_width = 0;  // Last measured pulse width (us)

// Callback function for measuring PWM input
void pwm_cbfunc(int pi, unsigned user_gpio, unsigned level, uint32_t tick) {
    if (level == 1) {  // rising edge
        rise_tick = tick;
    }
    else if (level == 0) {  // falling edge
        pulse_width = tick - rise_tick;  // TODO: Handle 72 min wrap-around
    }
}

int main(int argc, char **argv)
{
    const unsigned int pwm_in = 18; // GPIO Pin # for PWM in, change as reqd

    int pi = pigpio_start(0, 0);
    if (pi < 0) {
        fprintf(stderr, "pigpio initialization failed (%d)\n", pi);
        return pi;
    }

    // Set up callback for PWM input 
    callback(pi, pwm_in, EITHER_EDGE, pwm_cbfunc);

    while (true) {
        printf("PWM pulse width: %u\n", pulse_width);
        usleep(500000);
    }
}