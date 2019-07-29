#ifndef __APP_RADIO_H
#define __APP_RADIO_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {APP_RADIO_MODE_PRX, APP_RADIO_MODE_PTX} app_mode_t;

typedef void (*app_radio_callback_t)(void);

typedef struct
{
    app_mode_t mode;
    app_radio_callback_t on_transmit;
    app_radio_callback_t on_max_retransmit;
    app_radio_callback_t on_receive;
} app_radio_config_t;

uint32_t app_radio_init(app_radio_config_t *config);

void app_radio_print_l01_regs(void);

#endif
