/**
 * Copyright (c) 2015 - 2018, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "sdk_common.h"
#include "nrf.h"
#include "nrf_error.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_delay.h"
#include "app_util.h"
#include "nrf5dk_l01.h"
#include "app_radio.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

const uint8_t leds_list[LEDS_NUMBER] = LEDS_LIST;

static void clocks_start( void )
{
    NRF_CLOCK->TASKS_LFCLKSTART = 1;
    
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
}

static void gpio_init()
{
    for(int i = 0; i < (sizeof(leds_list) / sizeof(leds_list[0])); i++)
    {
        nrf_gpio_cfg_output(leds_list[i]);
    }
}

void on_transmit(void)
{
    NRF_LOG_INFO("On TX");
}

void on_max_retransmit(void)
{
    NRF_LOG_INFO("On Max RT");
}

void on_receive(void)
{
    NRF_LOG_INFO("On RX");
}

static void l01_configure(void)
{
    app_radio_config_t radio_config = {0};
    radio_config.mode = APP_RADIO_MODE_PTX;
    radio_config.on_transmit = on_transmit;
    radio_config.on_max_retransmit = on_max_retransmit;
    radio_config.on_receive = on_receive;
    app_radio_init(&radio_config);
}

int main(void)
{
    ret_code_t err_code;

    err_code = NRF_LOG_INIT(0);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    clocks_start();

    gpio_init();

    nrf_delay_ms(1);

    NRF_LOG_INFO("L01 test example Transmitter");  

    l01_configure();

    app_radio_print_l01_regs();
    
    uint8_t payload[8] = {1,0,1,2,3,4,5,6};
    
    while (true)
    {
        NRF_LOG_FLUSH();

        hal_nrf_write_tx_payload(payload, 8);
        payload[1]++;
        hal_nrf_chip_enable(CE_PULSE);
        nrf_delay_ms(1000);
    }
}
