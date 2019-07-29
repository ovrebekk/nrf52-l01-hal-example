#include "app_radio.h"
#include "nrf5dk_l01.h"
#include "nrf_delay.h"
#include "nrf_log.h"

static l01_instance_t my_l01;
static app_radio_callback_t m_on_transmit, m_on_max_retransmit, m_on_receive;

static void l01_irq(void)
{
    static uint8_t rx_payload[32];
    static uint8_t irq_flags;
    static uint32_t packet_length;
    
    hal_nrf_nrf52_set_instance(&my_l01);
    irq_flags = hal_nrf_get_clear_irq_flags();
    if(irq_flags & (1 << HAL_NRF_TX_DS))
    {
        if(m_on_transmit)
        {
            m_on_transmit();
        }
    }
    if(irq_flags & (1 << HAL_NRF_MAX_RT))
    {
        if(m_on_max_retransmit)
        {
            m_on_max_retransmit();
        }
    }
    if(irq_flags & (1 << HAL_NRF_RX_DR))
    {
        if(m_on_receive)
        {
            m_on_receive();
        }
    }
}

uint32_t app_radio_init(app_radio_config_t *config)
{
    l01_config_t l01_config = {1, 11, 12, 13, 14, 15, 16};
    hal_nrf_nrf52_init(&my_l01, &l01_config, l01_irq);
    
    m_on_transmit = config->on_transmit;
    m_on_max_retransmit = config->on_max_retransmit;
    m_on_receive = config->on_receive;

    nrf_delay_us(1500);

    hal_nrf_set_operation_mode((config->mode == APP_RADIO_MODE_PTX) ? HAL_NRF_PTX : HAL_NRF_PRX);
 
    hal_nrf_set_power_mode(HAL_NRF_PWR_UP);
        
    nrf_delay_us(1500);
    
    hal_nrf_set_crc_mode(HAL_NRF_CRC_16BIT);

    hal_nrf_set_address_width(5);
    
    hal_nrf_set_datarate(HAL_NRF_2MBPS);
    
    hal_nrf_set_rf_channel(2);
    
    if(config->mode == APP_RADIO_MODE_PTX)
    {
        hal_nrf_flush_tx();
    }
    else
    {
        hal_nrf_flush_rx();
        hal_nrf_set_rx_payload_width(HAL_NRF_PIPE0, 8);
    }
    
    hal_nrf_setup_dynamic_payload(0x3F);
    hal_nrf_enable_dynamic_payload(true);
}

void app_radio_print_l01_regs(void)
{
    uint8_t reg;
    uint8_t multireg[5];
    uint16_t length;
    NRF_LOG_INFO("Reg readout:");
    //hal_nrf_nrf52_set_instance(&my_l01);
    for(int i = 0; i <= 0x1D; i++)
    {
        switch(i)
        {
            case RX_ADDR_P0:
            case RX_ADDR_P1:
            case TX_ADDR:
                hal_nrf_read_multibyte_reg(i - RX_ADDR_P0, multireg);
                NRF_LOG_INFO("  Reg 0x%.2X: 0x%.2X-0x%.2X-0x%.2X-0x%.2X-0x%.2X", (int)i, (int)multireg[0], (int)multireg[1], (int)multireg[2], (int)multireg[3], (int)multireg[4]);
                break;
                
            default:
                reg = hal_nrf_read_reg(i);
		NRF_LOG_INFO("  Reg 0x%.2X: 0x%.2X", (int)i, (int)reg);  
                break;
        }
    }
}