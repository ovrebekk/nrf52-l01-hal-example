#include "nrf5dk_l01.h"
#include "nordic_common.h"
#include "nrf_gpio.h"
//#include "nrf_drv_config.h"
#include "nrf_drv_gpiote.h"
#include "nrf_delay.h"

#define UINT8(t) ((uint8_t) (t))
#define CSN_LOW() nrf_gpio_pin_clear(m_current_l01_instance->pin_ss)
#define CSN_HIGH() nrf_gpio_pin_set(m_current_l01_instance->pin_ss); nrf_delay_us(10)

static void nrf_write_multibyte_reg_common_body(const uint8_t *buf, uint8_t length);
static void nrf_read_multibyte_reg_common_body(uint8_t *buf, uint8_t length);



/** Basis function, read_multibyte register .
 * Use this function to read multiple bytes from
 * a multibyte radio-register
 *
 * @param reg Multibyte register to read from
 * @param *pbuf Pointer to buffer in which to store read bytes to
 *
 * @return pipe# of received data (MSB), if operation used by a hal_nrf_read_rx_pload
 * @return length of read data (LSB), either for hal_nrf_read_rx_pload or
 * for hal_nrf_get_address.
*/
uint16_t hal_nrf_read_multibyte_reg(uint8_t reg, uint8_t *pbuf);

/** Basis function, write_multibyte register.
 * Use this function to write multiple bytes to
 * a multiple radio register.
 *
 * @param reg Register to write
 * @param *pbuf pointer to buffer in which data to write is
 * @param length \# of bytes to write
*/
void hal_nrf_write_multibyte_reg(uint8_t reg, const uint8_t *pbuf, uint8_t length);

/**
 * Typedef for the CONFIG register. Contains all the bitaddressable
 * settings in the bits struct and the value sent to the radio in the uint8_t
 */
typedef union {
  uint8_t value;
	struct {
		uint8_t prim_rx : 1;
		uint8_t pwr_up : 1;
		uint8_t crc0 : 1;
		uint8_t en_crc : 1;
		uint8_t mask_max_rt : 1;
		uint8_t mask_tx_ds : 1;
		uint8_t mask_rx_dr : 1;
		const uint8_t : 1;
	} bits;
} config_t;

/**
 * Typedef for the EN_AA, EN_RXADDR and DYNPD registers. Contains all the
 * bitaddressable settings in the bits struct and the value sent to the radio
 * in the uint8_t
 */
typedef union {
  uint8_t value;
  struct {
    uint8_t pipe_0 : 1;
    uint8_t pipe_1 : 1;
    uint8_t pipe_2 : 1;
    uint8_t pipe_3 : 1;
    uint8_t pipe_4 : 1;
    uint8_t pipe_5 : 1;
    const uint8_t : 2;
  } bits;
} en_pipes_t;

/**
 * Typedef for the SETUP_AW register. Contains all the bitaddressable
 * settings in the bits struct and the value sent to the radio in the uint8_t
 */
typedef union {
  uint8_t value;
	struct {
		uint8_t aw : 2;
		const uint8_t : 6;
	} bits;
} setup_aw_t;

/**
 * Typedef for the SETUP_RETR register. Contains all the bitaddressable
 * settings in the bits struct and the value sent to the radio in the uint8_t
 */
typedef union {
  uint8_t value;
	struct {
		uint8_t arc : 4;
		uint8_t ard : 4;
	} bits;
} setup_retr_t;

/**
 * Typedef for the RF_CH register. Contains all the bitaddressable
 * settings in the bits struct and the value sent to the radio in the uint8_t
 */
typedef union {
  uint8_t value;
	struct {
		uint8_t rf_ch : 7;
		const uint8_t : 1;
	} bits;
} rf_ch_t;

/**
 * Typedef for the RF_SETUP register. Contains all the bitaddressable
 * settings in the bits struct and the value sent to the radio in the uint8_t
 */
typedef union {
  uint8_t value;
	struct {
		const uint8_t : 1;
		uint8_t rf_pwr : 2;
		uint8_t rf_dr_high : 1;
		uint8_t pll_lock : 1;
		uint8_t rf_dr_low : 1;
    const uint8_t : 1;
    uint8_t cont_wave : 1;
	} bits;
} rf_setup_t;

/**
 * Typedef for the RX_PW_Px registers. Contains all the bitaddressable
 * settings in the bits struct and the value sent to the radio in the uint8_t
 */
typedef union {
  uint8_t value;
	struct {
		uint8_t rx_pw : 6;
		const uint8_t : 2;
	} bits;
} rx_pw_t;

/**
 * Typedef for the FEATURE register. Contains all the bitaddressable
 * settings in the bits struct and the value sent to the radio in the uint8_t
 */
typedef union {
  uint8_t value;
	struct {
		uint8_t en_dyn_ack : 1;
		uint8_t en_ack_pay : 1;
		uint8_t en_dpl : 1;
		const uint8_t : 5;
	} bits;
} feature_t;

static const nrf_drv_spi_t  m_spi_master[L01_MODULE_MAX_COUNT] = {NRF_DRV_SPI_INSTANCE(0), NRF_DRV_SPI_INSTANCE(1)};
static uint32_t             m_current_index = 0;
static radio_irq_callback_t m_callback;
static l01_instance_t *m_current_l01_instance;

void gpiote_evt_handler_pin_irq(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    if(pin == m_current_l01_instance->pin_irq && action == NRF_GPIOTE_POLARITY_HITOLO)
    {
        m_callback();
    }
}

void hal_nrf_nrf52_init(l01_instance_t *l01_instance, l01_config_t *config, radio_irq_callback_t callback)
{
    uint32_t err_code;
    nrf_drv_spi_config_t spi_config;
    if(m_current_index < L01_MODULE_MAX_COUNT)
    {
        spi_config.frequency = NRF_DRV_SPI_FREQ_250K;
        spi_config.mode      = NRF_DRV_SPI_MODE_0;
        spi_config.bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;
        spi_config.ss_pin    = NRF_DRV_SPI_PIN_NOT_USED;
        spi_config.mosi_pin  = config->pin_mosi;
        spi_config.miso_pin  = config->pin_miso;
        spi_config.sck_pin   = config->pin_sck;
        l01_instance->spi_instance = m_spi_master[config->spi_num];
        err_code = nrf_drv_spi_init(&l01_instance->spi_instance, &spi_config, NULL, NULL);
        if (err_code != NRF_SUCCESS)
        {
            // Initialization failed. Take recovery action.
        }
        
        l01_instance->pin_ss    = config->pin_csn;
        l01_instance->pin_ce    = config->pin_ce;  
        l01_instance->pin_irq   = config->pin_irq;
        nrf_gpio_cfg_output(l01_instance->pin_ss);
        nrf_gpio_pin_set(l01_instance->pin_ss);
        nrf_gpio_cfg_output(l01_instance->pin_ce);
        nrf_gpio_pin_clear(l01_instance->pin_ce);
        nrf_gpio_cfg_input(l01_instance->pin_irq, NRF_GPIO_PIN_NOPULL);
        
        if(!nrf_drv_gpiote_is_init())
        {
            nrf_drv_gpiote_init();
        }   
        nrf_drv_gpiote_in_config_t gpiote_config;
        gpiote_config.hi_accuracy   = false;
        gpiote_config.is_watcher    = false;
        gpiote_config.pull          = NRF_GPIO_PIN_NOPULL;
        gpiote_config.sense         = NRF_GPIOTE_POLARITY_HITOLO;

        err_code = nrf_drv_gpiote_in_init(l01_instance->pin_irq, &gpiote_config, gpiote_evt_handler_pin_irq);
        nrf_drv_gpiote_in_event_enable(l01_instance->pin_irq, true);
        
        m_callback = callback;
        m_current_l01_instance = l01_instance;
        m_current_index++;
    }
}

void hal_nrf_nrf52_set_instance(l01_instance_t *l01_instance)
{
    
}

void hal_nrf_chip_enable(ce_mode_t ce_mode)
{
    switch(ce_mode)
    {
        case CE_ENABLE:
            nrf_gpio_pin_set(m_current_l01_instance->pin_ce);
            break;

        case CE_DISABLE:
            nrf_gpio_pin_clear(m_current_l01_instance->pin_ce);
            break;

        case CE_PULSE:
            nrf_gpio_pin_set(m_current_l01_instance->pin_ce);
            nrf_delay_us(10);
            nrf_gpio_pin_clear(m_current_l01_instance->pin_ce);
            break;
    }
}

static uint8_t hal_nrf_rw(uint8_t value)
{
    static uint8_t ret_val;
    nrf_drv_spi_transfer(&m_current_l01_instance->spi_instance, &value, 1, &ret_val, 1);
    return ret_val;
}

void hal_nrf_set_operation_mode(hal_nrf_operation_mode_t op_mode)
{
  config_t config;
  config.value = hal_nrf_read_reg (L01REG_CONFIG);

  if(op_mode == HAL_NRF_PRX)
  {
    config.bits.prim_rx = 1U;
  }
  else
  {
    config.bits.prim_rx = 0U;
  }

  hal_nrf_write_reg (L01REG_CONFIG, config.value);
}

void hal_nrf_set_power_mode(hal_nrf_pwr_mode_t pwr_mode)
{
  config_t config;
  config.value = hal_nrf_read_reg (L01REG_CONFIG);

  if(pwr_mode == HAL_NRF_PWR_UP)
  {
    config.bits.pwr_up = 1U;
  }
  else
  {
    config.bits.pwr_up = 0U;
  }

  hal_nrf_write_reg (L01REG_CONFIG, config.value);
}

void hal_nrf_set_crc_mode(hal_nrf_crc_mode_t crc_mode)
{
  config_t config;
  config.value = hal_nrf_read_reg (L01REG_CONFIG);

	switch (crc_mode)
	{
		case HAL_NRF_CRC_OFF:
			config.bits.en_crc = 0U;
			break;
		case HAL_NRF_CRC_8BIT:
			config.bits.en_crc = 1U;
			config.bits.crc0 = 0U;
			break;
		case HAL_NRF_CRC_16BIT:
			config.bits.en_crc = 1U;
			config.bits.crc0 = 1U;
			break;
		default:
			break;
	}

  hal_nrf_write_reg (L01REG_CONFIG, config.value);
}

void hal_nrf_set_irq_mode(hal_nrf_irq_source_t int_source, bool irq_state)
{
  config_t config;
  config.value = hal_nrf_read_reg (L01REG_CONFIG);

	switch (int_source)
	{
		case HAL_NRF_MAX_RT:
			config.bits.mask_max_rt = irq_state ? 0U : 1U;
      break;
    case HAL_NRF_TX_DS:
      config.bits.mask_tx_ds = irq_state ? 0U : 1U;
      break;
    case HAL_NRF_RX_DR:
      config.bits.mask_rx_dr = irq_state ? 0U : 1U;
      break;
  }

  hal_nrf_write_reg (L01REG_CONFIG, config.value);
}

uint8_t hal_nrf_get_clear_irq_flags(void)
{
  uint8_t retval;

  retval = hal_nrf_write_reg (L01REG_STATUS, (BIT_6|BIT_5|BIT_4));

  return (retval & (BIT_6|BIT_5|BIT_4));
}

uint8_t hal_nrf_clear_irq_flags_get_status(void)
{
  uint8_t retval;

  // When RFIRQ is cleared (when calling write_reg), pipe information is unreliable (read again with read_reg)
  retval = hal_nrf_write_reg (L01REG_STATUS, (BIT_6|BIT_5|BIT_4)) & (BIT_6|BIT_5|BIT_4);
  retval |= hal_nrf_read_reg (L01REG_STATUS) & (BIT_3|BIT_2|BIT_1|BIT_0);

  return (retval);
}


void hal_nrf_clear_irq_flag(hal_nrf_irq_source_t int_source)
{
  hal_nrf_write_reg (L01REG_STATUS, 1 << int_source);
}

uint8_t hal_nrf_get_irq_flags(void)
{
  return hal_nrf_nop() & (BIT_6|BIT_5|BIT_4);
}

void hal_nrf_open_pipe(hal_nrf_address_t pipe_num, bool auto_ack)
{
  en_pipes_t en_rxaddr;
  en_pipes_t en_aa;
  en_rxaddr.value = hal_nrf_read_reg (EN_RXADDR);
  en_aa.value = hal_nrf_read_reg (EN_AA);

  switch(pipe_num)
  {
    case HAL_NRF_PIPE0:
    case HAL_NRF_PIPE1:
    case HAL_NRF_PIPE2:
    case HAL_NRF_PIPE3:
    case HAL_NRF_PIPE4:
    case HAL_NRF_PIPE5:
      en_rxaddr.value = en_rxaddr.value | (1 << pipe_num);

      if(auto_ack)
      {
        en_aa.value = en_aa.value | (1 << pipe_num);
      }
      else
      {
        en_aa.value = en_aa.value & (uint8_t)~(1 << pipe_num);
      }
      break;

    case HAL_NRF_ALL:
      en_rxaddr.value = (uint8_t)(~(BIT_6|BIT_7));

      if(auto_ack)
      {
        en_aa.value = (uint8_t)(~(BIT_6|BIT_7));
      }
      else
      {
        en_aa.value = 0U;
      }
      break;

    case HAL_NRF_TX:
    default:
      break;
  }

  hal_nrf_write_reg (EN_RXADDR, en_rxaddr.value);
  hal_nrf_write_reg (EN_AA, en_aa.value);
}

void hal_nrf_close_pipe(hal_nrf_address_t pipe_num)
{
  en_pipes_t en_rxaddr;
  en_pipes_t en_aa;
  en_rxaddr.value = hal_nrf_read_reg (EN_RXADDR);
  en_aa.value = hal_nrf_read_reg (EN_AA);

  switch(pipe_num)
  {
    case HAL_NRF_PIPE0:
    case HAL_NRF_PIPE1:
    case HAL_NRF_PIPE2:
    case HAL_NRF_PIPE3:
    case HAL_NRF_PIPE4:
    case HAL_NRF_PIPE5:
      en_rxaddr.value = en_rxaddr.value & (uint8_t)~(1 << pipe_num);
      en_aa.value = en_aa.value & (uint8_t)~(1 << pipe_num);
      break;

    case HAL_NRF_ALL:
      en_rxaddr.value = 0U;
      en_aa.value = 0U;
      break;

    case HAL_NRF_TX:
    default:
      break;
  }

  hal_nrf_write_reg (EN_RXADDR, en_rxaddr.value);
  hal_nrf_write_reg (EN_AA, en_aa.value);
}

void hal_nrf_set_address(const hal_nrf_address_t address, const uint8_t *addr)
{
  switch(address)
  {
    case HAL_NRF_TX:
    case HAL_NRF_PIPE0:
    case HAL_NRF_PIPE1:
      hal_nrf_write_multibyte_reg(W_REGISTER + RX_ADDR_P0 + (uint8_t) address, addr, hal_nrf_get_address_width());
      break;
    case HAL_NRF_PIPE2:
    case HAL_NRF_PIPE3:
    case HAL_NRF_PIPE4:
    case HAL_NRF_PIPE5:
      hal_nrf_write_reg (RX_ADDR_P0 + (uint8_t) address, *addr);
      break;

    case HAL_NRF_ALL:
    default:
      break;
  }
}

uint8_t hal_nrf_get_address(uint8_t address, uint8_t *addr)
{
  switch (address)
  {
    case HAL_NRF_PIPE0:
    case HAL_NRF_PIPE1:
    case HAL_NRF_TX:
      return (uint8_t)hal_nrf_read_multibyte_reg (address, addr);
    default:
      *addr = hal_nrf_read_reg(RX_ADDR_P0 + address);
      return 1U;
  }
}

void hal_nrf_set_auto_retr(uint8_t retr, uint16_t delay)
{
  setup_retr_t setup_retr;
  setup_retr.bits.ard = (uint8_t)(delay >> 8);
  setup_retr.bits.arc = retr;

  hal_nrf_write_reg (SETUP_RETR, setup_retr.value);
}

void hal_nrf_set_address_width(hal_nrf_address_width_t address_width)
{
  setup_aw_t setup_aw;
  setup_aw.value = 0U;
  setup_aw.bits.aw = (uint8_t)address_width - 2U;

  hal_nrf_write_reg (SETUP_AW, setup_aw.value);
}

uint8_t hal_nrf_get_address_width (void)
{
  return hal_nrf_read_reg(SETUP_AW) + 2U;
}

void hal_nrf_set_rx_payload_width(uint8_t pipe_num, uint8_t pload_width)
{
  hal_nrf_write_reg (RX_PW_P0 + pipe_num, pload_width);
}

uint8_t hal_nrf_get_pipe_status(uint8_t pipe_num)
{
  en_pipes_t en_rxaddr;
  en_pipes_t en_aa;
  uint8_t en_rx_r, en_aa_r;

  en_rxaddr.value = hal_nrf_read_reg (EN_RXADDR);
  en_aa.value = hal_nrf_read_reg (EN_AA);

  switch (pipe_num)
  {
    case 0:
      en_rx_r = en_rxaddr.bits.pipe_0;
      en_aa_r = en_aa.bits.pipe_0;
      break;
    case 1:
      en_rx_r = en_rxaddr.bits.pipe_1;
      en_aa_r = en_aa.bits.pipe_1;
      break;
    case 2:
      en_rx_r = en_rxaddr.bits.pipe_2;
      en_aa_r = en_aa.bits.pipe_2;
      break;
    case 3:
      en_rx_r = en_rxaddr.bits.pipe_3;
      en_aa_r = en_aa.bits.pipe_3;
      break;
    case 4:
      en_rx_r = en_rxaddr.bits.pipe_4;
      en_aa_r = en_aa.bits.pipe_4;
      break;
    case 5:
      en_rx_r = en_rxaddr.bits.pipe_5;
      en_aa_r = en_aa.bits.pipe_5;
      break;
    default:
      en_rx_r = 0U;
      en_aa_r = 0U;
      break;
  }

  return (uint8_t)(en_aa_r << 1) + en_rx_r;
}

uint8_t hal_nrf_get_auto_retr_status(void)
{
  return hal_nrf_read_reg(OBSERVE_TX);
}

uint8_t hal_nrf_get_packet_lost_ctr(void)
{
  return ((hal_nrf_read_reg(OBSERVE_TX) & (BIT_7|BIT_6|BIT_5|BIT_4)) >> 4);
}

uint8_t hal_nrf_get_rx_payload_width(uint8_t pipe_num)
{
  uint8_t pw;

  switch (pipe_num)
  {
    case 0:
      pw = hal_nrf_read_reg (RX_PW_P0);
      break;
    case 1:
      pw = hal_nrf_read_reg (RX_PW_P1);
      break;
    case 2:
      pw = hal_nrf_read_reg (RX_PW_P2);
      break;
    case 3:
      pw = hal_nrf_read_reg (RX_PW_P3);
      break;
    case 4:
      pw = hal_nrf_read_reg (RX_PW_P4);
      break;
    case 5:
      pw = hal_nrf_read_reg (RX_PW_P5);
      break;
    default:
      pw = 0U;
      break;
  }

  return pw;
}

void hal_nrf_set_rf_channel(uint8_t channel)
{
  rf_ch_t rf_ch;
  rf_ch.value = 0U;
  rf_ch.bits.rf_ch = channel;
  hal_nrf_write_reg (RF_CH, rf_ch.value);
}

void hal_nrf_set_output_power(hal_nrf_output_power_t power)
{
  rf_setup_t rf_setup;
  rf_setup.value = hal_nrf_read_reg (RF_SETUP);

  rf_setup.bits.rf_pwr = (uint8_t)power;

  hal_nrf_write_reg (RF_SETUP, rf_setup.value);
}

void hal_nrf_set_datarate(hal_nrf_datarate_t datarate)
{
  rf_setup_t rf_setup;
  rf_setup.value = hal_nrf_read_reg (RF_SETUP);

  switch (datarate)
  {
    case HAL_NRF_250KBPS:
      rf_setup.bits.rf_dr_low = 1U;
      rf_setup.bits.rf_dr_high = 0U;
      break;
    case HAL_NRF_1MBPS:
      rf_setup.bits.rf_dr_low = 0U;
      rf_setup.bits.rf_dr_high = 0U;
      break;
    case HAL_NRF_2MBPS:
    default:
      rf_setup.bits.rf_dr_low = 0U;
      rf_setup.bits.rf_dr_high = 1U;
      break;
  }

  hal_nrf_write_reg (RF_SETUP, rf_setup.value);
}

bool hal_nrf_rx_fifo_empty(void)
{
  return (bool)((hal_nrf_read_reg(FIFO_STATUS) >> RX_EMPTY) & 0x01U);
}

bool hal_nrf_rx_fifo_full(void)
{
  return (bool)((hal_nrf_read_reg(FIFO_STATUS) >> RX_FULL) & 0x01U);
}

bool hal_nrf_tx_fifo_empty(void)
{
  return (bool)((hal_nrf_read_reg(FIFO_STATUS) >> TX_EMPTY) & 0x01U);
}

bool hal_nrf_tx_fifo_full(void)
{
  return (bool)((hal_nrf_read_reg(FIFO_STATUS) >> TX_FIFO_FULL) & 0x01U);
}

uint8_t hal_nrf_get_tx_fifo_status(void)
{
  return ((hal_nrf_read_reg(FIFO_STATUS) & ((1U<<TX_FIFO_FULL)|(1U<<TX_EMPTY))) >> 4);
}

uint8_t hal_nrf_get_rx_fifo_status(void)
{
  return (hal_nrf_read_reg(FIFO_STATUS) & ((1U<<RX_FULL)|(1U<<RX_EMPTY)));
}

uint8_t hal_nrf_get_fifo_status(void)
{
  return hal_nrf_read_reg(FIFO_STATUS);
}

uint8_t hal_nrf_get_transmit_attempts(void)
{
  return (hal_nrf_read_reg(OBSERVE_TX) & (BIT_3|BIT_2|BIT_1|BIT_0));
}

bool hal_nrf_get_carrier_detect(void)
{
  return (bool)(hal_nrf_read_reg(CD) & 0x01U);
}

void hal_nrf_activate_features(void)
{
  CSN_LOW();
  hal_nrf_rw(0x50);
  hal_nrf_rw(0x73);
  CSN_HIGH();
}

void hal_nrf_setup_dynamic_payload (uint8_t setup)
{
  en_pipes_t dynpd;
  dynpd.value = setup & (uint8_t)~0xC0U;

  hal_nrf_write_reg (DYNPD, dynpd.value);
}

void hal_nrf_enable_dynamic_payload(bool enable)
{
  feature_t feature;
  feature.value = hal_nrf_read_reg (FEATURE);
  feature.bits.en_dpl = (enable) ? 1U : 0U;

  hal_nrf_write_reg (FEATURE, feature.value);
}

void hal_nrf_enable_ack_payload(bool enable)
{
  feature_t feature;
  feature.value = hal_nrf_read_reg (FEATURE);
  feature.bits.en_ack_pay = (enable) ? 1U : 0U;

  hal_nrf_write_reg (FEATURE, feature.value);
}

void hal_nrf_enable_dynamic_ack(bool enable)
{
  feature_t feature;
  feature.value = hal_nrf_read_reg (FEATURE);
  feature.bits.en_dyn_ack = (enable) ? 1U : 0U;

  hal_nrf_write_reg (FEATURE, feature.value);
}

void hal_nrf_write_tx_payload(const uint8_t *tx_pload, uint8_t length)
{
  hal_nrf_write_multibyte_reg(W_TX_PAYLOAD, tx_pload, length);
}

void hal_nrf_write_tx_payload_noack(const uint8_t *tx_pload, uint8_t length)
{
  hal_nrf_write_multibyte_reg(W_TX_PAYLOAD_NOACK, tx_pload, length);
}

void hal_nrf_write_ack_payload(uint8_t pipe, const uint8_t *tx_pload, uint8_t length)
{
  hal_nrf_write_multibyte_reg(W_ACK_PAYLOAD | pipe, tx_pload, length);
}

uint8_t hal_nrf_read_rx_payload_width(void)
{
  uint8_t ret_val;

  CSN_LOW();
  hal_nrf_rw(R_RX_PL_WID);
  ret_val = hal_nrf_rw(0);
  CSN_HIGH();

  // If the length is larger than 32 bytes then the packet is corrupt, and the RX fifo should be flushed
  if(ret_val > 32)
  {
    hal_nrf_flush_rx();
    return 0;
  }

  return ret_val;
}

uint16_t hal_nrf_read_rx_payload(uint8_t *rx_pload)
{
  uint8_t reg;
  uint8_t length;
  reg = hal_nrf_get_rx_data_source();
  if (reg < 7)
  {
    length = hal_nrf_read_rx_payload_width();
    if(length > 0)
    {
      CSN_LOW();
      hal_nrf_rw(R_RX_PAYLOAD);
      for(int i = 0; i < length; i++)
      {
        *rx_pload++ = hal_nrf_rw(0);
      }
      CSN_HIGH();
    }
  }
  else
  {
    length = 0;
  }
  return (((uint16_t) reg << 8) | length);
}

uint8_t hal_nrf_get_rx_data_source(void)
{
  return ((hal_nrf_nop() & (BIT_3|BIT_2|BIT_1)) >> 1);
}

void hal_nrf_reuse_tx(void)
{
  CSN_LOW();
  hal_nrf_rw(REUSE_TX_PL);
  CSN_HIGH();
}

bool hal_nrf_get_reuse_tx_status(void)
{
  return (bool)((hal_nrf_get_fifo_status() & (1U<<TX_REUSE)) >> TX_REUSE);
}

void hal_nrf_flush_rx(void)
{
  CSN_LOW();
  hal_nrf_rw(FLUSH_RX);
  CSN_HIGH();
}

void hal_nrf_flush_tx(void)
{
  CSN_LOW();
  hal_nrf_rw(FLUSH_TX);
  CSN_HIGH();
}

uint8_t hal_nrf_nop(void)
{
  uint8_t retval;

  CSN_LOW();
  retval = hal_nrf_rw(L01CMD_NOP);
  CSN_HIGH();

  return retval;
}

void hal_nrf_set_pll_mode(bool pll_lock)
{
  rf_setup_t rf_setup;
  rf_setup.value = hal_nrf_read_reg (RF_SETUP);
  rf_setup.bits.pll_lock = (pll_lock) ? 1U : 0U;

  hal_nrf_write_reg(RF_SETUP, rf_setup.value);
}

void hal_nrf_enable_continious_wave (bool enable)
{
  rf_setup_t rf_setup;
  rf_setup.value = hal_nrf_read_reg (RF_SETUP);
  rf_setup.bits.cont_wave = (enable ? 1U : 0U);

  hal_nrf_write_reg(RF_SETUP, rf_setup.value);
}

uint8_t hal_nrf_read_reg(uint8_t reg)
{
    uint8_t spi_tx_data[] = {reg, 0};
    static uint8_t return_data[2];

    CSN_LOW();
    nrf_drv_spi_transfer(&m_current_l01_instance->spi_instance, spi_tx_data, 2, return_data, 2);
    CSN_HIGH();
    return return_data[1];
}

uint8_t hal_nrf_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t spi_tx_data[2];
    uint8_t spi_rx_data[2];

    CSN_LOW();
    spi_tx_data[0] = W_REGISTER + reg;
    spi_tx_data[1] = value;
    nrf_drv_spi_transfer(&m_current_l01_instance->spi_instance, spi_tx_data, 2, spi_rx_data, 2);
    CSN_HIGH();

    return spi_rx_data[0];
}

void nrf_read_multibyte_reg_common_body(uint8_t *buf, uint8_t length)
{
    nrf_drv_spi_transfer(&m_current_l01_instance->spi_instance, 0, 0, buf, length);
}

uint16_t hal_nrf_read_multibyte_reg(uint8_t reg, uint8_t *pbuf)
{
  uint8_t length;

  switch(reg)
  {
    case HAL_NRF_PIPE0:
    case HAL_NRF_PIPE1:
    case HAL_NRF_TX:
      length = hal_nrf_get_address_width();
      CSN_LOW();
      hal_nrf_rw(RX_ADDR_P0 + reg);
      break;

    case HAL_NRF_RX_PLOAD:
      reg = hal_nrf_get_rx_data_source();
      if (reg < 7U)
      {
        length = hal_nrf_read_rx_payload_width();
        CSN_LOW();
        hal_nrf_rw(R_RX_PAYLOAD);
      }
      else
      {
        length = 0U;
      }
      break;

    default:
      length = 0U;
      break;
  }

  uint8_t *buf = (uint8_t *)pbuf;
  nrf_read_multibyte_reg_common_body(buf, length);

  CSN_HIGH();

  return (((uint16_t) reg << 8) | length);
}

void nrf_write_multibyte_reg_common_body(const uint8_t *buf, uint8_t length)
{
    nrf_drv_spi_transfer(&m_current_l01_instance->spi_instance, buf, length, 0, 0);
}

void hal_nrf_write_multibyte_reg(uint8_t reg, const uint8_t *pbuf, uint8_t length)
{
    CSN_LOW();
    hal_nrf_rw(reg);
    nrf_write_multibyte_reg_common_body(pbuf, length);
    CSN_HIGH();
}
