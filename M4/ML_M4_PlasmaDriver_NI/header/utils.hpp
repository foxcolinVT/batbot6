#include <Arduino.h>

/*
 * Author: Ben Westcott
 * Date created: 8/25/22
 */

enum ML_port_direction { OUT = 0, IN = 1 };
enum ML_port_drvstrength { DRV_OFF = 0, DRV_ON = 1 };

void peripheral_port_init(const uint8_t pmux_msk, const uint8_t m4_pin, const ML_port_direction dir, const ML_port_drvstrength drv);

EAnalogChannel get_ADC_channel_num(const uint8_t adc_pin);

void TCC_sync(Tcc *TCCx);
void TCC_enable(Tcc *TCCx);
void TCC_disable(Tcc *TCCx);
void TCC_swrst(Tcc *TCCx);
void TCC_CH_CC_set(Tcc *TCCx, const uint8_t chnum, const uint8_t count_val);

void DMAC_enable(void);
void DMAC_disable(void);
void DMAC_swrst(void);
void DMAC_CH_enable(const uint8_t chnum);
void DMAC_CH_disable(const uint8_t chnum);
void DMAC_CH_swrst(const uint8_t chnum);
void DMAC_CH_intenset(const uint8_t chnum, const IRQn_Type IRQn, const uint8_t intmsk, const uint32_t prilvl);

void CCL_enable(void);
void CCL_disable(void);
void CCL_swrst(void);
void CCL_CH_enable(const uint8_t chnum);
void CCL_CH_disable(const uint8_t chnum);

void ADC_sync(Adc *ADCx);
void ADC_enable(Adc *ADCx);
void ADC_disable(Adc *ADCx);
void ADC_swrst(Adc *ADCx);
void ADC_swtrig_start(Adc *ADCx);
void ADC_slave_en(Adc *ADCx);
void ADC_prescale_set(Adc *ADCx, const uint16_t prescaler);

enum ML_SPI_mode { SLAVE = 0x2, MASTER = 0x3 };

void SERCOM_SPI_enable(SercomSpi *SPIx);
void SERCOM_SPI_disable(SercomSpi *SPIx);
void SERCOM_SPI_swrst(SercomSpi *SPIx);
void SERCOM_SPI_sync(SercomSpi *SPIx);
void SERCOM_SPI_intenset(SercomSpi *SPIx, IRQn_Type IRQn, const uint8_t intmsk, const uint8_t prilvl);

void TCC0_DT_set(uint8_t dth, uint8_t dtl);