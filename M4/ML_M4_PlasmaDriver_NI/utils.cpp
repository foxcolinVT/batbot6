/*
 * Author: Ben Westcott
 * Date created: 8/20/22
 */

#include "header/utils.hpp"

void peripheral_port_init(const uint8_t pmux_msk, const uint8_t m4_pin, const ML_port_direction dir, const ML_port_drvstrength drv){

  const EPortType port_grp = g_APinDescription[m4_pin].ulPort;
  const uint32_t pin = g_APinDescription[m4_pin].ulPin;

  PORT->Group[port_grp].PINCFG[pin].reg |= (dir ? (PORT_PINCFG_PMUXEN | PORT_PINCFG_INEN) : PORT_PINCFG_PMUXEN);

  PORT->Group[port_grp].PINCFG[pin].reg |= (drv ? PORT_PINCFG_DRVSTR : 0x0);

  PORT->Group[port_grp].PMUX[pin >> 1].reg |= pmux_msk;

  PORT->Group[port_grp].DIRSET.reg |= dir ? PORT_DIRSET_DIRSET(0x0) : PORT_DIRSET_DIRSET(pin);

}

EAnalogChannel get_ADC_channel_num(const uint8_t adc_pin){ 
    return g_APinDescription[adc_pin].ulADCChannelNumber;
}

void TCC_enable(Tcc *TCCx){
    TCCx->CTRLA.reg |= TCC_CTRLA_ENABLE;
    TCC_sync(TCCx);
}

void TCC_disable(Tcc *TCCx){
    TCCx->CTRLA.reg &= ~TCC_CTRLA_ENABLE;
    TCC_sync(TCCx);
}

void TCC_swrst(Tcc *TCCx){
    TCCx->CTRLA.reg |= TCC_CTRLA_SWRST;
    TCC_sync(TCCx);
}

void TCC_sync(Tcc *TCCx){
    while(TCCx->SYNCBUSY.reg);
}

void TCC_CH_CC_set(Tcc *TCCx, const uint8_t chnum, const uint8_t count_val){
    TCCx->CC[chnum].reg = TCC_CC_CC(count_val);
    TCC_sync(TCCx);
}

void DMAC_enable(void){
    DMAC->CTRL.reg |= DMAC_CTRL_DMAENABLE;
}

void DMAC_disable(void){
    DMAC->CTRL.reg &= ~DMAC_CTRL_DMAENABLE;
}

void DMAC_swrst(void){
    DMAC->CTRL.reg |= DMAC_CTRL_SWRST;
}

void DMAC_CH_enable(const uint8_t chnum){
    DMAC->Channel[chnum].CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;
}

void DMAC_CH_disable(const uint8_t chnum){
    DMAC->Channel[chnum].CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;
}

void DMAC_CH_swrst(const uint8_t chnum){
    DMAC->Channel[chnum].CHCTRLA.reg |= DMAC_CHCTRLA_SWRST;
}

void CCL_enable(void){
    CCL->CTRL.reg |= CCL_CTRL_ENABLE;
}

void CCL_disable(void){
    CCL->CTRL.reg &= ~CCL_CTRL_ENABLE;
}

void CCL_swrst(void){
    CCL->CTRL.reg |= CCL_CTRL_SWRST;
}

void CCL_CH_enable(const uint8_t chnum){
    CCL->LUTCTRL[chnum].reg |= CCL_LUTCTRL_ENABLE;
}

void CCL_CH_disable(const uint8_t chnum){
    CCL->LUTCTRL[chnum].reg &= ~CCL_LUTCTRL_ENABLE;
}

void ADC_enable(Adc *ADCx){
    ADCx->CTRLA.reg |= ADC_CTRLA_ENABLE;
    ADC_sync(ADCx);
}

void ADC_disable(Adc *ADCx){
    ADCx->CTRLA.reg &= ~ADC_CTRLA_ENABLE;
    ADC_sync(ADCx);
}

void ADC_swrst(Adc *ADCx){
    ADCx->CTRLA.reg |= ADC_CTRLA_SWRST;
    ADC_sync(ADCx);
}

void ADC_sync(Adc *ADCx){
    while(ADCx->SYNCBUSY.reg);
}

void TCC0_DT_set(uint8_t dth, uint8_t dtl){
  TCC0->WEXCTRL.reg |= (TCC_WEXCTRL_DTIEN0 | TCC_WEXCTRL_DTLS(dtl) | TCC_WEXCTRL_DTHS(dth));
}

// mode: 4, 5, 6
void TCC0_DITH_set(uint8_t mode, uint64_t cycles, uint64_t period, uint64_t compare){

  uint64_t CTRLA_res_msk;
  uint64_t PER_DITH_msk, CC_DITH_msk;
  switch(mode){
    case 5: {
      CTRLA_res_msk = TCC_CTRLA_RESOLUTION_DITH5;
      PER_DITH_msk = TCC_PER_DITH5_PER(period) | TCC_PER_DITH5_DITHER(cycles);
      CC_DITH_msk = TCC_CC_DITH5_CC(compare) | TCC_CC_DITH5_DITHER(cycles);
      break;
    }

    case 6: {
      CTRLA_res_msk = TCC_CTRLA_RESOLUTION_DITH6;
      PER_DITH_msk = TCC_PER_DITH6_PER(period) | TCC_PER_DITH6_DITHER(cycles);
      CC_DITH_msk = TCC_CC_DITH6_CC(compare) | TCC_CC_DITH6_DITHER(cycles);
      break;
    }

    default: {
      CTRLA_res_msk = TCC_CTRLA_RESOLUTION_DITH4;
      PER_DITH_msk = TCC_PER_DITH4_PER(period) | TCC_PER_DITH4_DITHER(cycles);
      CC_DITH_msk = TCC_CC_DITH4_CC(compare) | TCC_CC_DITH4_DITHER(cycles);
      break;
    }
  }

  TCC0->CTRLA.reg |= CTRLA_res_msk;

  TCC0->PER.reg = PER_DITH_msk;
  //TCC_sync(TCC0);

  TCC0->CC[0].reg = CC_DITH_msk;
  TCC0->CC[0].reg = CC_DITH_msk;
  TCC_sync(TCC0);
}