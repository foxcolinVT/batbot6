/*
 * Author: Ben Westcott
 * Date created: 8/10/22
 */

#include <Arduino.h>

// Note: macros/data structures used w/o prefixes ML (Mueller Lab) 
// exist at ${PATH_TO_ARDUINO_CONFIGS}/packages/adafruit/tools/CMSIS-Atmel/1.2.2/CMSIS/Device/ATMEL/samd51

#define ML_MCLK_UNDIV 120000000
#define ML_MCLK_CPUDIV1 (MCLK_CPUDIV_DIV(MCLK_CPUDIV_DIV_DIV1_Val))

#define ML_GCLK_CH 2
#define ML_GCLK_INITIAL_DIV 1
#define ML_GCLK_GENCTRL_DIV1 (GCLK_GENCTRL_DIV(ML_GCLK_INITIAL_DIV))
#define ML_GCLK_GENCTRL_SRC_DPLL (GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_DPLL0_Val))

// Channel enable, GCLK2, WRTLCK - disable future writing to reg
#define ML_GCLK2_PCHCTRL (GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK2 | GCLK_PCHCTRL_WRTLOCK)

// TODO: Finish PIOC controller programming
// TODO: APB/AHB coms more preferable for ADC/DAC operation? DMA exists on main bus matrix so idk if its more efficient to use GCLK and wait to sync w/ peripheral clocks
// TODO: Requests to DMA / look at ZeroDMA library since it could end up overwriting some modules. If it does, first try patching ZeroDMA, last resort is to directly program DMA 
// TODO: Play with deadtime values and dither values (optimize for a push/pull operation). Also look at PIOC for push/pull pin operation
// TODO: Interrupts
// TODO: If DAC, ADC, TCCx all exist in same clock domain, consider synchronization methods? 

static void GCLK0_init(void){

  MCLK->CPUDIV.reg = ML_MCLK_CPUDIV1;

  // wait for MCLK, 
  // while(!MCLK->INTFLAG.bit.CKRDY);                           

  GCLK->GENCTRL[ML_GCLK_CH].reg = ML_GCLK_GENCTRL_DIV1 |                // GCLK divider, GCLK0_FREQ = ML_MCLK_UNDIV/(ML_MCLK_CPUDIV * ML_GCLK_GENCTRL_DIV)
                                  GCLK_GENCTRL_IDC |                    // 50/50 duty 
                                  ML_GCLK_GENCTRL_SRC_DPLL |            // Source multiplexer selects DPLL0 (phase locked loop)
                                  GCLK_GENCTRL_OE | GCLK_GENCTRL_GENEN; // Output enable, Generator enable

                                 
  // wait for GEN2 sync
  while(GCLK->SYNCBUSY.bit.GENCTRL2);

  // Set ADC1/0 in GCLK peripheral channel cntrl registers
  //GCLK->PCHCTRL[ADC1_GCLK_ID].reg = ML_GCLK2_PCHCTRL;
  //GCLK->PCHCTRL[ADC0_GCLK_ID].reg = ML_GCLK2_PCHCTRL;

  // Set DAC in GCLK PCHCTRL
  //GCLK->PCHCTRL[DAC_GCLK_ID].reg =  ML_GCLK2_PCHCTRL;

  // PCHCTRL for TCC0/1/2
  GCLK->PCHCTRL[TCC0_GCLK_ID].reg = ML_GCLK2_PCHCTRL;                   // TCC0 and TCC1 ID are the same, so TCC1 is implicitly set
  //GCLK->PCHCTRL[TCC2_GCLK_ID].reg = ML_GCLK2_PCHCTRL;                     // TCC3 set for same above reason

}

#define GCLK0_FREQ (ML_MCLK_UNDIV/(1 * ML_HCLK_INITIAL_DIV))             // 60 Mhz

#define ML_M4_TCC0_CH0_PIN 7
#define ML_PB12_PIN 12
#define ML_TCC0_CH0_PMUXE 0x6                                            // pin # = 2*peripheral (even)
#define ML_TCC0_CH0 0x0

#define ML_M4_TCC0_CH3_PIN 6
#define ML_PB15_PIN 15
#define ML_TCC0_CH3_PMUXO 0x7                                              // pin # = 2*peripheral + 1 (odd)
#define ML_TCC0_CH3 0x3

void TCC0_init(void){
      
  // disable TCC
  TCC0->CTRLA.reg &= ~TCC_CTRLA_ENABLE;
  while(TCC0->SYNCBUSY.bit.ENABLE);
  
  // send software reset of TCC CTRLA.SWRST
  TCC0->CTRLA.reg |= TCC_CTRLA_SWRST;
  while(TCC0->SYNCBUSY.bit.SWRST);

  /*
  * Syncronization:
  * 
  * bits SYNCED on write: CTRLA.SWRST, CTRLA.ENABLE
  * regs SYNCED on write: CTRLBCLR, CTRLBSET, STATUS, WAVE, COUNT, PER/PERBUF, CCx/CCBUFx
  * regs SYNCED on read: CTRLBCLR, CTRLBSET, COUNT, WAVE, PER/PERBUF, CCx/CCBUFx
  */

  // DMA channel match request: set CTRLA.DMAOS=0
  
  // Sleep mode: CTRLA.RUNSTDBY
  // set prescalar, and prescalar sync
  // STOP cmd: CTRLB.CMD=0x2
  // PAUSE event: EVCTRL.EVACT1=0x3, STOP, EVCTRL.EVACT0=0x3, START
  // EVENT ACTIONS**
  TCC0->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV2 |
                  TCC_CTRLA_PRESCSYNC_PRESC;                             // try TC_CTRLA_PRESYNC_GCLK or TC_CTRLA_PRESYNC_RESYNC
  //              TCC_CTRLA_RESOLUTION_DITH4 |                           // enable dithering every 16 PWM frames
  //              TCC_CTRLA_RUNSTDBY;                                    // run TCC when MP in standby
   
  
  // dead time reg
  /*
  TCC0->WEXCTRL.reg = TCC_WEXCTRL_DTIEN0       |                         // DT insertion gen 0 enable
                      TCC_WEXCTRL_DTHS(0x1)      |                       // DT high side value set
                      TCC_WEXCTRL_DTLS(0x1)      |                       // DT low side value set
  //                  TCC_WEXCTRL_OTMX(0x0);                             // default output matrix config 
  */
  
  /*
  * Interrupts
  * 
  * Enable in INTENSET reg
  * Disable in INTENCLR reg
  * Status/CLR: INTFLAG reg
  * 
  * counter val continuously compared to each CC channel. When match, INTFLAG.MCx is set
  */
  /*
  TCC0->INTENSET.reg = TCC_INTENSET_OVF |                               // overflow fault
                    TCC_INTENSET_TRG |                                  // retigger
                    TCC_INTENSET_CNT |                                  // counter
                    TCC_INTENSET_ERR |                                  // error
                    TCC_INTENSET_UFS |                                  // non-recoverable update fault
                    TCC_INTENSET_FAULTA |                               // recoverable fault A/B
                    TCC_INTENSET_FAULTB |                      
                    TCC_INTENSET_FAULT0 |                               // non-recoverable fault 0/1
                    TCC_INTENSET_FAULT1 |  
                    TCC_INTENSET_MC(0)  |                               // Match or capture channel x
  */
  
  // TCCO->INTFLAG.reg                                                  // for reading interrupt status (pg 1866)
  // TCC0->STATUS.reg                                                   // reading tcc status (pg 1868)
  // TCC0->COUNT.reg                                                    // reading counter value, set CTRLBSET.CMD=READSYNC prior to read
                                                  
  
  // waveform generation option: WAVE.WAVEGEN
  // waveform output polarity WAVE.POL
  // RAMP operation: consider ramp2, CTRLBSET.IDXCMD
  // Count direction: CTRLB.DIR, When count reaches TOP or ZERO, INTFLAG.OVF set
  // waveform inversion: DRVCTRL.INVEN
  // sync required on read and write**
  /*
  * Single slope PWM:
  * 
  * R_PWM = log(TOP + 1)/log(2)
  * f_pwm = f_GCLK/(N*(TOP+1)), N is prescalar
  * 
  */
  TCC0->WAVE.reg = TCC_WAVE_WAVEGEN_NFRQ |                              // normal frequency operation
                   TCC_WAVE_RAMP_RAMP2   |                              // ramp 2 operation
                   TCC_WAVE_POL0;                                       // channel x polarity set
  while(TCC0->SYNCBUSY.bit.WAVE);
  
  
  // period reg
  TCC0->PER.reg = TCC_PER_PER(10);                                       // period value set
  //              TCC_PER_DITH4_DITHER(1) |                             // dithering cycle number
  //              TCC_PER_DITH4_PER(1)    |                             // period value set (if dithering enabled)
  while(TCC0->SYNCBUSY.bit.PER);
  
  // period buffer reg
  // sync required on read and write
  /*
  TCC0->PERBUF.reg = TCC_PERBUF_PERBUF(1);                              // value copied to PER on UPDATE condition
  //                 TCC_PERBUF_DITH4_DITHERBUF(1) |                    // dithering buffer update bits
  //                 TCC_PERBUF_DITH4_PERBUF(1)    |                    // period update if dithering enabled
  while(TCC00>SYNCBUSY.bit.PERBUF);
  */
  
  // capture compare reg
  // sync require on read and write
  
  TCC0->CC[ML_TCC0_CH0].reg = TCC_CC_CC(3);                             // CC value (18 bits)
  //              TCC_CC_DITH4_DITHER(1)                                // dithering cycle number
  //              TCC_CC_DITH4_CC(1)                                    // CC value (if dithering enabled)

  TCC0->CC[ML_TCC0_CH3].reg = TCC_CC_CC(3);
  
  while(TCC0->SYNCBUSY.bit.CC0 | TCC0->SYNCBUSY.bit.CC3);

  TCC0->DRVCTRL.reg |= TCC_DRVCTRL_INVEN3;                              // inverts ch3 wave, we want complimentary outs
  
  // Channel x CC buffer value regs: CCBUFx (force update w/ CTRLBSET.CMD=0x3)
  // capture compare buffer reg TCC0->CCBUF.reg, similar to PERBUF (pg 1882), NEEDS to wait for SYNC on read and write*

  const EPortType TCC0_PORT_GRP = g_APinDescription[ML_M4_TCC0_CH0_PIN].ulPort;
  
  const uint32_t TCC0_PIN_CH0 = g_APinDescription[ML_M4_TCC0_CH0_PIN].ulPin;

  // pin configuration reg
  PORT->Group[TCC0_PORT_GRP].PINCFG[TCC0_PIN_CH0].reg |=  PORT_PINCFG_PMUXEN;  // peripheral multiplexer enable
                                                    //PORT_PINCFG_DRVSTR   // enable stronger drive strength

  
  // shift TCC0_PIN right by one since PMUX arr length is 16, and PINCFG arr len is 32
  PORT->Group[TCC0_PORT_GRP].PMUX[TCC0_PIN_CH0 >> 1].reg |= PORT_PMUX_PMUXE(ML_TCC0_CH0_PMUXE); // peripheral multiplexer selection for even number pin (for odd: PORT_PMUX_PMUXO)

  // port dirset reg
  PORT->Group[TCC0_PORT_GRP].DIRSET.reg |= PORT_DIRSET_DIRSET(ML_PB12_PIN);                 // Set for output, else input

  // pretty much the same as CH0 pin setup
  const uint32_t TCC0_PIN_CH3 = g_APinDescription[ML_M4_TCC0_CH3_PIN].ulPin;

  PORT->Group[TCC0_PORT_GRP].PINCFG[TCC0_PIN_CH3].reg |= PORT_PINCFG_PMUXEN;

  PORT->Group[TCC0_PORT_GRP].PMUX[TCC0_PIN_CH3 >> 1].reg |= PORT_PMUX_PMUXO(ML_TCC0_CH3_PMUXO);

  PORT->Group[TCC0_PORT_GRP].DIRSET.reg |= PORT_DIRSET_DIRSET(ML_PB15_PIN);
  


  // TCC enable: set CTRLA.ENABLE
  TCC0->CTRLA.reg |= TCC_CTRLA_ENABLE;
  while(TCC0->SYNCBUSY.bit.ENABLE);
                                                                   
}

void setup() {

  GCLK0_init();
  TCC0_init();

}

void loop() {

}
