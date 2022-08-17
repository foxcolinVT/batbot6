/*
 * Author: Ben Westcott
 * Date created: 8/10/22
 */

#include "ML_M4_PlasmaDriver_NI.hpp"
#include "sine_tables.hpp"

void MCLK_init(void){

  // initial main clk division of 1
  MCLK->CPUDIV.reg = ML_MCLK_CPUDIV1;

  // enable AHB clock for DMAC
  MCLK->AHBMASK.reg |= MCLK_AHBMASK_DMAC;
}

void GCLK0_init(void){


  // wait for MCLK, 
  // while(!MCLK->INTFLAG.bit.CKRDY);                           

  GCLK->GENCTRL[0].reg = ML_GCLK_GENCTRL_DIV1 |                // GCLK divider, GCLK0_FREQ = ML_MCLK_UNDIV/(ML_MCLK_CPUDIV * ML_GCLK_GENCTRL_DIV)
                                  GCLK_GENCTRL_IDC |                    // 50/50 duty 
                                  ML_GCLK_GENCTRL_SRC_DPLL |            // Source multiplexer selects DPLL0 (phase locked loop)
                                  GCLK_GENCTRL_OE | GCLK_GENCTRL_GENEN; // Output enable, Generator enable

  // wait for GEN2 sync
  while(GCLK->SYNCBUSY.bit.GENCTRL0);

  // Set ADC1/0 in GCLK peripheral channel cntrl registers
  //GCLK->PCHCTRL[ADC1_GCLK_ID].reg = ML_GCLK2_PCHCTRL;
  //GCLK->PCHCTRL[ADC0_GCLK_ID].reg = ML_GCLK2_PCHCTRL;

  // Set DAC in GCLK PCHCTRL
  //GCLK->PCHCTRL[DAC_GCLK_ID].reg =  ML_GCLK2_PCHCTRL;

  // PCHCTRL for TCC0/1/2
  GCLK->PCHCTRL[TCC0_GCLK_ID].reg = ML_GCLK2_PCHCTRL;                   // TCC0 and TCC1 ID are the same, so TCC1 is implicitly set
  GCLK->PCHCTRL[TCC2_GCLK_ID].reg = ML_GCLK2_PCHCTRL;                     // TCC3 set for same above reason

}

static const EPortType TCC0_PORT_GRP = g_APinDescription[ML_TCC0_CH0_PIN].ulPort;
static const uint32_t TCC0_PIN_CH0 = g_APinDescription[ML_TCC0_CH0_PIN].ulPin;
static const uint32_t TCC0_PIN_CH1 = g_APinDescription[ML_TCC0_CH1_PIN].ulPin;

static void TCC0_PORT_init(void){

    // pin configuration reg
  PORT->Group[TCC0_PORT_GRP].PINCFG[TCC0_PIN_CH0].reg |=  PORT_PINCFG_PMUXEN;  // peripheral multiplexer enable
                                                    //PORT_PINCFG_DRVSTR   // enable stronger drive strength

  // shift TCC0_PIN right by one since PMUX arr length is 16, and PINCFG arr len is 32
  PORT->Group[TCC0_PORT_GRP].PMUX[TCC0_PIN_CH0 >> 1].reg |= ML_TCC0_CH0_PMUX_msk; // peripheral multiplexer selection for even number pin (for odd: PORT_PMUX_PMUXO)

  // port dirset reg
  PORT->Group[TCC0_PORT_GRP].DIRSET.reg |= PORT_DIRSET_DIRSET(ML_M4_TCC0_CH0_PIN);                 // Set for output, else input

  // pretty much the same as CH0 pin setup

  PORT->Group[TCC0_PORT_GRP].PINCFG[TCC0_PIN_CH1].reg |= PORT_PINCFG_PMUXEN;

  PORT->Group[TCC0_PORT_GRP].PMUX[TCC0_PIN_CH1 >> 1].reg |= ML_TCC0_CH1_PMUX_msk;

  PORT->Group[TCC0_PORT_GRP].DIRSET.reg |= PORT_DIRSET_DIRSET(ML_M4_TCC0_CH1_PIN);   

}


inline void TCC_enable(Tcc *tcc){
  tcc->CTRLA.reg |= TCC_CTRLA_ENABLE;
  while(tcc->SYNCBUSY.bit.ENABLE);
}

inline void TCC_disable(Tcc *tcc){
  tcc->CTRLA.reg &= ~TCC_CTRLA_ENABLE;
  while(tcc->SYNCBUSY.bit.ENABLE);
}

inline void TCC_swrst(Tcc *tcc){
  tcc->CTRLA.reg |= TCC_CTRLA_SWRST;
  while(tcc->SYNCBUSY.bit.SWRST);
}

void TCC0_init(void){
      
  // disable TCC
  TCC_disable(TCC0);
  
  // send software reset of TCC CTRLA.SWRST
  TCC_swrst(TCC0);

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
  TCC0->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV1 |
                    TCC_CTRLA_PRESCSYNC_PRESC;                         // try TC_CTRLA_PRESYNC_GCLK or TC_CTRLA_PRESYNC_RESYNC
   //                 TCC_CTRLA_DMAOS;     
   //               TCC_CTRLA_RESOLUTION_DITH4 |                           // enable dithering every 16 PWM frames
    //              TCC_CTRLA_RUNSTDBY;                                    // run TCC when MP in standby
   
  //TCC0->CTRLA.reg &= ~TCC_CTRLA_DMAOS;
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

 //TCC0->INTENSET.reg = TCC_INTENSET_OVF;
 //NVIC_EnableIRQ(TCC0_0_IRQn);
 //NVIC_SetPriority(TCC0_0_IRQn, 0);

  
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
  TCC0->WAVE.reg = TCC_WAVE_WAVEGEN_NFRQ |                             // normal frequency operation
                   TCC_WAVE_RAMP_RAMP2   |                              // ramp 2 operation
                   TCC_WAVE_POL0 | TCC_WAVE_POL1;                                      // channel x polarity set
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
  
  TCC0->CC[ML_TCC0_CH0].reg = TCC_CC_CC(5);                             // CC value (18 bits)
  //              TCC_CC_DITH4_DITHER(1)                                // dithering cycle number
  //              TCC_CC_DITH4_CC(1)                                    // CC value (if dithering enabled)

  TCC0->CC[ML_TCC0_CH1].reg = TCC_CC_CC(5);

  while(TCC0->SYNCBUSY.bit.CC0 | TCC0->SYNCBUSY.bit.CC1);

  TCC0_PORT_init();

  //TCC0->DRVCTRL.reg |= TCC_DRVCTRL_INVEN1;                              // inverts ch3 wave, we want complimentary outs
  
  // Channel x CC buffer value regs: CCBUFx (force update w/ CTRLBSET.CMD=0x3)
  // capture compare buffer reg TCC0->CCBUF.reg, similar to PERBUF (pg 1882), NEEDS to wait for SYNC on read and write*                                                              
}

inline void TCC0_DT_set(uint8_t dth, uint8_t dtl){
  TCC0->WEXCTRL.reg |= (TCC_WEXCTRL_DTIEN0 | TCC_WEXCTRL_DTIEN1 | TCC_WEXCTRL_DTLS(dtl) | TCC_WEXCTRL_DTHS(dth));
}

// mode: 4, 5, 6
void TCC0_DITH_set(char mode, uint64_t cycles, uint64_t period, uint64_t compare){

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
  while(TCC0->SYNCBUSY.bit.PER);

  TCC0->CC[ML_TCC0_CH0].reg = CC_DITH_msk;
  TCC0->CC[ML_TCC0_CH1].reg = CC_DITH_msk;
  while(TCC0->SYNCBUSY.bit.CC0 | TCC0->SYNCBUSY.bit.CC1);
}

static const EPortType TCC2_PORT_GRP = g_APinDescription[ML_TCC2_CH1_PIN].ulPort;
static const uint32_t TCC2_PIN_CH1 = g_APinDescription[ML_TCC2_CH1_PIN].ulPin;

static void TCC2_PORT_init(void){

  PORT->Group[TCC2_PORT_GRP].PINCFG[TCC2_PIN_CH1].reg |= PORT_PINCFG_PMUXEN;

  PORT->Group[TCC2_PORT_GRP].PMUX[TCC2_PIN_CH1 >> 1].reg |= ML_TCC2_CH1_PMUX_msk;

  PORT->Group[TCC2_PORT_GRP].DIRSET.reg |= PORT_DIRSET_DIRSET(ML_M4_TCC2_CH1_PIN);

}

void TCC2_init(void){
    // disable TCC
  TCC_disable(TCC2);
  
  // send software reset of TCC CTRLA.SWRST
  TCC_swrst(TCC2);

  
  TCC2->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV4 |
                    TCC_CTRLA_PRESCSYNC_PRESC;                         
   
  // When one-shot mode set to 0, TCC will generate DMA request on OVF trigger
  TCC2->CTRLA.reg &= ~TCC_CTRLA_DMAOS;
                                    

  TCC2->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;                      
  while(TCC2->SYNCBUSY.bit.WAVE);
  
  
  // period reg
  TCC2->PER.reg = TCC_PER_PER(10);   
  while(TCC2->SYNCBUSY.bit.PER);
  
  
  TCC2->CC[ML_TCC2_CH1].reg = TCC_CC_CC(5);
  while(TCC2->SYNCBUSY.bit.CC1);

  TCC2->INTENSET.reg = TCC_INTENSET_OVF;
  NVIC_EnableIRQ(TCC2_1_IRQn);
  NVIC_SetPriority(TCC2_1_IRQn, 0);

  TCC2_PORT_init();

  //TCC0->DRVCTRL.reg |= TCC_DRVCTRL_INVEN1;   
}

void TCC2_1_Handler(void){
  Serial.print("y");
}

static DmacDescriptor base_descriptor[12] __attribute__((aligned(16)));
static DmacDescriptor descriptor __attribute__((aligned(16)));
static volatile DmacDescriptor wb_descriptor[12] __attribute__((aligned(16)));

inline void DMAC_enable(void){
  DMAC->CTRL.reg |= DMAC_CTRL_DMAENABLE;
}

inline void DMAC_swrst(void){
    DMAC->CTRL.reg |= DMAC_CTRL_SWRST;
}

inline void DMAC_disable(void){
    DMAC->CTRL.reg &= ~DMAC_CTRL_DMAENABLE;
}

inline void DMAC_CH_enable(char ch){
  DMAC->Channel[ch].CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;
}

void DMAC_init(void){

  // disable and software reset 
  DMAC_disable();
  DMAC_swrst();

  // SRAM addr of descriptor memory section written to BASEADDR reg
  DMAC->BASEADDR.reg = (uint32_t) &base_descriptor;

  // SRAM addr of write-back section written to WRBADDR reg
  DMAC->WRBADDR.reg = (uint32_t) &wb_descriptor;

  DMAC_enable();

  // Set priority level x by setting CTRL.LVLENx
  DMAC->CTRL.reg |= DMAC_CTRL_LVLEN0 | DMAC_CTRL_LVLEN1 | DMAC_CTRL_LVLEN2 | DMAC_CTRL_LVLEN3;

  /*
       DMA channel intialization
  */

 // channel number of DMA channel written to CHCTRLA reg
 // trigger action set by writing to CHCTRLA.TRIGACT
 // trigger source set by writing to CHCTRLA.TRIGSRC
  DMAC->Channel[ML_DMAC_CHIRP_CH].CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;
  DMAC->Channel[ML_DMAC_CHIRP_CH].CHCTRLA.reg |= DMAC_CHCTRLA_SWRST;

  DMAC->Channel[ML_DMAC_CHIRP_CH].CHCTRLA.reg |= (DMAC_CHCTRLA_BURSTLEN_SINGLE |
                                                  DMAC_CHCTRLA_TRIGACT_BLOCK  |      
                                                  DMAC_CHCTRLA_TRIGSRC(ML_DMAC_TCC1_OVF_TRIG));


 // set channel priority to zero
  DMAC->Channel[ML_DMAC_CHIRP_CH].CHPRILVL.reg |=  DMAC_CHPRILVL_PRILVL_LVL0; 

 // descriptor setup
  descriptor.BTCTRL.reg = DMAC_BTCTRL_VALID          |            // Indicates that descriptor should be used
                          DMAC_BTCTRL_EVOSEL_DISABLE |            // disable event selection
                          DMAC_BTCTRL_BLOCKACT_NOACT |            // dont act after block transfer is completed, channel disables (pg. 445)
                          DMAC_BTCTRL_BEATSIZE_BYTE  |            // size of a single beat. Could also be HWORD or WORD
                          DMAC_BTCTRL_SRCINC         |            // allow source address pointer to increment
                          DMAC_BTCTRL_STEPSIZE_X1;                // NEXT ADDR = ADDR + 1 * (beat size in bytes)
  
  // number of beats, ie bytes in our case, to send
  descriptor.BTCNT.reg = SINE_LEN_ALT;

  // Location in memory of data to send. SRCADDR accepts last memory location of data
  descriptor.SRCADDR.reg = (uint32_t)&SINE_WAVE_TBL_ALT[0] + SINE_LEN_ALT * sizeof(uint8_t);
 
  // Send values to TCC0 count register, more specifically the update buffer, 
  // so byte placed in CCBUF from DMAC will be seen in CC reg next clock cycle
  descriptor.DSTADDR.reg = (uint32_t)&TCC1->CCBUF[0].reg;

  // Where to point to when BTCNT is reached, so point back to beginning
  descriptor.DESCADDR.reg = (uint32_t) &base_descriptor[0];

  // copy setup descriptor ptr into base descriptor allocation
  memcpy(&base_descriptor[0], &descriptor, sizeof(DmacDescriptor));
}


bool results_ready = false;
uint32_t CC_results = 0;

void TCC0_0_Handler(void){

  uint32_t cc_intreg_cpy = TCC0->INTFLAG.reg;
  //Serial.println("YYY");

  // make a copy of intflag reg
 //TCC_INTFLAG_Type intflag_cpy = TCC0->CC[0].INTFLAG;

  //TCC0->CTRLBSET.reg |= TCC_CTRLBSET_ONESHOT;

  // clear interrupt flags
  TCC0->INTFLAG.reg |= TCC_INTFLAG_OVF;

  if(cc_intreg_cpy & TCC_INTFLAG_OVF){
    results_ready = true;
  }
}


void setup() {

  Serial.begin(115200);

  MCLK_init();
  GCLK0_init();

  TCC0_init();
  TCC2_init();

  DMAC_init();

  //TCC0_DITH_set(4, 12, 10, 3);
  //TCC0_DT_set(0x0, 0x2);

  TCC_enable(TCC0);
  TCC_enable(TCC2);
  //DMAC_CH_enable(ML_DMAC_CHIRP_CH);

}

void loop() {

 // delay(10);
  if(results_ready){


        //Serial.println(TCC0->CCBUF[0].bit.CCBUF);
        results_ready = false;
  }
  //Serial.print("T");

}
