/*
 * Author: Ben Westcott
 * Date created: 8/10/22
 */

#include <Arduino.h>

//#define ML_M4EXPRESS

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
  GCLK->PCHCTRL[DAC_GCLK_ID].reg =  ML_GCLK2_PCHCTRL;

  // PCHCTRL for TCC0/1/2
  GCLK->PCHCTRL[TCC0_GCLK_ID].reg = ML_GCLK2_PCHCTRL;                   // TCC0 and TCC1 ID are the same, so TCC1 is implicitly set
  //GCLK->PCHCTRL[TCC2_GCLK_ID].reg = ML_GCLK2_PCHCTRL;                     // TCC3 set for same above reason

}

#define GCLK0_FREQ (ML_MCLK_UNDIV/(1 * ML_HCLK_INITIAL_DIV))             // 60 Mhz

#define ML_TCC0_CH0 0x0
#define ML_TCC0_CH1 0x1

#ifdef ML_M4EXPRESS

#define ML_TCC0_CH0_PIN 7
#define ML_M4_TCC0_CH0_PIN 12
#define ML_TCC0_CH0_PMUX 0x6
#define ML_TCC0_CH0_PMUX_msk (PORT_PMUX_PMUXE(ML_TCC0_CH0_PMUX))

#define ML_TCC0_CH1_PIN 4
#define ML_M4_TCC0_CH1_PIN 13
#define ML_TCC0_CH1_PMUX 0x6
#define ML_TCC0_CH1_PMUX_msk (PORT_PMUX_PMUXO(ML_TCC0_CH1_PMUX))

#else

#define ML_TCC0_CH0_PIN 45
#define ML_M4_TCC0_CH0_PIN 10
#define ML_TCC0_CH0_PMUX 0x5                        // pin # = 2*peripheral (even), 2*peripheral + 1 (odd)
#define ML_TCC0_CH0_PMUX_msk (PORT_PMUX_PMUXE(ML_TCC0_CH0_PMUX))

#define ML_TCC0_CH1_PIN 44
#define ML_M4_TCC0_CH1_PIN 11
#define ML_TCC0_CH1_PMUX 0x5
#define ML_TCC0_CH1_PMUX_msk (PORT_PMUX_PMUXO(ML_TCC0_CH1_PMUX))

#endif

const EPortType TCC0_PORT_GRP = g_APinDescription[ML_TCC0_CH0_PIN].ulPort;
const uint32_t TCC0_PIN_CH0 = g_APinDescription[ML_TCC0_CH0_PIN].ulPin;
const uint32_t TCC0_PIN_CH1 = g_APinDescription[ML_TCC0_CH1_PIN].ulPin;

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


static inline void TCC_enable(Tcc *tcc){
  tcc->CTRLA.reg |= TCC_CTRLA_ENABLE;
  while(tcc->SYNCBUSY.bit.ENABLE);
}

static inline void TCC_disable(Tcc *tcc){
  tcc->CTRLA.reg &= ~TCC_CTRLA_ENABLE;
  while(tcc->SYNCBUSY.bit.ENABLE);
}

static inline void TCC_swrst(Tcc *tcc){
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
  TCC0->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV2 |
                    TCC_CTRLA_PRESCSYNC_PRESC |                          // try TC_CTRLA_PRESYNC_GCLK or TC_CTRLA_PRESYNC_RESYNC
                    TCC_CTRLA_DMAOS;
     //               TCC_CTRLA_RESOLUTION_DITH4 |                           // enable dithering every 16 PWM frames
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

 TCC0->INTENSET.reg = TCC_INTENSET_OVF;
 NVIC_EnableIRQ(TCC0_0_IRQn);

  
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
  TCC0->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM |                              // normal frequency operation
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
  
  TCC0->CC[ML_TCC0_CH0].reg = TCC_CC_CC(3);                             // CC value (18 bits)
  //              TCC_CC_DITH4_DITHER(1)                                // dithering cycle number
  //              TCC_CC_DITH4_CC(1)                                    // CC value (if dithering enabled)

  TCC0->CC[ML_TCC0_CH1].reg = TCC_CC_CC(3);

  while(TCC0->SYNCBUSY.bit.CC0 | TCC0->SYNCBUSY.bit.CC1);

  TCC0_PORT_init();

  //TCC0->DRVCTRL.reg |= TCC_DRVCTRL_INVEN1;                              // inverts ch3 wave, we want complimentary outs
  
  // Channel x CC buffer value regs: CCBUFx (force update w/ CTRLBSET.CMD=0x3)
  // capture compare buffer reg TCC0->CCBUF.reg, similar to PERBUF (pg 1882), NEEDS to wait for SYNC on read and write*                                                              
}

static void TCC0_DT_set(uint8_t dth, uint8_t dtl){
  TCC0->WEXCTRL.reg |= (TCC_WEXCTRL_DTIEN0 | TCC_WEXCTRL_DTIEN1 | TCC_WEXCTRL_DTLS(dtl) | TCC_WEXCTRL_DTHS(dth));
}


// mode: 4, 5, 6
static void TCC0_DITH_set(char mode, uint64_t cycles, uint64_t period, uint64_t compare){

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

#define ML_DMAC_CHIRP_CH 0

#define ML_DMAC_TCC0_OVF_TRIG 0x16

static DmacDescriptor base_descriptor[12] __attribute__((aligned(16)));
static DmacDescriptor descriptor __attribute__((aligned(16)));
static volatile DmacDescriptor wb_descriptor[12] __attribute__((aligned(16)));


static const uint8_t buffer_tx[20] = {
	0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A,
	0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14,
};

static void DMAC_init(void){

  DMAC->CTRL.reg &= ~DMAC_CTRL_DMAENABLE;
  DMAC->CTRL.reg |= DMAC_CTRL_SWRST;

  // SRAM addr of descriptor memory section written to BASEADDR reg
  DMAC->BASEADDR.reg = (uint32_t) &base_descriptor;

  // SRAM addr of write-back section written to WRBADDR reg
  DMAC->WRBADDR.reg = (uint32_t) &wb_descriptor;

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

  DMAC->Channel[ML_DMAC_CHIRP_CH].CHCTRLA.reg |= (DMAC_CHCTRLA_BURSTLEN_2BEAT |
                                             DMAC_CHCTRLA_TRIGACT_BURST  |      
                                             DMAC_CHCTRLA_TRIGSRC(ML_DMAC_TCC0_OVF_TRIG));


 // set channel priority to zero
  DMAC->Channel[ML_DMAC_CHIRP_CH].CHPRILVL.reg |=  DMAC_CHPRILVL_PRILVL_LVL0; 

 // descriptor setup
  descriptor.BTCTRL.reg = DMAC_BTCTRL_VALID          | 
                              DMAC_BTCTRL_EVOSEL_DISABLE | 
                              DMAC_BTCTRL_BLOCKACT_NOACT |
                              DMAC_BTCTRL_BEATSIZE_BYTE  |
                              DMAC_BTCTRL_SRCINC         |
                              DMAC_BTCTRL_STEPSIZE_X1;
  
  descriptor.BTCNT.reg = sizeof(buffer_tx);
  descriptor.SRCADDR.reg = (uint32_t)&buffer_tx[0] + sizeof(buffer_tx);
  descriptor.DSTADDR.reg = (uint32_t)&TCC0->CCBUF[0].reg;
  descriptor.DESCADDR.reg = (uint32_t) &base_descriptor[0];

  memcpy(&base_descriptor[0], &descriptor, sizeof(DmacDescriptor));

}

static void DMAC_enable(void){
  DMAC->CTRL.reg |= DMAC_CTRL_DMAENABLE;
  DMAC->Channel[ML_DMAC_CHIRP_CH].CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;
}

bool results_ready = false;
uint32_t CC_results = 0;

void TCC0_0_Handler(void){

  TCC0->CTRLBSET.reg |= TCC_CTRLBSET_CMD_READSYNC;

  uint32_t cc_intreg_cpy = TCC0->INTFLAG.reg;

  // make a copy of intflag reg
 //TCC_INTFLAG_Type intflag_cpy = TCC0->CC[0].INTFLAG;

  // clear interrupt flags
  TCC0->INTFLAG.reg = TCC_INTFLAG_RESETVALUE;

  if(cc_intreg_cpy & TCC_INTFLAG_OVF){
    results_ready = true;
    CC_results = TCC0->CC[0].bit.CC;
  }
}


void setup() {

  //Serial.begin(9600);

  GCLK0_init();
  TCC0_init();

  DMAC_init();

  //TCC0_DITH_set(4, 12, 10, 3);
  //TCC0_DT_set(0x0, 0x2);

  TCC_enable(TCC0);
  DMAC_enable();

}

void loop() {

  if(results_ready){
    //    Serial.println(CC_results);
        results_ready = false;
  }
 // Serial.print("T");

}
