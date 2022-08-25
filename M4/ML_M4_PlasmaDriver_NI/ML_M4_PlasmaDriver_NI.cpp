/*
 * Author: Ben Westcott
 * Date created: 8/10/22
 */

#include "header/ML_M4_PlasmaDriver_NI.hpp"
#include "header/sine_tables.hpp"
#include "header/utils.hpp"

void MCLK_init(void){

  // initial main clk division of 1
  MCLK->CPUDIV.reg = ML_MCLK_CPUDIV1;

  // enable AHB clock for DMAC
  MCLK->AHBMASK.reg |= MCLK_AHBMASK_DMAC;
  MCLK->APBCMASK.reg |= MCLK_APBCMASK_CCL;
}

void GCLK_init(void){


  // wait for MCLK, 
  // while(!MCLK->INTFLAG.bit.CKRDY);                           

  GCLK->GENCTRL[ML_GCLK_CH].reg = GCLK_GENCTRL_DIV(1) |                // GCLK divider, GCLK7_FREQ = ML_MCLK_UNDIV/(ML_MCLK_CPUDIV * ML_GCLK_GENCTRL_DIV)
                                  GCLK_GENCTRL_IDC |                    // 50/50 duty 
                                  GCLK_GENCTRL_SRC_DPLL0 |            // Source multiplexer selects DPLL0 (phase locked loop)
                                  GCLK_GENCTRL_GENEN; // Output enable, Generator enable

  // wait for GEN7 sync
  while(GCLK->SYNCBUSY.bit.GENCTRL7);



  // Set ADC1/0 in GCLK peripheral channel cntrl registers
  //GCLK->PCHCTRL[ADC1_GCLK_ID].reg = ML_GCLK2_PCHCTRL;
  //GCLK->PCHCTRL[ADC0_GCLK_ID].reg = ML_GCLK2_PCHCTRL;

  // Set DAC in GCLK PCHCTRL
  // GCLK->PCHCTRL[DAC_GCLK_ID].reg =  ML_GCLK2_PCHCTRL;

  // PCHCTRL for TCC0/1/2
  GCLK->PCHCTRL[TCC0_GCLK_ID].reg = ML_GCLK7_PCHCTRL;                  // TCC0 and TCC1 ID are the same, so TCC1 is implicitly set
  
  GCLK->PCHCTRL[CCL_GCLK_ID].reg = ML_GCLK7_PCHCTRL;

  GCLK->PCHCTRL[ADC0_GCLK_ID].reg = ML_GCLK7_PCHCTRL;
  GCLK->PCHCTRL[ADC1_GCLK_ID].reg = ML_GCLK7_PCHCTRL;
  // GCLK->PCHCTRL[TCC2_GCLK_ID].reg = ML_GCLK2_PCHCTRL;                     // TCC3 set for same above reason

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
   //               TCC_CTRLA_DMAOS;     
   //               TCC_CTRLA_RESOLUTION_DITH4 |                           // enable dithering every 16 PWM frames
   //               TCC_CTRLA_RUNSTDBY;                                    // run TCC when MP in standby
   
  // TCC0->CTRLA.reg &= ~TCC_CTRLA_DMAOS;
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
  // NVIC_EnableIRQ(TCC0_0_IRQn);
  // NVIC_SetPriority(TCC0_0_IRQn, 0);

  
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
  //TCC_sync(TCC0);
  
  
  // period reg
  TCC0->PER.reg = TCC_PER_PER(100);                                       // period value set
  //              TCC_PER_DITH4_DITHER(1) |                             // dithering cycle number
  //              TCC_PER_DITH4_PER(1)    |                             // period value set (if dithering enabled)

  //TCC_sync(TCC0);
  
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
  
  //TCC0->CC[ML_TCC0_CH0].reg = TCC_CC_CC(50);                             // CC value (18 bits)
  //              TCC_CC_DITH4_DITHER(1)                                // dithering cycle number
  //              TCC_CC_DITH4_CC(1)                                    // CC value (if dithering enabled)

  //TCC0->CC[ML_TCC0_CH1].reg = TCC_CC_CC(50);

  //TCC_sync(TCC0);

  //TCC0_PORT_init();
  
  // TCC0->DRVCTRL.reg |= TCC_DRVCTRL_INVEN1;                              // inverts ch3 wave, we want complimentary outs
  
  // Channel x CC buffer value regs: CCBUFx (force update w/ CTRLBSET.CMD=0x3)
  // capture compare buffer reg TCC0->CCBUF.reg, similar to PERBUF (pg 1882), NEEDS to wait for SYNC on read and write*                                                              
}

void TCC0_0_Handler(void){}

void TCC1_init(void){
  // disable TCC
  TCC_disable(TCC1);
  
  // send software reset of TCC CTRLA.SWRST
  TCC_swrst(TCC1);
  
  TCC1->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV1 |
                    TCC_CTRLA_PRESCSYNC_PRESC;                         
   
  // When one-shot mode set to 0, TCC will generate DMA request on OVF trigger
  TCC1->CTRLA.reg &= ~TCC_CTRLA_DMAOS;
                                    

  TCC1->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;                      
  //TCC_sync(TCC1);
  
  
  // period reg
  // We want a full duty cycle range out of ch3 from values given from DMA
  // Thus, make the TCC period = the maximum amplitude of the wavetable which is 0xff
  // TCC1->PER.reg = TCC_PER_PER(ML_TCC1_CH3_INITIAL_PER);   
  TCC1->PER.reg = TCC_PER_PER(256);   
  TCC_sync(TCC1);
  
  // start timer @ 50% duty cycle which would mean counting to half the period
  // TCC1->CC[ML_TCC1_CH3].reg = TCC_CC_CC((unsigned)(ML_TCC1_CH3_INITIAL_PER / 2));
  //TCC1->CC[ML_TCC1_CH3].reg = TCC_CC_CC(128);

  //TCC1->CC[0].reg = TCC_CC_CC(128);
  //TCC_sync(TCC1);

  //TCC1_PORT_init();

   //TCC1->INTENSET.reg = TCC_INTENSET_OVF;
   //NVIC_EnableIRQ(TCC1_0_IRQn);
   //NVIC_SetPriority(TCC1_0_IRQn, 0);

  // TCC0->DRVCTRL.reg |= TCC_DRVCTRL_INVEN1;   
}


void TCC1_0_Handler(void){}

static DmacDescriptor base_descriptor[12] __attribute__((aligned(16)));
static DmacDescriptor descriptor[12] __attribute__((aligned(16)));
static volatile DmacDescriptor wb_descriptor[12] __attribute__((aligned(16)));

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
 /* DMAC_CH_disable((uint8_t)ML_DMAC_CHIRP_CH);
  DMAC_CH_swrst((uint8_t)ML_DMAC_CHIRP_CH);

  DMAC->Channel[ML_DMAC_CHIRP_CH].CHCTRLA.reg |= (DMAC_CHCTRLA_BURSTLEN_SINGLE |
                                                  DMAC_CHCTRLA_TRIGACT_BLOCK  |      
                                                  DMAC_CHCTRLA_TRIGSRC(ML_DMAC_TCC1_OVF_TRIG));


 // set channel priority to zero
  DMAC->Channel[ML_DMAC_CHIRP_CH].CHPRILVL.reg |=  DMAC_CHPRILVL_PRILVL_LVL0; */
}

void DMAC_CH_init(const uint8_t chnum, const uint32_t chsettings, const uint8_t prilvl){

  DMAC_CH_disable(chnum);
  DMAC_CH_swrst(chnum);

  DMAC->Channel[chnum].CHCTRLA.reg = chsettings;

  DMAC->Channel[chnum].CHPRILVL.reg = prilvl;

}

void DMAC_chirp_descriptor_init(const uint16_t btsettings, const uint16_t btcnt, const uint32_t srcaddr, const uint32_t dstaddr, const uint8_t bdindex){

   // descriptor setup
  /*descriptor.BTCTRL.reg = DMAC_BTCTRL_VALID          |            // Indicates that descriptor should be used
                          DMAC_BTCTRL_EVOSEL_DISABLE |            // disable event selection
                          DMAC_BTCTRL_BLOCKACT_NOACT |            // dont act after block transfer is completed, channel disables (pg. 445)
                          DMAC_BTCTRL_BEATSIZE_HWORD |            // size of a single beat. Could also be HWORD or WORD
                          DMAC_BTCTRL_SRCINC         |            // allow source address pointer to increment
                          DMAC_BTCTRL_STEPSIZE_X1;                // NEXT ADDR = ADDR + 1 * (beat size in bytes)
  */

  descriptor[bdindex].BTCTRL.reg = btsettings;
  // number of beats, ie bytes in our case, to send
  descriptor[bdindex].BTCNT.reg = btcnt;

  // Location in memory of data to send. SRCADDR accepts last memory location of data
  //descriptor.SRCADDR.reg = (uint32_t)&SINE_WAVE_TBL_ALT[0] + SINE_LEN_ALT * sizeof(uint8_t);
  descriptor[bdindex].SRCADDR.reg = srcaddr;

  // Send values to TCC0 count register, more specifically the update buffer, 
  // so byte placed in CCBUF from DMAC will be seen in CC reg next clock cycle
  descriptor[bdindex].DSTADDR.reg = dstaddr;

  // Where to point to when BTCNT is reached, so point back to beginning
  descriptor[bdindex].DESCADDR.reg = (uint32_t) &base_descriptor[bdindex];

  // copy setup descriptor ptr into base descriptor allocation
  memcpy(&base_descriptor[bdindex], &descriptor[bdindex], sizeof(DmacDescriptor));

}

/*

    TRUTH TABLE: y = BUF(x) = x
    in[2] | in[1] | in[0] | out
    ---------------------------
       0  |   0   |   0   | T[0] = 0
       0  |   0   |   1   | T[1] = 0
       0  |   1   |   0   | T[2] = 0
       0  |   1   |   1   | T[3] = 0
       1  |   0   |   0   | T[4] = 1
       1  |   0   |   1   | T[5] = 1
       1  |   1   |   0   | T[6] = 1
       1  |   1   |   1   | T[7] = 1
*/
const static uint8_t truth_buffer = 0b11110000;

/*
    TRUTH TABLE: y = AND(x1, x2, X)
    in[2] | in[1] | in[0] | out
    ---------------------------
       0  |   0   |   0   | T[0] = 0
       0  |   0   |   1   | T[1] = 0
       0  |   1   |   0   | T[2] = 0
       0  |   1   |   1   | T[3] = 0
       1  |   0   |   0   | T[4] = 0
       1  |   0   |   1   | T[5] = 0
       1  |   1   |   0   | T[6] = 1
       1  |   1   |   1   | T[7] = 1
*/
const static uint8_t truth_and = 0b110000;

void CCL_init(void){

  CCL_disable();
  CCL_swrst();

  CCL_CH_disable((uint8_t)ML_CCL_LUT_BUF_CH);
  CCL_CH_disable((uint8_t)ML_CCL_LUT_AND_CH);

  CCL->LUTCTRL[ML_CCL_LUT_BUF_CH].reg |= CCL_LUTCTRL_TRUTH(truth_buffer)  |
                                          CCL_LUTCTRL_INSEL0_TCC          |
                                          CCL_LUTCTRL_INSEL1_MASK         |
                                          CCL_LUTCTRL_INSEL2_MASK         |
                                          CCL_LUTCTRL_FILTSEL_SYNCH       |
                                          CCL_LUTCTRL_EDGESEL;

  //CCL->LUTCTRL[ML_CCL_LUT_BUF_CH].reg &= ~CCL_LUTCTRL_LUTEI | ~CCL_LUTCTRL_LUTEO;
                                  

  
  CCL->LUTCTRL[ML_CCL_LUT_AND_CH].reg |= CCL_LUTCTRL_TRUTH(truth_and)     |
                                         CCL_LUTCTRL_INSEL0_TCC           |
                                         CCL_LUTCTRL_INSEL1_LINK          |
                                         CCL_LUTCTRL_INSEL2_MASK          |
                                         CCL_LUTCTRL_FILTSEL_SYNCH        |
                                         CCL_LUTCTRL_EDGESEL;
  
  //CCL->LUTCTRL[ML_CCL_LUT_AND_CH].reg &= ~CCL_LUTCTRL_LUTEI | ~CCL_LUTCTRL_LUTEO;

  //CCL_CH_enable((uint8_t)ML_CCL_LUT_AND_CH);
  //CCL_CH_enable((uint8_t)ML_CCL_LUT_BUF_CH);
  
}

void ADC_init(Adc *ADCx, const unsigned muxneg_ain, const unsigned muxpos_ain){

  // Write protected registers: CTRLA (except ENABLE and SWRT bits), EVCTRL, CALIB

  ADC_disable(ADCx);

  ADC_swrst(ADCx);

  // f_adc = f_gclk7/128 = 1Mhz
  ADCx->CTRLA.reg = ADC_CTRLA_PRESCALER_DIV128;

  ADCx->CTRLB.reg = ADC_CTRLB_RESSEL_12BIT |        // 12-bit resolution
                    ADC_CTRLB_FREERUN;              // start new conversion when previous completes

  // set positive and negative mux inputs
  ADCx->INPUTCTRL.reg = muxneg_ain | muxpos_ain;

  // sampling time = (SAMPLEN + 1) * (CLK_ADC) -> ST = CLK_ADC
  ADCx->SAMPCTRL.reg = ADC_SAMPCTRL_SAMPLEN(0x0);

  // average control
  ADCx->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1 |    // number of samples added together
                      ADC_AVGCTRL_ADJRES(0x0);     // division coefficient: 2^n steps

  ADCx->REFCTRL.reg = ADC_REFCTRL_REFSEL_INTVCC1;  // set reference voltage to VDDANA

  //ADCx->INTENSET.reg = ADC_INTENSET_RESRDY;
  //NVIC_SetPriority(ADC0_1_IRQn, 0);
  //NVIC_EnableIRQ(ADC0_1_IRQn);
  
  // all of the above regs are write-protected
  ADC_sync(ADCx);
}


void ADC0_1_Handler(void){}



const int sample_freq = 1E6;
const double chirp_duration = 5E-3;
const uint16_t num_samples = sample_freq * chirp_duration;

uint16_t chirp_out_buffer[num_samples];
uint16_t chirp_right_in_buffer[num_samples];
uint16_t chirp_left_in_buffer[num_samples];

uint32_t generate_chirp(void){

  const double t1 = chirp_duration;

  const double phi = 0;

  const int f0 = 105E3;
  const int f1 = 5E3;

  const double k = (f1 - f0) / t1;

  for(int i = num_samples - 1; i >= 0; --i){

    // time step
    double t = t1 * ((double)i / num_samples);

    // time domain model for a bat chirp
    double chirp = cos(phi + 2*PI * (f0 * t + k/2 * t * t));

    // Hanning window - remove harmonics from frequency distribution, leave fundemental frequencies
    double window = 0.5 * (1 - cos(2*PI * i/(num_samples - 1)));

    // fill DMA buffer 
    chirp_out_buffer[i] = (uint16_t)((256/2) * (1 + chirp * window));
  }
  return (uint32_t)&chirp_out_buffer[0] + num_samples * sizeof(uint16_t);
}

void setup() {

  MCLK_init();
  GCLK_init();

  TCC0_init();

  TCC_CH_CC_set(TCC0, ML_TCC0_CH0, 50);
  TCC_CH_CC_set(TCC0, ML_TCC0_CH1, 50);

  peripheral_port_init(ML_TCC0_CH0_PMUX_msk, ML_TCC0_CH0_PIN, OUT, DRV_OFF);
  peripheral_port_init(ML_TCC0_CH1_PMUX_msk, ML_TCC0_CH1_PIN, OUT, DRV_OFF);

  TCC1_init();

  TCC_CH_CC_set(TCC1, ML_TCC1_CH3, 128);

  peripheral_port_init(ML_TCC1_CH3_PMUX_msk, ML_TCC1_CH3_PIN, OUT, DRV_OFF);
  peripheral_port_init(ML_TCC1_CH0_PMUX_msk, ML_TCC1_CH0_PIN, OUT, DRV_OFF);

  ADC_init(ADC0,
           ADC_INPUTCTRL_MUXNEG_GND,
           ADC_INPUTCTRL_MUXPOS(get_ADC_channel_num(ML_ADC0_AIN5_PIN)));

  ADC_init(ADC1,
           ADC_INPUTCTRL_MUXNEG_GND,
           ADC_INPUTCTRL_MUXPOS(get_ADC_channel_num(ML_ADC1_AIN6_PIN)));

  //CCL_init()

  DMAC_init();

  uint32_t chirp_out_cs = DMAC_CHCTRLA_BURSTLEN_SINGLE |
                          DMAC_CHCTRLA_TRIGACT_BLOCK  |      
                          DMAC_CHCTRLA_TRIGSRC(ML_DMAC_TCC1_OVF_TRIG);

  DMAC_CH_init(ML_DMAC_CHIRP_OUT_CH, chirp_out_cs, DMAC_CHPRILVL_PRILVL_LVL0);

  uint32_t chirp_in_cs = DMAC_CHCTRLA_BURSTLEN_SINGLE |
                         DMAC_CHCTRLA_TRIGACT_BLOCK;

  DMAC_CH_init(ML_DMAC_CHIRP_RIGHT_IN_CH, (chirp_in_cs | DMAC_CHCTRLA_TRIGSRC(ML_DMAC_ADC0_RESRDY_TRIG)), DMAC_CHPRILVL_PRILVL_LVL1);
  DMAC_CH_init(ML_DMAC_CHIRP_LEFT_IN_CH, (chirp_in_cs | DMAC_CHCTRLA_TRIGSRC(ML_DMAC_ADC1_RESRDY_TRIG)), DMAC_CHPRILVL_PRILVL_LVL1);
  
  uint32_t chirp_out_srcaddr = generate_chirp();

  uint16_t chirp_out_ds = DMAC_BTCTRL_VALID |
                          DMAC_BTCTRL_EVOSEL_DISABLE |
                          DMAC_BTCTRL_BLOCKACT_NOACT |
                          DMAC_BTCTRL_BEATSIZE_HWORD |
                          DMAC_BTCTRL_SRCINC        |
                          DMAC_BTCTRL_STEPSIZE_X1;
  
  DMAC_chirp_descriptor_init(chirp_out_ds, num_samples, chirp_out_srcaddr, (uint32_t)&TCC1->CCBUF[ML_TCC1_CH3].reg, 0);

  uint16_t chirp_in_ds = DMAC_BTCTRL_VALID |
                         DMAC_BTCTRL_EVOSEL_DISABLE |
                         DMAC_BTCTRL_BEATSIZE_HWORD |
                         DMAC_BTCTRL_BLOCKACT_NOACT |
                         DMAC_BTCTRL_DSTINC         |
                         DMAC_BTCTRL_STEPSIZE_X1;

  DMAC_chirp_descriptor_init(chirp_in_ds, num_samples, (uint32_t)&ADC0->RESULT.reg, (uint32_t)&chirp_right_in_buffer[0], 1);
  DMAC_chirp_descriptor_init(chirp_in_ds, num_samples, (uint32_t)&ADC1->RESULT.reg, (uint32_t)&chirp_left_in_buffer[0], 2);



/*
  for(int i=0; i < num_samples; i++){
    if(i % 10 == 0){
      Serial.println();
    }
    Serial.printf("%d, ", chirp_buffer[i]);
  }*/

  //TCC0_DITH_set(4, 12, 10, 3);
  //TCC0_DT_set(0x0, 0x2)

  TCC_enable(TCC0);
  TCC_enable(TCC1);

  ADC_enable(ADC0);
  ADC_enable(ADC1);

  DMAC_CH_enable(ML_DMAC_CHIRP_OUT_CH);
  DMAC_CH_enable(ML_DMAC_CHIRP_RIGHT_IN_CH);
  DMAC_CH_enable(ML_DMAC_CHIRP_LEFT_IN_CH);
  
  //CCL_enable();

  Serial.begin(115200);
  while(!Serial);

}

void loop() {}
