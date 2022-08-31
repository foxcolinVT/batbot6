#include <Arduino.h>

volatile boolean results0Part0Ready = false;
volatile boolean results0Part1Ready = false;
volatile boolean results1Part0Ready = false;
volatile boolean results1Part1Ready = false;



volatile DmacDescriptor wb_descriptor[12] __attribute__((aligned(16)));
DmacDescriptor base_descriptor[12] __attribute__((aligned(16)));
DmacDescriptor descriptor __attribute__((aligned(16)));

DmacDescriptor linked_descriptor[2] __attribute__((aligned(16)));

DmacDescriptor left_linked_descriptor_alt[1000] __attribute__((aligned(16)));
DmacDescriptor right_linked_descriptor_alt[1000] __attribute__((aligned(16)));

uint16_t adcResults0[5000];
uint16_t adcResults1[5000];

uint16_t adcBufR[8][5000];
uint16_t adcBufL[8][5000];

void lde_setup(void){

  descriptor.DESCADDR.reg = (uint32_t)&linked_descriptor[0];
  descriptor.SRCADDR.reg = (uint32_t)&ADC1->RESULT.reg;
  descriptor.DSTADDR.reg = (uint32_t)&adcResults1 + sizeof(uint16_t) * 2500;
  descriptor.BTCNT.reg = 2500;
  descriptor.BTCTRL.reg = DMAC_BTCTRL_BEATSIZE_HWORD |
                          DMAC_BTCTRL_DSTINC |
                          DMAC_BTCTRL_VALID  |
                          DMAC_BTCTRL_BLOCKACT_SUSPEND;
  memcpy(&base_descriptor[1], &descriptor, sizeof(descriptor));

  descriptor.DESCADDR.reg = (uint32_t)&base_descriptor[1];
  descriptor.SRCADDR.reg = (uint32_t)&ADC1->RESULT.reg;
  descriptor.DSTADDR.reg = (uint32_t)&adcResults1[2500] + sizeof(uint16_t) * 2500;
  descriptor.BTCNT.reg = 2500;
  descriptor.BTCTRL.reg = DMAC_BTCTRL_BEATSIZE_HWORD |
                          DMAC_BTCTRL_DSTINC |
                          DMAC_BTCTRL_VALID  |
                          DMAC_BTCTRL_BLOCKACT_SUSPEND;
  memcpy(&linked_descriptor[1], &descriptor, sizeof(descriptor));

  descriptor.DESCADDR.reg = (uint32_t)&linked_descriptor[0];
  descriptor.SRCADDR.reg = (uint32_t)&ADC0->RESULT.reg;
  descriptor.DSTADDR.reg = (uint32_t)&adcResults0 + sizeof(uint16_t) * 2500;
  descriptor.BTCNT.reg = 2500;
  descriptor.BTCTRL.reg = DMAC_BTCTRL_BEATSIZE_HWORD |
                          DMAC_BTCTRL_DSTINC |
                          DMAC_BTCTRL_VALID  |
                          DMAC_BTCTRL_BLOCKACT_SUSPEND;
  memcpy(&base_descriptor[0], &descriptor, sizeof(descriptor));

  descriptor.DESCADDR.reg = (uint32_t)&base_descriptor[0];
  descriptor.SRCADDR.reg = (uint32_t)&ADC0->RESULT.reg;
  descriptor.DSTADDR.reg = (uint32_t)&adcResults0[2500] + sizeof(uint16_t) * 2500;
  descriptor.BTCNT.reg = 2500;
  descriptor.BTCTRL.reg = DMAC_BTCTRL_BEATSIZE_HWORD |
                          DMAC_BTCTRL_DSTINC |
                          DMAC_BTCTRL_VALID  |
                          DMAC_BTCTRL_BLOCKACT_SUSPEND;
  memcpy(&linked_descriptor[0], &descriptor, sizeof(descriptor));

}

void lda_setup(void){

  uint16_t chirp_in_ds = DMAC_BTCTRL_BEATSIZE_HWORD |
                         DMAC_BTCTRL_DSTINC         |
                         DMAC_BTCTRL_VALID          |
                         DMAC_BTCTRL_BLOCKACT_SUSPEND;

  uint16_t chirp_btcnt = 5000;
  uint32_t chirp_in_addr_inc = chirp_btcnt * sizeof(uint16_t);

  descriptor.DESCADDR.reg = (uint32_t)&left_linked_descriptor_alt[0];
  descriptor.SRCADDR.reg = (uint32_t)&ADC0->RESULT.reg;
  descriptor.DSTADDR.reg = (uint32_t)&adcBufL[0] + chirp_in_addr_inc;
  descriptor.BTCNT.reg = chirp_btcnt;
  descriptor.BTCTRL.reg = chirp_in_ds;

  memcpy(&base_descriptor[0], &descriptor, sizeof(descriptor));

  descriptor.DESCADDR.reg = (uint32_t)&right_linked_descriptor_alt[0];
  descriptor.SRCADDR.reg = (uint32_t)&ADC1->RESULT.reg;
  descriptor.DSTADDR.reg = (uint32_t)&adcBufR[0] + chirp_in_addr_inc;
  descriptor.BTCNT.reg = chirp_btcnt;
  descriptor.BTCTRL.reg = chirp_in_ds;

  memcpy(&base_descriptor[1], &descriptor, sizeof(descriptor));

  for(unsigned i=1; i < 30; i++){

    descriptor.DESCADDR.reg = (uint32_t)&left_linked_descriptor_alt[i];
    descriptor.SRCADDR.reg = (uint32_t)&ADC0->RESULT.reg;
    descriptor.DSTADDR.reg = (uint32_t)&adcBufL[i] + chirp_in_addr_inc;
    descriptor.BTCNT.reg = chirp_btcnt;
    descriptor.BTCTRL.reg = chirp_in_ds;

    memcpy(&left_linked_descriptor_alt[i - 1], &descriptor, sizeof(descriptor));

    descriptor.DESCADDR.reg = (uint32_t)&right_linked_descriptor_alt[i];
    descriptor.SRCADDR.reg = (uint32_t)&ADC1->RESULT.reg;
    descriptor.DSTADDR.reg = (uint32_t)&adcBufR[i] + chirp_in_addr_inc;
    descriptor.BTCNT.reg = chirp_btcnt;
    descriptor.BTCTRL.reg = chirp_in_ds;

    memcpy(&right_linked_descriptor_alt[i - 1], &descriptor, sizeof(descriptor));
  }

  descriptor.DESCADDR.reg = (uint32_t)&base_descriptor[0];
  descriptor.SRCADDR.reg = (uint32_t)&ADC0->RESULT.reg;
  descriptor.DSTADDR.reg = (uint32_t)&adcBufL[7] + chirp_in_addr_inc;
  descriptor.BTCNT.reg = chirp_btcnt;
  descriptor.BTCTRL.reg = chirp_in_ds;

  memcpy(&left_linked_descriptor_alt[7], &descriptor, sizeof(descriptor));

  descriptor.DESCADDR.reg = (uint32_t)&base_descriptor[1];
  descriptor.SRCADDR.reg = (uint32_t)&ADC1->RESULT.reg;
  descriptor.DSTADDR.reg = (uint32_t)&adcBufR[7] + chirp_in_addr_inc;
  descriptor.BTCNT.reg = chirp_btcnt;
  descriptor.BTCTRL.reg = chirp_in_ds;

  memcpy(&right_linked_descriptor_alt[7], &descriptor, sizeof(descriptor));
}

const uint32_t num_pages = 8;
const uint32_t num_samples = 5000;

void ldl_setup(uint16_t alt_buffer[num_pages][num_samples], 
              const uint32_t npages, 
              const uint16_t nsamples, 
              const uint32_t srcaddr,
              const uint16_t btsettings,
              DmacDescriptor ld[num_pages], 
              DmacDescriptor *basecpy){

  const uint32_t buff_addr_inc = nsamples * sizeof(uint16_t);

  DmacDescriptor run;

  run.SRCADDR.reg = srcaddr;
  run.BTCNT.reg = nsamples;
  run.BTCTRL.reg = btsettings;

  run.DESCADDR.reg = (uint32_t)&ld[0];
  run.DSTADDR.reg = (uint32_t)&alt_buffer[0] + buff_addr_inc;

  memcpy((void *)basecpy, &run, sizeof(DmacDescriptor));

  for(uint32_t i=1; i < npages; i++){

    run.DESCADDR.reg = (uint32_t)&ld[i];
    run.DSTADDR.reg = (uint32_t)&alt_buffer[i] + buff_addr_inc;

    memcpy((void *)&ld[i - 1], &run, sizeof(DmacDescriptor));
  }

  run.DESCADDR.reg = (uint32_t) basecpy;
  run.DSTADDR.reg = (uint32_t)&alt_buffer[npages - 1] + buff_addr_inc;

  memcpy((void *)&ld[npages - 1], &run, sizeof(DmacDescriptor));
}

void ADC_test(void) {

  MCLK->CPUDIV.reg = MCLK_CPUDIV_DIV_DIV1;


  GCLK->GENCTRL[7].reg = GCLK_GENCTRL_DIV(1) |                // GCLK divider, GCLK7_FREQ = ML_MCLK_UNDIV/(ML_MCLK_CPUDIV * ML_GCLK_GENCTRL_DIV)
                                  GCLK_GENCTRL_IDC |                    // 50/50 duty 
                                  GCLK_GENCTRL_SRC_DPLL0 |            // Source multiplexer selects DPLL0 (phase locked loop)
                                  GCLK_GENCTRL_GENEN; // Output enable, Generator enable

  // wait for GEN7 sync
  while(GCLK->SYNCBUSY.bit.GENCTRL7);


  GCLK->PCHCTRL[ADC0_GCLK_ID].reg = GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK7 | GCLK_PCHCTRL_WRTLOCK;                  // TCC0 and TCC1 ID are the same, so TCC1 is implicitly set
  
  GCLK->PCHCTRL[ADC1_GCLK_ID].reg = GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK7 | GCLK_PCHCTRL_WRTLOCK;


  DMAC->BASEADDR.reg = (uint32_t)&base_descriptor;
  DMAC->WRBADDR.reg = (uint32_t)&wb_descriptor;
  DMAC->CTRL.reg = DMAC_CTRL_DMAENABLE | DMAC_CTRL_LVLEN(0xf);
  
  uint16_t chirp_in_ds = DMAC_BTCTRL_BEATSIZE_HWORD |
                         DMAC_BTCTRL_DSTINC         |
                         DMAC_BTCTRL_VALID          |
                         DMAC_BTCTRL_BLOCKACT_SUSPEND;

  ldl_setup(
    adcBufR,
    num_pages,
    num_samples,
    (uint32_t)&ADC1->RESULT.reg,
    chirp_in_ds,
    right_linked_descriptor_alt,
    &base_descriptor[1]
  );

  ldl_setup(
    adcBufL,
    num_pages,
    num_samples,
    (uint32_t)&ADC0->RESULT.reg,
    chirp_in_ds,
    left_linked_descriptor_alt,
    &base_descriptor[0]
  );

  DMAC->Channel[0].CHCTRLA.reg = DMAC_CHCTRLA_TRIGSRC(ADC0_DMAC_ID_RESRDY) |
                                 DMAC_CHCTRLA_TRIGACT_BURST;



  DMAC->Channel[0].CHINTENSET.reg = DMAC_CHINTENSET_SUSP;

  NVIC_SetPriority(DMAC_0_IRQn, 0);
  NVIC_EnableIRQ(DMAC_0_IRQn);

  DMAC->Channel[1].CHCTRLA.reg = DMAC_CHCTRLA_TRIGSRC(ADC1_DMAC_ID_RESRDY) |
                                 DMAC_CHCTRLA_TRIGACT_BURST;



  DMAC->Channel[1].CHINTENSET.reg = DMAC_CHINTENSET_SUSP;

  NVIC_SetPriority(DMAC_1_IRQn, 0);
  NVIC_EnableIRQ(DMAC_1_IRQn);

  ADC1->INPUTCTRL.bit.MUXPOS = ADC_INPUTCTRL_MUXPOS_AIN6_Val;
  while(ADC1->SYNCBUSY.bit.INPUTCTRL);

  ADC1->SAMPCTRL.bit.SAMPLEN = 0x02;
  while(ADC1->SYNCBUSY.bit.SAMPCTRL);

  ADC1->CTRLB.reg = ADC_CTRLB_RESSEL_12BIT | ADC_CTRLB_FREERUN;
  while(ADC1->SYNCBUSY.bit.CTRLB);

  ADC1->CTRLA.bit.SLAVEEN = 0x1;


  ADC0->INPUTCTRL.bit.MUXPOS = ADC_INPUTCTRL_MUXPOS_AIN5_Val;
  while(ADC0->SYNCBUSY.bit.INPUTCTRL);

  ADC0->SAMPCTRL.bit.SAMPLEN = 0x02;
  while(ADC0->SYNCBUSY.bit.SAMPCTRL);

  ADC0->CTRLB.reg = ADC_CTRLB_RESSEL_12BIT | ADC_CTRLB_FREERUN;
  while(ADC0->SYNCBUSY.bit.CTRLB);

  ADC0->CTRLA.reg = ADC_CTRLA_PRESCALER_DIV8;
  ADC0->CTRLA.bit.ENABLE = 1;
  while(ADC0->SYNCBUSY.bit.ENABLE);

  ADC0->SWTRIG.bit.START = 1;
  while(ADC0->SYNCBUSY.bit.SWTRIG);

  DMAC->Channel[0].CHCTRLA.bit.ENABLE = 1;
  DMAC->Channel[1].CHCTRLA.bit.ENABLE = 1;

  Serial.begin(115200);
  while(!Serial);

  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
}


void DMAC_0_Handlerp(void){

  static unsigned count = 0;

  if(DMAC->Channel[0].CHINTFLAG.bit.SUSP){

    DMAC->Channel[0].CHCTRLB.reg = DMAC_CHCTRLB_CMD_RESUME;
    DMAC->Channel[0].CHINTFLAG.bit.SUSP = 1;

    if(count >= 8){
      //rdy0 = true;
      count = 0;
    } else count++;

  }
}

void DMAC_1_Handlerp(void){

  static uint8_t count1 = 0;
  
  if(DMAC->Channel[1].CHINTFLAG.bit.SUSP){

    DMAC->Channel[1].CHCTRLB.reg = DMAC_CHCTRLB_CMD_RESUME;
    DMAC->Channel[1].CHINTFLAG.bit.SUSP = 1;

    if(count1){
      results1Part1Ready = true;
    } else results1Part0Ready = true;

    count1 = (count1 + 1) % 2;

    digitalWrite(11, HIGH);
    digitalWrite(11, LOW);

  }
}