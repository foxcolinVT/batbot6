#include <Arduino.h>

//#define ML_M4EXPRESS

// Note: macros/data structures used w/o prefixes ML (Mueller Lab) 
// exist at ${PATH_TO_ARDUINO_CONFIGS}/packages/adafruit/tools/CMSIS-Atmel/1.2.2/CMSIS/Device/ATMEL/samd51

/*
        MCLK definitions
*/

#define ML_MCLK_UNDIV 120000000
#define ML_MCLK_CPUDIV1 (MCLK_CPUDIV_DIV(MCLK_CPUDIV_DIV_DIV1_Val))

void MCLK_init(void);

/*
        GCLK definitions
*/

#define ML_GCLK_CH 7

// Channel enable, GCLK7, WRTLCK - disable future writing to reg
#define ML_GCLK7_PCHCTRL (GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK7 | GCLK_PCHCTRL_WRTLOCK)
#define ML_GCLK1_PCHCTRL (GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK1)

void GCLK_init(void);

/*
        TCC definitions
*/

inline void TCC_enable(Tcc *tcc);
inline void TCC_disable(Tcc *tcc);
inline void TCC_swrst(Tcc *tcc);

static inline void TCC_sync(Tcc *tcc);

static void TCC_CH_PORT_init(const EPortType port_grp, const uint32_t pin, const uint8_t pmux_msk, const uint8_t m4_pin);

static inline void TCC_CH_CC_set(volatile TCC_CC_Type *CCx, uint8_t count_val);

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

// For D45, TCC0.0: PC12 -> peripheral function F and PMUXE
#define ML_TCC0_CH0_PIN 45
#define ML_M4_TCC0_CH0_PIN 10
#define ML_TCC0_CH0_PMUX 0x5                       
#define ML_TCC0_CH0_PMUX_msk (PORT_PMUX_PMUXE(ML_TCC0_CH0_PMUX))

// For D44, TCC0.1: PC11 -> peripheral function F and PMUXO
#define ML_TCC0_CH1_PIN 44
#define ML_M4_TCC0_CH1_PIN 11
#define ML_TCC0_CH1_PMUX 0x5
#define ML_TCC0_CH1_PMUX_msk (PORT_PMUX_PMUXO(ML_TCC0_CH1_PMUX))

#endif


static void TCC0_PORT_init(void);
void TCC0_init(void);
inline void TCC0_DT_set(uint8_t dth, uint8_t dtl);
void TCC0_DITH_set(uint8_t mode, uint64_t cycles, uint64_t period, uint64_t compare);

#define ML_TCC1_CH3 3

// For D23, TCC1.3: PA15 -> peripheral function G and PMUXO
#define ML_TCC1_CH3_PIN 23
#define ML_M4_TCC1_CH3_PIN 15
#define ML_M4_TCC1_CH3_PMUX 0x6
#define ML_TCC1_CH3_PMUX_msk (PORT_PMUX_PMUXO(ML_M4_TCC1_CH3_PMUX))

#define ML_TCC1_CH3_INITIAL_PER (SINE_ALT_MAX_AMPLITUDE - 1)

#define ML_TCC1_CH0_PIN 8
#define ML_M4_TCC1_CH0_PIN 18 //PB18, TCC1.0 : function f, PMUXE
#define ML_M4_TCC1_CH0_PMUX 0x5
#define ML_TCC1_CH0_PMUX_msk (PORT_PMUX_PMUXE(ML_TCC1_CH0_PMUX))

static void TCC1_PORT_init(void);
void TCC1_init(void);

/*
        DMAC definitions
*/

#define ML_DMAC_TCC0_OVF_TRIG 0x16
#define ML_DMAC_TCC1_OVF_TRIG 0x1D

inline void DMAC_enable(void);
inline void DMAC_swrst(void);

inline void DMAC_CH_enable(char ch);
inline void DMAC_CH_disable(char ch);
inline void DMAC_CH_swrst(char ch);

#define ML_DMAC_CHIRP_CH 0

void DMAC_init(void);

void DMAC_chirp_descriptor_init(const uint16_t btsettings, const uint16_t btcnt, const uint32_t srcaddr, const uint32_t dstaddr, const uint8_t bdindex);

uint32_t generate_chirp(void);

/*
        CCL definitions
*/

#define ML_CCL_LUT_BUF_CH 1
#define ML_CCL_LUT_AND_CH 0

// CCL_WO[0] - PB23: D11, peripheral function N

#define ML_CCL_CH0_PIN 11
#define ML_M4_CCL_CH0_PIN 23
#define ML_M4_CCL_CH0_PMUX 0xD
#define ML_CCL_CH0_PMUX_msk (PORT_PMUX_PMUXO(ML_M4_CCL_CH0_PMUX))

inline void CCL_enable(void);
inline void CCL_disable(void);
inline void CCL_swrst(void);
inline void CCL_CH_enable(uint8_t ch);
inline void CCL_CH_disable(uint8_t ch);

static void CCL_PORT_init(void);
void CCL_init(void);

// PA05, A1: peripheral function B, PMUXO
#define ML_ADC0_AIN5_PIN A1
#define ML_M4_ADC0_AIN5_PIN 5
#define ML_M4_ADC0_AIN5_PMUX 0x1
#define ML_ADC0_AIN5_PMUX_msk (PORT_PMUX_PMUXO(ML_M4_ADC0_AIN5_PMUX))
#define ML_ADC0_AIN 5

// PB04, A7: peripheral function B, PMUXE
#define ML_ADC1_AIN6_PIN A7
#define ML_M4_ADC1_AIN6_PIN 4
#define ML_M4_ADC1_AIN6_PMUX 0x1
#define ML_ADC1_AIN6_PMUX_msk (PORT_PMUX_PMUXE(ML_M4_ADC1_AIN6_PMUX))
#define ML_ADC1_AIN 6

inline void ADC_enable(Adc *ADCx);
inline void ADC_disable(Adc *ADCx);
inline void ADC_swrst(Adc *ADCX);

static inline void ADC_sync(Adc *ADCx);

static void ADC_AIN_PORT_init(const EPortType port_grp, const uint32_t pin, const uint8_t pmux_msk);

void ADC_init(Adc *ADCx, const uint8_t muxneg_ain, const uint8_t muxpos_ain);

