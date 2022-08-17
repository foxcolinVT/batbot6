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

#define ML_GCLK_CH 1
#define ML_GCLK_INITIAL_DIV 1
#define ML_GCLK_GENCTRL_DIV1 (GCLK_GENCTRL_DIV(ML_GCLK_INITIAL_DIV))
#define ML_GCLK_GENCTRL_SRC_DPLL (GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_DPLL1_Val))

// Channel enable, GCLK2, WRTLCK - disable future writing to reg
#define ML_GCLK2_PCHCTRL (GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_WRTLOCK)
#define GCLK0_FREQ (ML_MCLK_UNDIV/(1 * ML_HCLK_INITIAL_DIV))             // 60 Mhz

void GCLK0_init(void);

/*
        TCC definitions
*/

inline void TCC_enable(Tcc *tcc);
inline void TCC_disable(Tcc *tcc);
inline void TCC_swrst(Tcc *tcc);

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

static void TCC0_PORT_init(void);
void TCC0_init(void);
inline void TCC0_DT_set(uint8_t dth, uint8_t dtl);
void TCC0_DITH_set(char mode, uint64_t cycles, uint64_t period, uint64_t compare);

#define ML_TCC2_CH1 1

#define ML_TCC2_CH1_PIN 23
#define ML_M4_TCC2_CH1_PIN 15
#define ML_M4_TCC2_CH1_PMUX 0x7
#define ML_TCC2_CH1_PMUX_msk (PORT_PMUX_PMUXE(ML_M4_TCC2_CH1_PMUX))

static void TCC2_PORT_init(void);
void TCC2_init(void);

/*
        DMAC definitions
*/

#define ML_DMAC_TCC0_OVF_TRIG 0x16
#define ML_DMAC_TCC1_OVF_TRIG 0x1D

inline void DMAC_enable(void);
inline void DMAC_swrst(void);
inline void DMAC_CH_enable(char ch);

#define ML_DMAC_CHIRP_CH 0

void DMAC_init(void);
