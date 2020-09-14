#include "uf2.h"

/** 
    Wait for the synchronization bits (BITMASK) of the peripheral (PER).
**/
#define SYNCBUSY_WAIT(PER, BITMASK)                                            \
while ((PER)->SYNCBUSY.reg & (BITMASK)) {                                      \
    continue;                                                                  \
}
/**
    Set the peripheral's clock source.
 
    - PER_GLCK_ID is the GCLK ID of a peripheral (e.g. DAC_GCLK_ID).
    - GCLK_INDEX is the numeric index of the GCLK source (i.e. 0-11).
 
    This will work because GCLK_PCHCTRL_GEN_GCLKx_Val == x for x in [0, 11]
    (see gclk.h).
    Therefore GCLK_PCHCTRL_GEN(x) == GCLK_PCHCTRL_GEN(GCLK_PCHCTRL_GEN_GCLKx_Val)
    and so the correct value will be set in the GEN register of the GCLK.
 
    The procedure for setting a peripheral's clock source follows from
    section 14.6.3.3 Selecting the Clock Source for a Peripheral (p. 155)
    of the SAMD5xE5x datasheet:
        1. Disable the Peripheral Channel by writing PCHCTRLm.CHEN=0
        2. Assert that PCHCTRLm.CHEN reads '0'
        3. Change the source of the Peripheral Channel by writing PCHCTRLm.GEN
        4. Re-enable the Peripheral Channel by writing PCHCTRLm.CHEN=1
    
    ...and from section 14.6.3.1 Enabling a Peripheral Clock (p. 155):

    The PCHCTRLm.CHEN bit must be synchronized to the generic clock domain.
    PCHCTRLm.CHEN will continue to read as its previous state until the
    synchronization is complete.

    This necessary synchronization is the reason for the final while-loop.
**/
#define SET_CLOCK_SOURCE(PER_GCLK_ID, GCLK_INDEX)                              \
GCLK->PCHCTRL[(PER_GCLK_ID)].bit.CHEN = 0;                                     \
while (GCLK->PCHCTRL[(PER_GCLK_ID)].bit.CHEN) {                                \
    continue;                                                                  \
}                                                                              \
GCLK->PCHCTRL[(PER_GCLK_ID)].bit.GEN = GCLK_PCHCTRL_GEN((GCLK_INDEX));         \
GCLK->PCHCTRL[(PER_GCLK_ID)].bit.CHEN = 1;                                     \
while (GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_GENCTRL((GCLK_INDEX))) {             \
    continue;                                                                  \
}

// Final CPU speed
#define __CLEARCORE_CLOCK_HZ    (48000000)             // 48 MHz
#define __SYSTEM_CLOCK          (__CLEARCORE_CLOCK_HZ)
// Oscillator output into XOSC1
#define __CLEARCORE_OSC_HZ      (25000000)              // 25 MHz
// GCLK0 FREQ
#define __CLEARCORE_GCLK0_HZ    __CLEARCORE_CLOCK_HZ
// GCLK5 FREQ
#define __CLEARCORE_GCLK1_HZ    (1000000)               // 1 MHz
// DPLL1 FREQ
#define __CLEARCORE_DPLL0_HZ    (96000000)             // 96 MHz

void system_init(void) {
    // Automatic wait states.
    NVMCTRL->CTRLA.bit.AUTOWS = 1;

    /* Software reset the module to ensure it is re-initialized correctly */
    /* Note: Due to synchronization, there is a delay from writing CTRL.SWRST until the reset is complete.
     * CTRL.SWRST and STATUS.SYNCBUSY will both be cleared when the reset is complete
     */
    GCLK->CTRLA.bit.SWRST = 1;
    while (GCLK->SYNCBUSY.bit.SWRST) {
        /* wait for reset to complete */
    }

    // Configure the main clock to run at 48 MHz based off of the external oscillator

    // Start the external 10MHz MEMS oscillator
    OSCCTRL->XOSCCTRL[1].reg =
        OSCCTRL_XOSCCTRL_IMULT(4) |
        OSCCTRL_XOSCCTRL_IPTAT(3) |
        OSCCTRL_XOSCCTRL_ENABLE;
    // Wait for clock to run
    while (!OSCCTRL->STATUS.bit.XOSCRDY1) {
        continue;
    }
    // Create 1MHz clock on GCLK1 to act as source for DPLL0 and SERCOM6
    GCLK->GENCTRL[1].reg = GCLK_GENCTRL_SRC_XOSC1 |
                           GCLK_GENCTRL_GENEN |
                           GCLK_GENCTRL_DIV(__CLEARCORE_OSC_HZ /
                                            __CLEARCORE_GCLK1_HZ) |
                           GCLK_GENCTRL_IDC;
    SYNCBUSY_WAIT(GCLK, GCLK_SYNCBUSY_GENCTRL1);

    // Make good 96MHz CPU clock using DPLL0 multiplying GCLK2 up
    SET_CLOCK_SOURCE(OSCCTRL_GCLK_ID_FDPLL0, 1);
    // Set the integer part of the frequency multiplier (loop divider ratio)
    OSCCTRL->Dpll[0].DPLLRATIO.reg =
    OSCCTRL_DPLLRATIO_LDR(__CLEARCORE_DPLL0_HZ / __CLEARCORE_GCLK1_HZ - 1);  
    // Set GCLK as the DPLL clock reference, and set Wake Up Fast
    OSCCTRL->Dpll[0].DPLLCTRLB.reg = OSCCTRL_DPLLCTRLB_REFCLK_GCLK |
                                     OSCCTRL_DPLLCTRLB_WUF;
        
    // Set the DPLL (digital phase-locked loop) to run in standby and sleep mode
    // If ONDEMAND is not set, the signal will be generated constantly
    // Finally, enable the DPLL
    OSCCTRL->Dpll[0].DPLLCTRLA.reg = OSCCTRL_DPLLCTRLA_RUNSTDBY |
                                     OSCCTRL_DPLLCTRLA_ENABLE;

    while (OSCCTRL->STATUS.bit.DPLL0LCKR) {
        continue;
    }
    // Route DPLL1 to CPU Clock @ 48MHz.
    GCLK->GENCTRL[0].reg = GCLK_GENCTRL_SRC_DPLL0 |
                           GCLK_GENCTRL_GENEN |
                           GCLK_GENCTRL_DIV(__CLEARCORE_DPLL0_HZ / __CLEARCORE_GCLK0_HZ);
    SYNCBUSY_WAIT(GCLK, GCLK_SYNCBUSY_GENCTRL0);
    // Clocks running and locked, switch core clock to 48MHz
    MCLK->CPUDIV.reg = MCLK_CPUDIV_DIV_DIV1;

#if defined(CLEARCORE_ID)
    // Set the clock source for SERCOM6 to GCLK0 and enable the peripheral channel
    GCLK->PCHCTRL[SERCOM6_GCLK_ID_CORE].reg |= GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN;		
    // Enable the clock and sync with core
    MCLK->APBDMASK.reg |= MCLK_APBDMASK_SERCOM6;
    clearCoreInit();
#endif
   /* Turn on the digital interface clock */
    //MCLK->APBAMASK.reg |= MCLK_APBAMASK_GCLK;

    /*
     * Now that all system clocks are configured, we can set CLKDIV .
     * These values are normally the ones present after Reset.
     */
    MCLK->CPUDIV.reg = MCLK_CPUDIV_DIV_DIV1;

    SysTick_Config(1000);
}

// 1kHZ timer tick
void SysTick_Handler(void) { 
    LED_TICK(); 
#if defined(CLEARCORE_ID)
    clearCoreLEDseq();
#endif
}
