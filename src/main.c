/* ----------------------------------------------------------------------------
 *         SAM Software Package License
 * ----------------------------------------------------------------------------
 * Copyright (c) 2011-2014, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following condition is met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

/**
 * --------------------
 * SAM-BA Implementation on SAMD21
 * --------------------
 * Requirements to use SAM-BA :
 *
 * Supported communication interfaces :
 * --------------------
 *
 * SERCOM5 : RX:PB23 TX:PB22
 * Baudrate : 115200 8N1
 *
 * USB : D-:PA24 D+:PA25
 *
 * Pins Usage
 * --------------------
 * The following pins are used by the program :
 * PA25 : input/output
 * PA24 : input/output
 * PB23 : input
 * PB22 : output
 * PA15 : input
 *
 * The application board shall avoid driving the PA25,PA24,PB23,PB22 and PA15
 * signals
 * while the boot program is running (after a POR for example)
 *
 * Clock system
 * --------------------
 * CPU clock source (GCLK_GEN_0) - 8MHz internal oscillator (OSC8M)
 * SERCOM5 core GCLK source (GCLK_ID_SERCOM5_CORE) - GCLK_GEN_0 (i.e., OSC8M)
 * GCLK Generator 1 source (GCLK_GEN_1) - 48MHz DFLL in Clock Recovery mode
 * (DFLL48M)
 * USB GCLK source (GCLK_ID_USB) - GCLK_GEN_1 (i.e., DFLL in CRM mode)
 *
 * Memory Mapping
 * --------------------
 * SAM-BA code will be located at 0x0 and executed before any applicative code.
 *
 * Applications compiled to be executed along with the bootloader will start at
 * 0x2000
 * Before jumping to the application, the bootloader changes the VTOR register
 * to use the interrupt vectors of the application @0x2000.<- not required as
 * application code is taking care of this
 *
 */

#include "uf2.h"

static void check_start_application(void);

static void check_and_restore_fuses(void);

bool main_b_cdc_enable = false;
extern int8_t led_tick_step;

#ifdef SAMD21
#define RESET_CONTROLLER PM
#endif
#if defined(SAMD51) || defined(SAME53)
#define RESET_CONTROLLER RSTC
#endif

/**
 * \brief Start the user application if it's safe to do so. 
 * Note: this function won't return if all goes well. If something is 
 * wrong, such as the user program is invalid or missing, this function
 * returns allowing the bootloader code to run.
 */
static void check_start_application(void) {
    uint32_t app_start_address;

    /* Load the Reset Handler address of the application */
    app_start_address = *(uint32_t *)(APP_START_ADDRESS + 4);
    
    /**
     * Test reset vector of application @APP_START_ADDRESS+4
     * Sanity check on the Reset_Handler address
     */
    if (app_start_address < APP_START_ADDRESS || app_start_address > FLASH_SIZE) {
        /* Stay in bootloader */
        return;
    }

#if USE_SINGLE_RESET
    if (SINGLE_RESET()) {
        if (RESET_CONTROLLER->RCAUSE.bit.POR || *DBL_TAP_PTR != DBL_TAP_MAGIC_QUICK_BOOT) {
            // the second tap on reset will go into app
            *DBL_TAP_PTR = DBL_TAP_MAGIC_QUICK_BOOT;
            // this will be cleared after succesful USB enumeration
            // this is around 1.5s
            resetHorizon = timerHigh + 300;
            return;
        }
    }
#endif
    if (RESET_CONTROLLER->RCAUSE.bit.POR) {
        *DBL_TAP_PTR = 0;
    }
    else if (*DBL_TAP_PTR == DBL_TAP_MAGIC) {
        *DBL_TAP_PTR = 0;
        return; // stay in bootloader
    }
    else {
        if (*DBL_TAP_PTR != DBL_TAP_MAGIC_QUICK_BOOT) {
            // Turn the LED on to notify the user to send the
            // next reset click to stay in the bootloader
            LED_MSC_ON();
            *DBL_TAP_PTR = DBL_TAP_MAGIC;
            delay(500);
        }
        *DBL_TAP_PTR = 0;
    }

    LED_MSC_OFF();
#if defined(__SAMD21E18A__)
    RGBLED_set_color(COLOR_LEAVE);
#endif

    /* Rebase the Stack Pointer */
    __set_MSP(*(uint32_t *)APP_START_ADDRESS);

    /* Rebase the vector table base address */
    SCB->VTOR = ((uint32_t)APP_START_ADDRESS & SCB_VTOR_TBLOFF_Msk);

    /* Jump to application Reset Handler in the application */
    asm("bx %0" ::"r"(app_start_address));
}

extern char _etext;
extern char _end;

/**
 *  \brief SAMD21 SAM-BA Main loop.
 *  \return Unused (ANSI-C compatibility).
 */
int main(void) {
    // if VTOR is set, we're not running in bootloader mode; halt
    if (SCB->VTOR)
        while (1) {
        }

// Disable the watchdog, in case the application set it.
#if defined(SAMD21)
    WDT->CTRL.reg = 0;
    while(WDT->STATUS.bit.SYNCBUSY) {}
#endif
#if defined(SAMD51) || defined(SAMD53)
    WDT->CTRLA.reg = 0;
    while(WDT->SYNCBUSY.reg) {}
#endif

#if (USB_VID == 0x239a) && (USB_PID == 0x0013) // Adafruit Metro M0
    // Delay a bit so SWD programmer can have time to attach.
    delay(15);
#endif

    // Enable 2.7V brownout detection. The default fuse value is 1.7
    // Set brownout detection to ~2.7V. Default from factory is 1.7V,
    // which is too low for proper operation of external SPI flash chips (they are 2.7-3.6V).
    // Also without this higher level, the SAMD51 will write zeros to flash intermittently.
    // Disable while changing level.
    SUPC->BOD33.bit.ENABLE = 0;
    SUPC->BOD33.bit.LEVEL = 200;  // 2.7V: 1.5V + LEVEL * 6mV.
    // Don't reset right now.
    SUPC->BOD33.bit.ACTION = SUPC_BOD33_ACTION_NONE_Val;
    SUPC->BOD33.bit.ENABLE = 1; // enable brown-out detection

    // Wait for BOD33 peripheral to be ready.
    while (!SUPC->STATUS.bit.BOD33RDY) {}

    // Wait for voltage to rise above BOD33 value.
    while (SUPC->STATUS.bit.BOD33DET) {}

    
    // If we are starting from a power-on or a brownout,
    // wait for the voltage to stabilize. Don't do this on an
    // external reset because it interferes with the timing of double-click.
    // "BODVDD" means BOD33.
    if (RSTC->RCAUSE.bit.POR || RSTC->RCAUSE.bit.BODVDD) {
        do {
            // Check again in 100ms.
            delay(100);
        } while (SUPC->STATUS.bit.BOD33DET);
    }

    // Now enable reset if voltage falls below minimum.
    SUPC->BOD33.bit.ENABLE = 0;
    // Wait for BOD33 to synchronize disable cmd
    while (!SUPC->STATUS.bit.B33SRDY) {}
    SUPC->BOD33.bit.ACTION = SUPC_BOD33_ACTION_RESET_Val;
    SUPC->BOD33.bit.ENABLE = 1;

    led_init();

    check_and_restore_fuses();

    logmsg("Start");
    assert((uint32_t)&_etext < APP_START_ADDRESS);
    // bossac writes at 0x20005000
    assert(!USE_MONITOR || (uint32_t)&_end < 0x20005000);

    assert(8 << NVMCTRL->PARAM.bit.PSZ == FLASH_PAGE_SIZE);
    assert(FLASH_PAGE_SIZE * NVMCTRL->PARAM.bit.NVMP == FLASH_SIZE);

#if defined(CLEARCORE_ID)
    clearCoreInit();
#endif

    /* Jump in application if condition is satisfied */
    check_start_application();

    /* We have determined we should stay in the monitor. */
    /* System initialization */
    system_init();
#if defined(CLEARCORE_ID)

    // Setup initial lightshow
    clearCoreFlashLEDinit();
#endif

    __DMB();
    __enable_irq();

#if USE_UART
    /* UART is enabled in all cases */
    usart_open();
#endif

    logmsg("Before main loop");

    usb_init();

    // not enumerated yet
    RGBLED_set_color(COLOR_START);
    led_tick_step = 10;

    /* Wait for a complete enum on usb or a '#' char on serial line */
    while (1) {
        if (USB_Ok()) {
            if (!main_b_cdc_enable) {
#if USE_SINGLE_RESET
                // this might have been set
                resetHorizon = 0;
#endif
                RGBLED_set_color(COLOR_USB);
                led_tick_step = 1;
                #if defined(CLEARCORE_ID)
                    ledState = true;
                #endif

#if USE_SCREEN
                screen_init();
                draw_drag();
#endif
            }

            main_b_cdc_enable = true;
        }

#if USE_MONITOR
        // Check if a USB enumeration has succeeded
        // And com port was opened
        if (main_b_cdc_enable) {
            logmsg("entering monitor loop");
            // SAM-BA on USB loop
            while (1) {
                sam_ba_monitor_run();
            }
        }
#if USE_UART
        /* Check if a '#' has been received */
        if (!main_b_cdc_enable && usart_sharp_received()) {
            RGBLED_set_color(COLOR_UART);
            sam_ba_monitor_init(SAM_BA_INTERFACE_USART);
            /* SAM-BA on UART loop */
            while (1) {
                sam_ba_monitor_run();
            }
        }
#endif
#else // no monitor
        if (main_b_cdc_enable) {
            process_msc();
        }
#endif
        if (!main_b_cdc_enable) {
            // get more predictable timings before the USB is enumerated
            for (int i = 1; i < 256; ++i) {
                asm("nop");
            }
        }
    }
}

void check_and_restore_fuses(void) {
    // If fuses have been reset to all ones, the bootloader protection is
    // cleared, so the next attempt to program the board will erase the 
    // bootloader and application. To prevent this, fix the fuses.
    // If the fuses are corrupt, it is likely that the rest of user 
    // NVM is also corrupt, so it is ok to erase it. User app should detect
    // corrupt user space and handle it.
    if (((uint32_t *)NVMCTRL_USER)[0] == 0xffffffff) {
        // Turn off cache and put in manual mode.
        NVMCTRL->CTRLA.bit.CACHEDIS0 = 1;
        NVMCTRL->CTRLA.bit.CACHEDIS1 = 1;
        NVMCTRL->CTRLA.bit.WMODE = NVMCTRL_CTRLA_WMODE_MAN;
        // Set address to write.
        NVMCTRL->ADDR.reg = NVMCTRL_USER;
        // Erase user space
        // This will destroy calibration and MAC address info. 
        // But the data is likely already lost at this point
        NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMDEX_KEY | NVMCTRL_CTRLB_CMD_EP;
	while (!(NVMCTRL->STATUS.bit.READY)) {}
        // Clear page buffer.
        NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMDEX_KEY | NVMCTRL_CTRLB_CMD_PBC;
	while (!(NVMCTRL->STATUS.bit.READY)) {}
        // Reasonable fuse values
        ((uint32_t *)NVMCTRL_USER)[0] = 0xF69A9239;
        ((uint32_t *)NVMCTRL_USER)[1] = 0xAEECFF80;
        // Write the fuses
	NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMDEX_KEY | NVMCTRL_CTRLB_CMD_WQW;
	while (!(NVMCTRL->STATUS.bit.READY)) {}
        resetIntoBootloader();
    }
}
