#include "uf2.h"

static uint32_t timerLow;
// Set if USB port is active
extern bool main_b_cdc_enable;

uint32_t timerHigh, resetHorizon;

#if defined(CLEARCORE_ID)
// - - - - - - - - - - - - - - - - - - - - -
// Teknic ClearCore Hardware display driver
// - - - - - - - - - - - - - - - - - - - - -
// Board LED states, one bit per LED
uint32_t clearCoreLEDs;
uint32_t clearCoreTicks;
bool ledState = true;
bool clearCoreReady = false;
#endif

void delay(uint32_t ms) {
// SAMD21 starts up at 1mhz by default.
#ifdef SAMD21
    ms <<= 8;
#endif
// SAMD51 starts up at 48mhz by default.
#if defined(SAMD51) || defined(SAME53)
    ms <<= 12;
#endif
    for (int i = 1; i < ms; ++i) {
        asm("nop");
    }
}

void timerTick(void) {
    if (timerLow-- == 0) {
        timerLow = TIMER_STEP;
        timerHigh++;
        if (resetHorizon && timerHigh >= resetHorizon) {
            resetHorizon = 0;
            resetIntoApp();
        }
    }
}

void panic(int code) {
    logval("PANIC", code);
    while (1) {
    }
}

int writeNum(char *buf, uint32_t n, bool full) {
    int i = 0;
    int sh = 28;
    while (sh >= 0) {
        int d = (n >> sh) & 0xf;
        if (full || d || sh == 0 || i) {
            buf[i++] = d > 9 ? 'A' + d - 10 : '0' + d;
        }
        sh -= 4;
    }
    return i;
}

void resetIntoApp() {
    // reset without waiting for double tap (only works for one reset)
    RGBLED_set_color(COLOR_LEAVE);
    *DBL_TAP_PTR = DBL_TAP_MAGIC_QUICK_BOOT;
    NVIC_SystemReset();
}

void resetIntoBootloader() {
    // reset without waiting for double tap (only works for one reset)
    *DBL_TAP_PTR = DBL_TAP_MAGIC;
    NVIC_SystemReset();
}

#if USE_LOGS
struct LogStore logStoreUF2;

void logreset() {
    logStoreUF2.ptr = 0;
    logmsg("Reset logs.");
}

void logwritenum(uint32_t n) {
    char buff[9];
    buff[writeNum(buff, n, false)] = 0;
    logwrite("0x");
    logwrite(buff);
}

void logwrite(const char *msg) {
    const int jump = sizeof(logStoreUF2.buffer) / 4;
    if (logStoreUF2.ptr >= sizeof(logStoreUF2.buffer) - jump) {
        logStoreUF2.ptr -= jump;
        memmove(logStoreUF2.buffer, logStoreUF2.buffer + jump, logStoreUF2.ptr);
    }
    int l = strlen(msg);
    if (l + logStoreUF2.ptr >= sizeof(logStoreUF2.buffer)) {
        logwrite("TOO LONG!\n");
        return;
    }
    memcpy(logStoreUF2.buffer + logStoreUF2.ptr, msg, l);
    logStoreUF2.ptr += l;
    logStoreUF2.buffer[logStoreUF2.ptr] = 0;
}

void logmsg(const char *msg) {
    logwrite(msg);
    logwrite("\n");
}

void logval(const char *lbl, uint32_t v) {
    logwrite(lbl);
    logwrite(": ");
    logwritenum(v);
    logwrite("\n");
}
#endif

static uint32_t now;
static uint32_t signal_end;
int8_t led_tick_step = 1;
static uint8_t limit = 200;

void led_tick() {
    now++;
    if (signal_end) {
        if (now == signal_end - 1000) {
            LED_MSC_ON();
        }
        if (now == signal_end) {
            signal_end = 0;
        }
    } else {
        uint8_t curr = now & 0xff;
        if (curr == 0) {
            LED_MSC_ON();
            if (limit < 10 || limit > 250) {
                led_tick_step = -led_tick_step;
            }
            limit += led_tick_step;
        } else if (curr == limit) {
            LED_MSC_OFF();
        }
    }
}

void led_signal() {
    if (signal_end < now) {
        signal_end = now + 2000;
        LED_MSC_OFF();
    }
}

void led_init() {
#if defined(LED_PIN)
    PINOP(LED_PIN, DIRSET);
#endif
    LED_MSC_ON();

#if defined(BOARD_RGBLED_CLOCK_PIN)
    // using APA102, set pins to outputs
    PINOP(BOARD_RGBLED_CLOCK_PIN, DIRSET);
    PINOP(BOARD_RGBLED_DATA_PIN, DIRSET);

    // This won't work for neopixel, because we're running at 1MHz or thereabouts...
    RGBLED_set_color(COLOR_LEAVE);
#endif

#if USE_SCREEN
    // turn display backlight
    screen_early_init();
#endif
#
}

#if defined(BOARD_RGBLED_CLOCK_PIN)
void write_apa_byte(uint8_t x) {
    for (uint8_t i = 0x80; i != 0; i >>= 1) {
        if (x & i)
            PINOP(BOARD_RGBLED_DATA_PIN, OUTSET);
        else
            PINOP(BOARD_RGBLED_DATA_PIN, OUTCLR);

        PINOP(BOARD_RGBLED_CLOCK_PIN, OUTSET);
        // for (uint8_t j=0; j<25; j++) /* 0.1ms */
        //  __asm__ __volatile__("");

        PINOP(BOARD_RGBLED_CLOCK_PIN, OUTCLR);
        // for (uint8_t j=0; j<25; j++) /* 0.1ms */
        //  __asm__ __volatile__("");
    }
}
#endif

void RGBLED_set_color(uint32_t color) {
#if defined(BOARD_RGBLED_CLOCK_PIN)
    write_apa_byte(0x0);
    write_apa_byte(0x0);
    write_apa_byte(0x0);
    write_apa_byte(0x0);

    write_apa_byte(0xFF);
    write_apa_byte(color >> 16);
    write_apa_byte(color >> 8);
    write_apa_byte(color);

    write_apa_byte(0xFF);
    write_apa_byte(0xFF);
    write_apa_byte(0xFF);
    write_apa_byte(0xFF);

    // set clock port low for ~10ms
    delay(50);
#elif defined(BOARD_NEOPIXEL_PIN)
    uint8_t buf[BOARD_NEOPIXEL_COUNT * 3];
#if 0
    memset(buf, 0, sizeof(buf));
    buf[0] = color >> 8;
    buf[1] = color >> 16;
    buf[2] = color;
#else
    for (int i = 0; i < BOARD_NEOPIXEL_COUNT * 3; i += 3) {
        buf[i + 0] = color >> 8;
        buf[i + 1] = color >> 16;
        buf[i + 2] = color;
    }
#endif
    neopixel_send_buffer(buf, BOARD_NEOPIXEL_COUNT * 3);
#endif
}

#if defined(CLEARCORE_ID)
// - - - - - - - - - - - - - - - - - - - - -
// Teknic ClearCore Hardware display driver
// - - - - - - - - - - - - - - - - - - - - -

// LED pattern and state sequencer
#define NVIC_ST_CURRENT (*((volatile uint32_t *)0xE000E018))
// Initial LED display during upgrade
void clearCoreFlashLEDinit(void)
{
    clearCoreLEDs = (SR_LED_INIT_PATTERN);
    clearCoreReady = true;
    clearCoreLEDsend();
}

// Scan LEDs during flash process
void clearCoreFlashLEDprogress(bool reverse) 
{
    uint32_t scanNow;
    if (clearCoreLEDs & SR_SCANNER_MASK) {
        if (reverse) 
            scanNow = (clearCoreLEDs<<1) & SR_SCANNER_MASK;
        else
            scanNow = (clearCoreLEDs>>1) & SR_SCANNER_MASK;
        clearCoreLEDs &= ~(SR_SCANNER_MASK);
        if (scanNow==0) {
            scanNow = reverse ? SR_LED_IO_5 : SR_LED_IO_0;
        }
        clearCoreLEDs |= scanNow;
    }
    clearCoreLEDsend();
}

// Cause LED scan pattern when boot loader is running
void clearCoreLEDseq()
{
    if (clearCoreTicks > 3000) {
        // Connected? 
        if (main_b_cdc_enable) {
            clearCoreLEDs |= SR_MSC_LED;
        }
        else {
            clearCoreLEDs &= ~SR_MSC_LED;
        }
        clearCoreFlashLEDprogress(main_b_cdc_enable);

         // Delay until next shift
        clearCoreTicks = 0;
    }
    else clearCoreTicks++;
}

// Send next LED state
void clearCoreLEDsend()
{
    if (!clearCoreReady) {
        return;
    }
    uint32_t ledMask = (ledState) ? UINT32_MAX : ~SR_MSC_LED;
    // Send the next values to the chain
    SERCOM6->SPI.DATA.reg = (clearCoreLEDs & ledMask) ^ SR_INVERSIONS;
    // Wait for TX-complete interrupt flag in case we get here too quickly
    while(!(SERCOM6->SPI.INTFLAG.bit.TXC)) {};

    // Strobe the output with minimum pulse width to display last transfer
    PORT->Group[1].OUTSET.reg = SR_LED_STROBE;
    PORT->Group[1].OUTCLR.reg = SR_LED_STROBE;
}

// Setup hardware to use SPI for LED shift chain
void clearCoreInit() {
    clearCoreLEDs = SR_LED_WAIT_PATTERN;
    ledState = false;
    
    // Set the clock source for SERCOM6 to GCLK0 and enable the peripheral channel
    GCLK->PCHCTRL[SERCOM6_GCLK_ID_CORE].reg = GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN;		
    // Enable the clock and sync with core
    MCLK->APBDMASK.reg |= MCLK_APBDMASK_SERCOM6;

    // Setup pins for SERCOM6 in SPI master mode and enable it to control them
    PORT->Group[2].DIRSET.reg = (0x1 << 5) | (0x1 << 7);
    PORT->Group[2].PMUX[5 >> 1].reg |= PORT_PMUX_PMUXO(0x2);
    PORT->Group[2].PMUX[7 >> 1].reg |= PORT_PMUX_PMUXO(0x2);
    PORT->Group[2].PINCFG[5].reg |= PORT_PINCFG_PMUXEN;
    PORT->Group[2].PINCFG[7].reg |= PORT_PINCFG_PMUXEN;

    // Force a data load on B09
    PORT->Group[1].OUTCLR.reg = SR_LED_STROBE;
    PORT->Group[1].DIRSET.reg = SR_LED_STROBE;

    // Disable SERCOM6 to switch its role
    SERCOM6->SPI.CTRLA.bit.ENABLE = 0;
    while(SERCOM6->SPI.SYNCBUSY.reg) {};	// Wait for disable sync
    // Sets SERCOM0 to SPI Master mode
    SERCOM6->SPI.CTRLA.reg |= SERCOM_SPI_CTRLA_MODE(0x3);
    // Sets PAD[3] to DO, PAD[2] to DI, and sets LSB-first transmission
    SERCOM6->SPI.CTRLA.reg |= SERCOM_SPI_CTRLA_DOPO(0x2) | SERCOM_SPI_CTRLA_DIPO(0x2)
                    | SERCOM_SPI_CTRLA_DORD;

    // Enables 32bit DATA register transactions
    SERCOM6->SPI.CTRLC.reg |= SERCOM_SPI_CTRLC_DATA32B;

    // Sets the baud rate to GCLK0/(2(x+1))
    SERCOM6->SPI.BAUD.reg = 0;

    // Enables SERCOM6 and wait for core sync
    SERCOM6->SPI.CTRLA.bit.ENABLE = 1;
    while(SERCOM6->SPI.SYNCBUSY.reg) {};

    clearCoreReady = true;
    // Send the initial values to the chain
    clearCoreLEDsend();
    // Enable the chain, clear & set SR_ENn
    PORT->Group[1].OUTCLR.reg = SR_LED_ENABLE;
    PORT->Group[1].DIRSET.reg = SR_LED_ENABLE;
}
#endif //defined(CLEARCORE_ID)