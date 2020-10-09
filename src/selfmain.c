#include "uf2.h"

#include "sam.h"

#ifdef SAMD21
#define BOOTLOADER_K 8
#endif
#if defined(SAMD51) || defined(SAME53)
#define BOOTLOADER_K 16
#endif

extern const uint8_t bootloader[];
extern const uint16_t bootloader_crcs[];

uint8_t pageBuf[FLASH_ROW_SIZE];
bool main_b_cdc_enable = true;
#if defined(CLEARCORE_ID)
void smartEepromInitialize(void); 
#endif


#ifdef SAMD21
#define NVM_FUSE_ADDR NVMCTRL_AUX0_ADDRESS
#define exec_cmd(cmd)                                                          \
    do {                                                                       \
        NVMCTRL->STATUS.reg |= NVMCTRL_STATUS_MASK;                            \
        NVMCTRL->ADDR.reg = (uint32_t)NVMCTRL_USER / 2;                        \
        NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | cmd;                    \
        while (NVMCTRL->INTFLAG.bit.READY == 0) {}                             \
    } while (0)
#endif
#if defined(SAMD51) || defined(SAME53)
#define NVM_FUSE_ADDR NVMCTRL_FUSES_BOOTPROT_ADDR
#define exec_cmd(cmd)                                                          \
    do {                                                                       \
        NVMCTRL->ADDR.reg = (uint32_t)NVMCTRL_USER;                        \
        NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMDEX_KEY | cmd;                    \
        while (NVMCTRL->STATUS.bit.READY == 0) {}                              \
    } while (0)
#endif


void setBootProt(int v) {
    uint32_t buf[FLASH_USER_PAGE_SIZE / sizeof(uint32_t)];
    memcpy(buf, (void *) NVMCTRL_USER, FLASH_USER_PAGE_SIZE);

    #ifdef SAMD21
    while (!(NVMCTRL->INTFLAG.reg & NVMCTRL_INTFLAG_READY)) {}
    #endif
    #if defined(SAMD51) || defined(SAME53)
    while (NVMCTRL->STATUS.bit.READY == 0) {}
    #endif

    uint32_t bootprot = (buf[0] & NVMCTRL_FUSES_BOOTPROT_Msk) >> NVMCTRL_FUSES_BOOTPROT_Pos;

    logval("fuse0", buf[0]);
    logval("fuse1", buf[1]);
    logval("bootprot", bootprot);
    logval("needed", v);

    if (bootprot == v)
        return;

    // change the in-memory copy of the user page
    buf[0] = (buf[0] & ~NVMCTRL_FUSES_BOOTPROT_Msk) | (v << NVMCTRL_FUSES_BOOTPROT_Pos);

    #ifdef SAMD21
    NVMCTRL->CTRLB.reg = NVMCTRL->CTRLB.reg | NVMCTRL_CTRLB_CACHEDIS | NVMCTRL_CTRLB_MANW;

    exec_cmd(NVMCTRL_CTRLA_CMD_EAR);
    exec_cmd(NVMCTRL_CTRLA_CMD_PBC);
    #endif
    #if defined(SAMD51) || defined(SAME53)
    NVMCTRL->CTRLA.bit.WMODE = NVMCTRL_CTRLA_WMODE_MAN;

    NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMDEX_KEY | NVMCTRL_CTRLB_CMD_PBC;
    while (NVMCTRL->STATUS.bit.READY == 0) {}

    NVMCTRL->ADDR.reg = NVMCTRL_USER;
    NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMDEX_KEY | NVMCTRL_CTRLB_CMD_EP;
    while (NVMCTRL->STATUS.bit.READY == 0) {}
    #endif

    // copy the in-memory copy of the user page into the page buffer for writing
    uint32_t *dest = (uint32_t *) NVMCTRL_USER;
    uint32_t *src = (uint32_t *) buf;
    uint16_t wordsLeft = FLASH_USER_PAGE_SIZE / sizeof(uint32_t);
    while (wordsLeft) {
        for (uint8_t i = 0; i < 4; ++i) {
            *dest = *src;
            dest++;
            src++;
            wordsLeft--;
        }
        // bombs away
        #ifdef SAMD21
        exec_cmd(NVMCTRL_CTRLA_CMD_WAP);
        #endif
        #if defined(SAMD51) || defined(SAME53)

        NVMCTRL->ADDR.reg = (uint32_t) (dest - 4);
        NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMDEX_KEY | NVMCTRL_CTRLB_CMD_WQW;
        while (NVMCTRL->STATUS.bit.READY == 0) {}
        #endif
    }

    resetIntoApp();
}

int main(void) {
    led_init();

    logmsg("Start");

    assert((8 << NVMCTRL->PARAM.bit.PSZ) == FLASH_PAGE_SIZE);
    // assert(FLASH_PAGE_SIZE * NVMCTRL->PARAM.bit.NVMP == FLASH_SIZE);

    /* We have determined we should stay in the monitor. */
    /* System initialization */
    system_init();
    __disable_irq();
    __DMB();

    logmsg("Before main loop");
    #if defined(CLEARCORE_ID)
      // Setup initial lightshow
      clearCoreInit();
      clearCoreFlashLEDinit();
    #endif

    #ifdef SAMD21
    setBootProt(7); // 0k
    #endif
    #if defined(SAMD51) || defined(SAME53)
    // We only need to set the BOOTPROT once on the SAMD51. For updates, we can
    // temporarily turn the protection off instead.
    if (NVMCTRL->STATUS.bit.BOOTPROT != 13) {
        setBootProt(13); // 16k
    }

    exec_cmd(NVMCTRL_CTRLB_CMD_SBPDIS);
    NVMCTRL->CTRLA.bit.CACHEDIS0 = true;
    NVMCTRL->CTRLA.bit.CACHEDIS1 = true;
    #endif

    const uint8_t *ptr = bootloader;
    int i;

    for (i = 0; i < BOOTLOADER_K; ++i) {
        int crc = 0;
        for (int j = 0; j < 1024; ++j) {
            crc = add_crc(*ptr++, crc);
        }
        if (bootloader_crcs[i] != crc) {
            logmsg("Invalid checksum. Aborting.");
            panic(1);
        }
    }

    for (i = 0; i < BOOTLOADER_K * 1024; i += FLASH_ROW_SIZE) {
        memcpy(pageBuf, &bootloader[i], FLASH_ROW_SIZE);
        flash_write_row((void *)i, (void *)pageBuf);
    }

    logmsg("Update successful!");

    // re-base int vector back to bootloader, so that the flash erase below doesn't write over the
    // vectors
    SCB->VTOR = 0;

    // Write zeros to the stack location and reset handler location so the
    // bootloader doesn't run us a second time. We don't need to erase to write
    // zeros. The remainder of the write unit will be set to 1s which should
    // preserve the existing values but its not critical.
    uint32_t zeros[2] = {0, 0};

    flash_write_words((void *)(BOOTLOADER_K * 1024), zeros, 2);

 #if defined(CLEARCORE_ID)
    delay(1000);
 #else
    for (i = 0; i < 20; ++i) {
        LED_MSC_TGL();
        delay(1000);
   }
 #endif

    LED_MSC_OFF();

    #ifdef SAMD21
    setBootProt(2); // 8k
    #endif
    // For the SAMD51, the boot protection will automatically be re-enabled on
    // reset.
    #if defined(CLEARCORE_ID)
        clearCoreInit();
    #endif

    resetIntoBootloader();

    while (1) {
    }
}
