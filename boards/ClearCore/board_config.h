#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include <stdint.h>

#define VENDOR_NAME "Teknic, Inc."
#define PRODUCT_NAME "ClearCore"
#define VOLUME_LABEL "CLEAR_BOOT"
#define INDEX_URL "https://www.teknic.com"
// This macro delimits all ClearCore implementations
#define CLEARCORE_ID "SAME53N19A-ClearCore-F1"
#define BOARD_ID     CLEARCORE_ID

#define USB_VID 0x2890          // Teknic
#define USB_PID 0x0022

#define BOOT_USART_MODULE                 SERCOM0
#define BOOT_USART_MASK                   APBAMASK
#define BOOT_USART_BUS_CLOCK_INDEX        MCLK_APBAMASK_SERCOM0
#define BOOT_USART_PAD_SETTINGS           UART_RX_PAD1_TX_PAD0
#define BOOT_USART_PAD3                   PINMUX_UNUSED
#define BOOT_USART_PAD2                   PINMUX_UNUSED
#define BOOT_USART_PAD1                   PINMUX_PA09C_SERCOM0_PAD1
#define BOOT_USART_PAD0                   PINMUX_PA08C_SERCOM0_PAD0
#define BOOT_GCLK_ID_CORE                 SERCOM0_GCLK_ID_CORE
#define BOOT_GCLK_ID_SLOW                 SERCOM0_GCLK_ID_SLOW

// LED Assignments
#define SR_A_CTRL_3         (1U << 0)
#define SR_A_CTRL_2         (1U << 1)
#define SR_LED_IO_5         (1U << 2)
#define SR_LED_IO_4         (1U << 3)
#define SR_LED_IO_3         (1U << 4)
#define SR_LED_IO_2         (1U << 5)
#define SR_LED_IO_1         (1U << 6)
#define SR_LED_IO_0         (1U << 7)
#define SR_EN_OUT_3         (1U << 8)
#define SR_EN_OUT_2         (1U << 9)
#define SR_EN_OUT_1         (1U << 10)
#define SR_EN_OUT_0         (1U << 11)
#define SR_UART_INV_1       (1U << 12)
#define SR_UART_INV_0       (1U << 13)
#define SR_UNDERGLOW        (1U << 14)
#define SR_LED_USB          (1U << 15)
#define SR_UART_SPI_SEL_1   (1U << 16)
#define SR_UART_SPI_SEL_0   (1U << 17)
#define SR_LED_COM_0        (1U << 18)
#define SR_LED_COM_1        (1U << 19)
#define SR_HPIO_CTRL_0      (1U << 20)
#define SR_LED_DI_6         (1U << 21)
#define SR_LED_DI_7         (1U << 22)
#define SR_LED_DI_8         (1U << 23)
#define SR_LED_ADI_12       (1U << 24)
#define SR_LED_ADI_11       (1U << 25)
#define SR_LED_ADI_10       (1U << 26)
#define SR_LED_ADI_9        (1U << 27)
#define SR_ADIN_CONFIG_12   (1U << 28)
#define SR_ADIN_CONFIG_11   (1U << 29)
#define SR_ADIN_CONFIG_10   (1U << 30)
#define SR_ADIN_CONFIG_9    (1U << 31)

#define  SR_MSC_LED         (SR_UNDERGLOW)

#define SR_SCANNER_MASK     (SR_LED_IO_5 | SR_LED_IO_4 | SR_LED_IO_3 | \
                             SR_LED_IO_2 | SR_LED_IO_1 | SR_LED_IO_0)
#define SR_LED_INIT_PATTERN (SR_SCANNER_MASK | SR_MSC_LED)
#define SR_LED_WAIT_PATTERN (SR_MSC_LED)

#define SR_INVERSIONS    (~(SR_LED_USB | SR_LED_IO_4 | SR_LED_IO_5 | \
                            SR_LED_COM_0 | SR_LED_COM_1 | SR_ADIN_CONFIG_9 | \
                            SR_ADIN_CONFIG_10 | SR_ADIN_CONFIG_11 | \
                            SR_ADIN_CONFIG_12 | SR_UNDERGLOW | SR_EN_OUT_0 | \
                            SR_EN_OUT_1 | SR_EN_OUT_2 | SR_EN_OUT_3 | \
                            SR_UART_INV_1 | SR_UART_INV_0))

#define SR_LED_STROBE       (1<<2)
#define SR_LED_ENABLE       (1<<1)

// Initialize the hardware
void clearCoreInit(void);
// Teknic ClearCore LED string API
void clearCoreLEDseq(void);
// Low level sender
void clearCoreLEDsend(void);
// Each call updates the IO LED patterns
void clearCoreFlashLEDprogress(_Bool reverse);
// Setup the initial LED pattern
void clearCoreFlashLEDinit(void);

#endif
