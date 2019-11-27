/************************************************************************************
 * configs/coordinator/include/board.h
 *
 *   Copyright (C) 2016-2017 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *           Mark Olsson <post@markolsson.se>
 *           David Sidrane <david_s5@nscdg.com>
 *           Bob Feretich <bob.feretich@rafresearch.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __CONFIG_NUCLEO_144_INCLUDE_BOARD_H
#define __CONFIG_NUCLEO_144_INCLUDE_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

#ifdef __KERNEL__
#include "stm32_rcc.h"
#ifdef CONFIG_STM32F7_SDMMC1
#  include "stm32_sdmmc.h"
#endif
#endif

/* #include "hardware/stm32_pinmap.h" */

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/

/* The Nucleo-144  board provides the following clock sources:
 *
 *   MCO: 8 MHz from MCO output of ST-LINK is used as input clock
 *   X2:  32.768 KHz crystal for LSE
 *   X3:  HSE crystal oscillator (not provided)
 *
 * So we have these clock source available within the STM32
 *
 *   HSI: 16 MHz RC factory-trimmed
 *   LSI: 32 KHz RC
 *   HSE: 8 MHz from MCO output of ST-LINK
 *   LSE: 32.768 kHz
 */

#define STM32_BOARD_XTAL        8000000ul

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY     32768

/* Main PLL Configuration.
 *
 * PLL source is HSE = 8,000,000
 *
 * PLL_VCO = (STM32_HSE_FREQUENCY / PLLM) * PLLN
 * Subject to:
 *
 *     2 <= PLLM <= 63
 *   192 <= PLLN <= 432
 *   192 MHz <= PLL_VCO <= 432MHz
 *
 * SYSCLK  = PLL_VCO / PLLP
 * Subject to
 *
 *   PLLP = {2, 4, 6, 8}
 *   SYSCLK <= 216 MHz
 *
 * USB OTG FS, SDMMC and RNG Clock = PLL_VCO / PLLQ
 * Subject to
 *   The USB OTG FS requires a 48 MHz clock to work correctly. The SDMMC
 *   and the random number generator need a frequency lower than or equal
 *   to 48 MHz to work correctly.
 *
 * 2 <= PLLQ <= 15
 */

/* Highest SYSCLK with USB OTG FS clock = 48 MHz
 *
 * PLL_VCO = (8,000,000 / 4) * 216 = 432 MHz
 * SYSCLK  = 432 MHz / 2 = 216 MHz
 * USB OTG FS, SDMMC and RNG Clock = 432 MHz / 9 = 48 MHz
 */

#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(4)
#define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(216)
#define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP_2
#define STM32_PLLCFG_PLLQ       RCC_PLLCFG_PLLQ(9)

#define STM32_VCO_FREQUENCY     ((STM32_HSE_FREQUENCY / 4) * 216)
#define STM32_SYSCLK_FREQUENCY  (STM32_VCO_FREQUENCY / 2)
#define STM32_OTGFS_FREQUENCY   (STM32_VCO_FREQUENCY / 9)

/* Configure factors for  PLLSAI clock */

#define CONFIG_STM32F7_PLLSAI 1
#define STM32_RCC_PLLSAICFGR_PLLSAIN    RCC_PLLSAICFGR_PLLSAIN(192)
#define STM32_RCC_PLLSAICFGR_PLLSAIP    RCC_PLLSAICFGR_PLLSAIP(8)
#define STM32_RCC_PLLSAICFGR_PLLSAIQ    RCC_PLLSAICFGR_PLLSAIQ(4)
#define STM32_RCC_PLLSAICFGR_PLLSAIR    RCC_PLLSAICFGR_PLLSAIR(2)

/* Configure Dedicated Clock Configuration Register */

#define STM32_RCC_DCKCFGR1_PLLI2SDIVQ  RCC_DCKCFGR1_PLLI2SDIVQ(1)
#define STM32_RCC_DCKCFGR1_PLLSAIDIVQ  RCC_DCKCFGR1_PLLSAIDIVQ(1)
#define STM32_RCC_DCKCFGR1_PLLSAIDIVR  RCC_DCKCFGR1_PLLSAIDIVR(0)
#define STM32_RCC_DCKCFGR1_SAI1SRC     RCC_DCKCFGR1_SAI1SEL(0)
#define STM32_RCC_DCKCFGR1_SAI2SRC     RCC_DCKCFGR1_SAI2SEL(0)
#define STM32_RCC_DCKCFGR1_TIMPRESRC   0
#define STM32_RCC_DCKCFGR1_DFSDM1SRC   0
#define STM32_RCC_DCKCFGR1_ADFSDM1SRC  0

/* Configure factors for  PLLI2S clock */

#define CONFIG_STM32F7_PLLI2S 1
#define STM32_RCC_PLLI2SCFGR_PLLI2SN   RCC_PLLI2SCFGR_PLLI2SN(192)
#define STM32_RCC_PLLI2SCFGR_PLLI2SP   RCC_PLLI2SCFGR_PLLI2SP(2)
#define STM32_RCC_PLLI2SCFGR_PLLI2SQ   RCC_PLLI2SCFGR_PLLI2SQ(2)
#define STM32_RCC_PLLI2SCFGR_PLLI2SR   RCC_PLLI2SCFGR_PLLI2SR(2)

/* Configure Dedicated Clock Configuration Register 2 */

#define STM32_RCC_DCKCFGR2_USART1SRC  RCC_DCKCFGR2_USART1SEL_APB
#define STM32_RCC_DCKCFGR2_USART2SRC  RCC_DCKCFGR2_USART2SEL_APB
#define STM32_RCC_DCKCFGR2_UART4SRC   RCC_DCKCFGR2_UART4SEL_APB
#define STM32_RCC_DCKCFGR2_UART5SRC   RCC_DCKCFGR2_UART5SEL_APB
#define STM32_RCC_DCKCFGR2_USART6SRC  RCC_DCKCFGR2_USART6SEL_APB
#define STM32_RCC_DCKCFGR2_UART7SRC   RCC_DCKCFGR2_UART7SEL_APB
#define STM32_RCC_DCKCFGR2_UART8SRC   RCC_DCKCFGR2_UART8SEL_APB
#define STM32_RCC_DCKCFGR2_I2C1SRC    RCC_DCKCFGR2_I2C1SEL_HSI
#define STM32_RCC_DCKCFGR2_I2C2SRC    RCC_DCKCFGR2_I2C2SEL_HSI
#define STM32_RCC_DCKCFGR2_I2C3SRC    RCC_DCKCFGR2_I2C3SEL_HSI
#define STM32_RCC_DCKCFGR2_I2C4SRC    RCC_DCKCFGR2_I2C4SEL_HSI
#define STM32_RCC_DCKCFGR2_LPTIM1SRC  RCC_DCKCFGR2_LPTIM1SEL_APB
#define STM32_RCC_DCKCFGR2_CECSRC     RCC_DCKCFGR2_CECSEL_HSI
#define STM32_RCC_DCKCFGR2_CK48MSRC   RCC_DCKCFGR2_CK48MSEL_PLL
#define STM32_RCC_DCKCFGR2_SDMMCSRC   RCC_DCKCFGR2_SDMMCSEL_48MHZ
#define STM32_RCC_DCKCFGR2_SDMMC2SRC  RCC_DCKCFGR2_SDMMC2SEL_48MHZ
#define STM32_RCC_DCKCFGR2_DSISRC     RCC_DCKCFGR2_DSISEL_PHY


/* Several prescalers allow the configuration of the two AHB buses, the
 * high-speed APB (APB2) and the low-speed APB (APB1) domains. The maximum
 * frequency of the two AHB buses is 216 MHz while the maximum frequency of
 * the high-speed APB domains is 108 MHz. The maximum allowed frequency of
 * the low-speed APB domain is 54 MHz.
 */

/* AHB clock (HCLK) is SYSCLK (216 MHz) */

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK  /* HCLK  = SYSCLK / 1 */
#define STM32_HCLK_FREQUENCY    STM32_SYSCLK_FREQUENCY
#define STM32_BOARD_HCLK        STM32_HCLK_FREQUENCY  /* same as above, to satisfy compiler */

/* APB1 clock (PCLK1) is HCLK/4 (54 MHz) */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd4     /* PCLK1 = HCLK / 4 */
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY/4)

/* Timers driven from APB1 will be twice PCLK1 */

#define STM32_APB1_TIM2_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM5_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM12_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM13_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM14_CLKIN  (2*STM32_PCLK1_FREQUENCY)

/* APB2 clock (PCLK2) is HCLK/2 (108MHz) */

#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLKd2     /* PCLK2 = HCLK / 2 */
#define STM32_PCLK2_FREQUENCY   (STM32_HCLK_FREQUENCY/2)

/* Timers driven from APB2 will be twice PCLK2 */

#define STM32_APB2_TIM1_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM8_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM9_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM10_CLKIN  (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM11_CLKIN  (2*STM32_PCLK2_FREQUENCY)

/* SDMMC dividers.  Note that slower clocking is required when DMA is disabled
 * in order to avoid RX overrun/TX underrun errors due to delayed responses
 * to service FIFOs in interrupt driven mode.  These values have not been
 * tuned!!!
 *
 * SDMMCCLK=48MHz, SDMMC_CK=SDMMCCLK/(118+2)=400 KHz
 */

#define STM32_SDMMC_INIT_CLKDIV         (118 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)

/* DMA ON:  SDMMCCLK=48MHz, SDMMC_CK=SDMMCCLK/(1+2)=16 MHz
 * DMA OFF: SDMMCCLK=48MHz, SDMMC_CK=SDMMCCLK/(2+2)=12 MHz
 */

#ifdef CONFIG_SDIO_DMA
#  define STM32_SDMMC_MMCXFR_CLKDIV     (1 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)
#else
#  define STM32_SDMMC_MMCXFR_CLKDIV     (2 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)
#endif

/* DMA ON:  SDMMCCLK=48MHz, SDMMC_CK=SDMMCCLK/(1+2)=16 MHz
 * DMA OFF: SDMMCCLK=48MHz, SDMMC_CK=SDMMCCLK/(2+2)=12 MHz
 */

#ifdef CONFIG_SDIO_DMA
#  define STM32_SDMMC_SDXFR_CLKDIV      (1 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)
#else
#  define STM32_SDMMC_SDXFR_CLKDIV      (2 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)
#endif

#if defined(CONFIG_STM32F7_SDMMC2)
#  define GPIO_SDMMC2_D0 GPIO_SDMMC2_D0_1
#  define GPIO_SDMMC2_D1 GPIO_SDMMC2_D1_1
#  define GPIO_SDMMC2_D2 GPIO_SDMMC2_D2_1
#  define GPIO_SDMMC2_D3 GPIO_SDMMC2_D3_1
#endif

/* DMA Channl/Stream Selections *****************************************************/

/* Stream selections are arbitrary for now but might become important in the future
 * if we set aside more DMA channels/streams.
 *
 * SDMMC DMA is on DMA2
 *
 * SDMMC1 DMA
 *   DMAMAP_SDMMC1_1 = Channel 4, Stream 3
 *   DMAMAP_SDMMC1_2 = Channel 4, Stream 6
 *
 * SDMMC2 DMA
 *   DMAMAP_SDMMC2_1 = Channel 11, Stream 0
 *   DMAMAP_SDMMC3_2 = Channel 11, Stream 5
 */

#define DMAMAP_SDMMC1  DMAMAP_SDMMC1_1
#define DMAMAP_SDMMC2  DMAMAP_SDMMC2_1


/* FLASH wait states
 *
 *  --------- ---------- -----------
 *  VDD       MAX SYSCLK WAIT STATES
 *  --------- ---------- -----------
 *  1.7-2.1 V   180 MHz    8
 *  2.1-2.4 V   216 MHz    9
 *  2.4-2.7 V   216 MHz    8
 *  2.7-3.6 V   216 MHz    7
 *  --------- ---------- -----------
 */

#define BOARD_FLASH_WAITSTATES 7

/* LED definitions ******************************************************************/

/* The Nucleo-144 board has numerous LEDs but only three, LD1 a Green LED, LD2 a Blue
 * LED and LD3 a Red LED, that can be controlled by software. The following
 * definitions assume the default Solder Bridges are installed.
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in any way.
 * The following definitions are used to access individual LEDs.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED1        0
#define BOARD_LED2        1
#define BOARD_LED3        2
#define BOARD_NLEDS       3

#define BOARD_LED_GREEN   BOARD_LED1
#define BOARD_LED_BLUE    BOARD_LED2
#define BOARD_LED_RED     BOARD_LED3

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT    (1 << BOARD_LED1)
#define BOARD_LED2_BIT    (1 << BOARD_LED2)
#define BOARD_LED3_BIT    (1 << BOARD_LED3)

/* If CONFIG_ARCH_LEDS is defined, the usage by the board port is defined in
 * include/board.h and src/stm32_leds.c. The LEDs are used to encode OS-related
 * events as follows:
 *
 *
 *   SYMBOL                     Meaning                      LED state
 *                                                        Red   Green Blue
 *   ----------------------  --------------------------  ------ ------ ----*/

#define LED_STARTED        0 /* NuttX has been started   OFF    OFF   OFF  */
#define LED_HEAPALLOCATE   1 /* Heap has been allocated  OFF    OFF   ON   */
#define LED_IRQSENABLED    2 /* Interrupts enabled       OFF    ON    OFF  */
#define LED_STACKCREATED   3 /* Idle stack created       OFF    ON    ON   */
#define LED_INIRQ          4 /* In an interrupt          N/C    N/C   GLOW */
#define LED_SIGNAL         5 /* In a signal handler      N/C    GLOW  N/C  */
#define LED_ASSERTION      6 /* An assertion failed      GLOW   N/C   GLOW */
#define LED_PANIC          7 /* The system has crashed   Blink  OFF   N/C  */
#define LED_IDLE           8 /* MCU is is sleep mode     ON     OFF   OFF  */

/* Thus if the Green LED is statically on, NuttX has successfully booted and
 * is, apparently, running normally.  If the Red LED is flashing at
 * approximately 2Hz, then a fatal error has been detected and the system
 * has halted.
 */

/* Button definitions ***************************************************************/

/* The STM32F7 Discovery supports one button:  Pushbutton B1, labeled "User", is
 * connected to GPIO PI11.  A high value will be sensed when the button is depressed.
 */

#define BUTTON_USER        0
#define NUM_BUTTONS        1
#define BUTTON_USER_BIT    (1 << BUTTON_USER)

/* Alternate function pin selections ************************************************/

/* TIM */

#define GPIO_TIM1_CH1OUT GPIO_TIM1_CH1OUT_1
#define GPIO_TIM2_CH1OUT GPIO_TIM2_CH1OUT_1
#define GPIO_TIM3_CH1OUT GPIO_TIM3_CH1OUT_1
#define GPIO_TIM4_CH1OUT GPIO_TIM4_CH1OUT_1

/* USART3:
 * Use  USART3 and the USB virtual COM port
 */

# define GPIO_USART3_RX GPIO_USART3_RX_3
# define GPIO_USART3_TX GPIO_USART3_TX_3

/* DMA channels *************************************************************/

/* ADC */

#define ADC1_DMA_CHAN DMAMAP_ADC1_1
#define ADC2_DMA_CHAN DMAMAP_ADC2_1
#define ADC3_DMA_CHAN DMAMAP_ADC3_1

/* SPI
 *
 *
 *  PA6   SPI1_MISO CN12-13
 *  PA7   SPI1_MOSI CN12-15
 *  PA5   SPI1_SCK  CN12-11
 *
 *  PB14  SPI2_MISO CN12-28
 *  PB15  SPI2_MOSI CN12-26
 *  PB13  SPI2_SCK  CN12-30
 *
 *  PB4   SPI3_MISO CN12-27
 *  PB5   SPI3_MOSI CN12-29
 *  PB3   SPI3_SCK  CN12-31
 */

#define GPIO_SPI1_MISO   GPIO_SPI1_MISO_1
#define GPIO_SPI1_MOSI   GPIO_SPI1_MOSI_1
#define GPIO_SPI1_SCK    GPIO_SPI1_SCK_1

#define GPIO_SPI2_MISO   GPIO_SPI2_MISO_1
#define GPIO_SPI2_MOSI   GPIO_SPI2_MOSI_1
#define GPIO_SPI2_SCK    GPIO_SPI2_SCK_3

#define GPIO_SPI3_MISO   GPIO_SPI3_MISO_1
#define GPIO_SPI3_MOSI   GPIO_SPI3_MOSI_2
#define GPIO_SPI3_SCK    GPIO_SPI3_SCK_1

/* I2C
 *
 *
 *  PB8   I2C1_SCL CN12-3
 *  PB9   I2C1_SDA CN12-5

 *  PB10   I2C2_SCL CN11-51
 *  PB11 I2C2_SDA CN12-18
 *
 *  PA8   I2C3_SCL CN12-23
 *  PC9   I2C3_SDA CN12-1
 *
 */

#define GPIO_I2C1_SCL GPIO_I2C1_SCL_2
#define GPIO_I2C1_SDA GPIO_I2C1_SDA_2

#define GPIO_I2C2_SCL (GPIO_I2C2_SCL_1)
#define GPIO_I2C2_SDA (GPIO_I2C2_SDA_1)

#define GPIO_I2C3_SCL GPIO_I2C3_SCL_1
#define GPIO_I2C3_SDA GPIO_I2C3_SDA_1

/* The STM32 F7 connects to a SMSC LAN8742A PHY using these pins:
 *
 *   STM32 F7 BOARD        LAN8742A
 *   GPIO     SIGNAL       PIN NAME
 *   -------- ------------ -------------
 *   PG11     RMII_TX_EN   TXEN
 *   PG13     RMII_TXD0    TXD0
 *   PB13     RMII_TXD1    TXD1
 *   PC4      RMII_RXD0    RXD0/MODE0
 *   PC5      RMII_RXD1    RXD1/MODE1
 *   PG2      RMII_RXER    RXER/PHYAD0 -- Not used
 *   PA7      RMII_CRS_DV  CRS_DV/MODE2
 *   PC1      RMII_MDC     MDC
 *   PA2      RMII_MDIO    MDIO
 *   N/A      NRST         nRST
 *   PA1      RMII_REF_CLK nINT/REFCLK0
 *   N/A      OSC_25M      XTAL1/CLKIN
 *
 * The PHY address is either 0 or 1, depending on the state of PG2 on reset.
 * PG2 is not controlled but appears to result in a PHY address of 0.
 */

#define GPIO_ETH_RMII_TX_EN   GPIO_ETH_RMII_TX_EN_2
#define GPIO_ETH_RMII_TXD0    GPIO_ETH_RMII_TXD0_2
#define GPIO_ETH_RMII_TXD1    GPIO_ETH_RMII_TXD1_1

/* Encoders:
 *
 * This configuration assume that SB13 (Ethernet signal jumper) was
 * de-soldered. All Ethernet function is disabled here due to 
 * I/O conflict (PA1 for specific).
 * 
 * Pin map:
 * Encoders: PA0, PA1, PA15, PB3, PB4, PB5, PB6, PB7, PC6, PC7, PE9, PE11
 *    Enc0: GPIO_TIM1_CH1IN_2 (PE9) + GPIO_TIM1_CH2IN_2 (PE11)
 *    Enc1: GPIO_TIM2_CH1IN_2 (PA15) + GPIO_TIM2_CH2IN_2 (PB3)
 *    Enc2: GPIO_TIM3_CH1IN_2 (PB4) + GPIO_TIM3_CH2IN_2 (PB5)
 *    Enc3: GPIO_TIM4_CH1IN_1 (PB6) + GPIO_TIM4_CH2IN_1 (PB7)
 *    Enc4: GPIO_TIM5_CH1IN_1 (PA0) + GPIO_TIM5_CH2IN_1 (PA1)
 *    Enc5: GPIO_TIM8_CH1IN_1 (PC6) + GPIO_TIM8_CH2IN_1 (PC7)
 *
 */
#define GPIO_TIM1_CH1IN     (GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN9)
#define GPIO_TIM1_CH2IN     (GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN11)
#define GPIO_TIM2_CH1IN     (GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_PULLUP|GPIO_PORTA|GPIO_PIN15)
#define GPIO_TIM2_CH2IN     (GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN3)
#define GPIO_TIM3_CH1IN     (GPIO_ALT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN4)
#define GPIO_TIM3_CH2IN     (GPIO_ALT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN5)
#define GPIO_TIM4_CH1IN     (GPIO_ALT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_PULLUP|GPIO_PORTD|GPIO_PIN12)
#define GPIO_TIM4_CH2IN     (GPIO_ALT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_PULLUP|GPIO_PORTD|GPIO_PIN13)
#define GPIO_TIM5_CH1IN     (GPIO_ALT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_PULLUP|GPIO_PORTA|GPIO_PIN0)
#define GPIO_TIM5_CH2IN     (GPIO_ALT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_PULLUP|GPIO_PORTA|GPIO_PIN1)
#define GPIO_TIM8_CH1IN     (GPIO_ALT|GPIO_AF3|GPIO_SPEED_50MHz|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN6)
#define GPIO_TIM8_CH2IN     (GPIO_ALT|GPIO_AF3|GPIO_SPEED_50MHz|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN7)

/* Motor pulse signals:
 *
 *  Timer12 channel 1 out is not available on Nucleo-144. Thus, channel 2 is used.
 *
 * Motor pluse: PA6, PB8, PB9, PB15, PE5, PF9
 *     Motor0(Timer9): GPIO_TIM9_CH1OUT_2 (PE5)
 *     Motor1(Timer10): GPIO_TIM10_CH1OUT_1 (PB8)
 *     Motor2(Timer11): GPIO_TIM11_CH1OUT_1 (PB9)
 *     Motor3(Timer12): GPIO_TIM12_CH2OUT_1 (PB15)
 *     Motor4(Timer13): GPIO_TIM13_CH1OUT_1 (PA6)
 *     Motor5(Timer14): GPIO_TIM14_CH1OUT_2 (PF9)
 *
 */
#define GPIO_TIM9_CH1OUT    GPIO_TIM9_CH1OUT_2
#define GPIO_TIM10_CH1OUT   GPIO_TIM10_CH1OUT_1
#define GPIO_TIM11_CH1OUT   GPIO_TIM11_CH1OUT_1
#define GPIO_TIM12_CH2OUT   GPIO_TIM12_CH2OUT_1
#define GPIO_TIM13_CH1OUT   GPIO_TIM13_CH1OUT_1
#define GPIO_TIM14_CH1OUT   GPIO_TIM14_CH1OUT_2

/* Motor control signals:
 *
 * Each motor requires 6 control signal lines plus 2 lines for status LEDs.
 *
 */
#define GPIO_MTR1_DIRECTION (GPIO_OUTPUT|GPIO_PORTC|GPIO_PIN0|GPIO_SPEED_50MHz|GPIO_FLOAT)
#define GPIO_MTR1_SVON      (GPIO_OUTPUT|GPIO_PORTF|GPIO_PIN0|GPIO_SPEED_50MHz|GPIO_FLOAT)
#define GPIO_MTR1_ALRM_RES  (GPIO_OUTPUT|GPIO_PORTB|GPIO_PIN0|GPIO_SPEED_50MHz|GPIO_FLOAT)
#define GPIO_MTR1_DEV_CLR   (GPIO_OUTPUT|GPIO_PORTF|GPIO_PIN7|GPIO_SPEED_50MHz|GPIO_FLOAT)
#define GPIO_MTR1_ALRM      (GPIO_INPUT|GPIO_PORTE|GPIO_PIN0|GPIO_SPEED_50MHz|GPIO_PULLUP)
#define GPIO_MTR1_CMPLT     (GPIO_INPUT|GPIO_PORTE|GPIO_PIN8|GPIO_SPEED_50MHz|GPIO_PULLUP)
#define GPIO_MTR1_LED_RED   (GPIO_OUTPUT|GPIO_PORTD|GPIO_PIN0|GPIO_SPEED_50MHz|GPIO_FLOAT)
#define GPIO_MTR1_LED_GRN   (GPIO_OUTPUT|GPIO_PORTD|GPIO_PIN1|GPIO_SPEED_50MHz|GPIO_FLOAT)

#define GPIO_MTR2_DIRECTION (GPIO_OUTPUT|GPIO_PORTC|GPIO_PIN2|GPIO_SPEED_50MHz|GPIO_FLOAT)
#define GPIO_MTR2_SVON      (GPIO_OUTPUT|GPIO_PORTF|GPIO_PIN1|GPIO_SPEED_50MHz|GPIO_FLOAT)
#define GPIO_MTR2_ALRM_RES  (GPIO_OUTPUT|GPIO_PORTB|GPIO_PIN1|GPIO_SPEED_50MHz|GPIO_FLOAT)
#define GPIO_MTR2_DEV_CLR   (GPIO_OUTPUT|GPIO_PORTF|GPIO_PIN8|GPIO_SPEED_50MHz|GPIO_FLOAT)
#define GPIO_MTR2_ALRM      (GPIO_INPUT|GPIO_PORTE|GPIO_PIN2|GPIO_SPEED_50MHz|GPIO_PULLUP)
#define GPIO_MTR2_CMPLT     (GPIO_INPUT|GPIO_PORTE|GPIO_PIN10|GPIO_SPEED_50MHz|GPIO_PULLUP)
#define GPIO_MTR2_LED_RED   (GPIO_OUTPUT|GPIO_PORTD|GPIO_PIN2|GPIO_SPEED_50MHz|GPIO_FLOAT)
#define GPIO_MTR2_LED_GRN   (GPIO_OUTPUT|GPIO_PORTD|GPIO_PIN3|GPIO_SPEED_50MHz|GPIO_FLOAT)

#define GPIO_MTR3_DIRECTION (GPIO_OUTPUT|GPIO_PORTC|GPIO_PIN3|GPIO_SPEED_50MHz|GPIO_FLOAT)
#define GPIO_MTR3_SVON      (GPIO_OUTPUT|GPIO_PORTF|GPIO_PIN2|GPIO_SPEED_50MHz|GPIO_FLOAT)
#define GPIO_MTR3_ALRM_RES  (GPIO_OUTPUT|GPIO_PORTB|GPIO_PIN2|GPIO_SPEED_50MHz|GPIO_FLOAT)
#define GPIO_MTR3_DEV_CLR   (GPIO_OUTPUT|GPIO_PORTF|GPIO_PIN10|GPIO_SPEED_50MHz|GPIO_FLOAT)
#define GPIO_MTR3_ALRM      (GPIO_INPUT|GPIO_PORTE|GPIO_PIN3|GPIO_SPEED_50MHz|GPIO_PULLUP)
#define GPIO_MTR3_CMPLT     (GPIO_INPUT|GPIO_PORTE|GPIO_PIN12|GPIO_SPEED_50MHz|GPIO_PULLUP)
#define GPIO_MTR3_LED_RED   (GPIO_OUTPUT|GPIO_PORTD|GPIO_PIN4|GPIO_SPEED_50MHz|GPIO_FLOAT)
#define GPIO_MTR3_LED_GRN   (GPIO_OUTPUT|GPIO_PORTD|GPIO_PIN5|GPIO_SPEED_50MHz|GPIO_FLOAT)

#define GPIO_MTR4_DIRECTION (GPIO_OUTPUT|GPIO_PORTC|GPIO_PIN8|GPIO_SPEED_50MHz|GPIO_FLOAT)
#define GPIO_MTR4_SVON      (GPIO_OUTPUT|GPIO_PORTF|GPIO_PIN3|GPIO_SPEED_50MHz|GPIO_FLOAT)
#define GPIO_MTR4_ALRM_RES  (GPIO_OUTPUT|GPIO_PORTB|GPIO_PIN6|GPIO_SPEED_50MHz|GPIO_FLOAT)
#define GPIO_MTR4_DEV_CLR   (GPIO_OUTPUT|GPIO_PORTF|GPIO_PIN12|GPIO_SPEED_50MHz|GPIO_FLOAT)
#define GPIO_MTR4_ALRM      (GPIO_INPUT|GPIO_PORTE|GPIO_PIN4|GPIO_SPEED_50MHz|GPIO_PULLUP)
#define GPIO_MTR4_CMPLT     (GPIO_INPUT|GPIO_PORTE|GPIO_PIN13|GPIO_SPEED_50MHz|GPIO_PULLUP)
#define GPIO_MTR4_LED_RED   (GPIO_OUTPUT|GPIO_PORTD|GPIO_PIN6|GPIO_SPEED_50MHz|GPIO_FLOAT)
#define GPIO_MTR4_LED_GRN   (GPIO_OUTPUT|GPIO_PORTD|GPIO_PIN7|GPIO_SPEED_50MHz|GPIO_FLOAT)

#define GPIO_MTR5_DIRECTION (GPIO_OUTPUT|GPIO_PORTC|GPIO_PIN9|GPIO_SPEED_50MHz|GPIO_FLOAT)
#define GPIO_MTR5_SVON      (GPIO_OUTPUT|GPIO_PORTF|GPIO_PIN4|GPIO_SPEED_50MHz|GPIO_FLOAT)
#define GPIO_MTR5_ALRM_RES  (GPIO_OUTPUT|GPIO_PORTB|GPIO_PIN12|GPIO_SPEED_50MHz|GPIO_FLOAT)
#define GPIO_MTR5_DEV_CLR   (GPIO_OUTPUT|GPIO_PORTF|GPIO_PIN13|GPIO_SPEED_50MHz|GPIO_FLOAT)
#define GPIO_MTR5_ALRM      (GPIO_INPUT|GPIO_PORTE|GPIO_PIN6|GPIO_SPEED_50MHz|GPIO_PULLUP)
#define GPIO_MTR5_CMPLT     (GPIO_INPUT|GPIO_PORTE|GPIO_PIN14|GPIO_SPEED_50MHz|GPIO_PULLUP)
#define GPIO_MTR5_LED_RED   (GPIO_OUTPUT|GPIO_PORTD|GPIO_PIN11|GPIO_SPEED_50MHz|GPIO_FLOAT)
#define GPIO_MTR5_LED_GRN   (GPIO_OUTPUT|GPIO_PORTD|GPIO_PIN14|GPIO_SPEED_50MHz|GPIO_FLOAT)

#define GPIO_MTR6_DIRECTION (GPIO_OUTPUT|GPIO_PORTC|GPIO_PIN10|GPIO_SPEED_50MHz|GPIO_FLOAT)
#define GPIO_MTR6_SVON      (GPIO_OUTPUT|GPIO_PORTF|GPIO_PIN5|GPIO_SPEED_50MHz|GPIO_FLOAT)
#define GPIO_MTR6_ALRM_RES  (GPIO_OUTPUT|GPIO_PORTB|GPIO_PIN13|GPIO_SPEED_50MHz|GPIO_FLOAT)
#define GPIO_MTR6_DEV_CLR   (GPIO_OUTPUT|GPIO_PORTF|GPIO_PIN14|GPIO_SPEED_50MHz|GPIO_FLOAT)
#define GPIO_MTR6_ALRM      (GPIO_INPUT|GPIO_PORTE|GPIO_PIN7|GPIO_SPEED_50MHz|GPIO_PULLUP)
#define GPIO_MTR6_CMPLT     (GPIO_INPUT|GPIO_PORTE|GPIO_PIN15|GPIO_SPEED_50MHz|GPIO_PULLUP)
#define GPIO_MTR6_LED_RED   (GPIO_OUTPUT|GPIO_PORTD|GPIO_PIN15|GPIO_SPEED_50MHz|GPIO_FLOAT)
#define GPIO_MTR6_LED_GRN   (GPIO_OUTPUT|GPIO_PORTF|GPIO_PIN15|GPIO_SPEED_50MHz|GPIO_FLOAT)

#define GPIO_EMERGENGY_STOP (GPIO_INPUT|GPIO_PORTA|GPIO_PIN3|GPIO_SPEED_50MHz|GPIO_PULLUP)

/* Home sensor signals:
 *    All sensors are active-low. The home driver automatically negates the logic 
 *    back to active-high.
 *
 */

#define GPIO_HOME_0         (GPIO_INPUT|GPIO_PORTG|GPIO_PIN0|GPIO_SPEED_50MHz|GPIO_PULLUP)
#define GPIO_HOME_1         (GPIO_INPUT|GPIO_PORTG|GPIO_PIN1|GPIO_SPEED_50MHz|GPIO_PULLUP)
#define GPIO_HOME_2         (GPIO_INPUT|GPIO_PORTG|GPIO_PIN2|GPIO_SPEED_50MHz|GPIO_PULLUP)
#define GPIO_HOME_3         (GPIO_INPUT|GPIO_PORTG|GPIO_PIN3|GPIO_SPEED_50MHz|GPIO_PULLUP)
#define GPIO_HOME_4         (GPIO_INPUT|GPIO_PORTG|GPIO_PIN9|GPIO_SPEED_50MHz|GPIO_PULLUP)
#define GPIO_HOME_5         (GPIO_INPUT|GPIO_PORTG|GPIO_PIN14|GPIO_SPEED_50MHz|GPIO_PULLUP)

#endif  /* __CONFIG_NUCLEO_144_INCLUDE_BOARD_H */

