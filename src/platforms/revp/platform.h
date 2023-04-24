/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* This file provides the platform specific declarations for the native implementation. */

#ifndef PLATFORMS_REVP_PLATFORM_H
#define PLATFORMS_REVP_PLATFORM_H

#include "gpio.h"
#include "timing.h"
#include "timing_stm32.h"

#define PLATFORM_HAS_TRACESWO
#define NUM_TRACE_PACKETS 128U /* This is an 8K buffer */
#define TRACESWO_PROTOCOL 2U   /* 1 = Manchester, 2 = NRZ / async */
#define PLATFORM_HAS_POWER_SWITCH

#ifdef ENABLE_DEBUG
#define PLATFORM_HAS_DEBUG
extern bool debug_bmp;
#endif

#define PLATFORM_IDENT   ""
#define UPD_IFACE_STRING "@Internal Flash   /0x08000000/8*001Kg"

/* Hardware definitions... */
#define JTAG_PORT    GPIOA
#define TDI_PORT     JTAG_PORT
#define TMS_DIR_PORT JTAG_PORT
#define TMS_PORT     JTAG_PORT
#define TCK_PORT     JTAG_PORT
#define TDO_PORT     JTAG_PORT
#define TDI_PIN      GPIO9
#define TMS_DIR_PIN  GPIO1
#define TMS_PIN      GPIO2
#define TCK_PIN      GPIO5
#define TDO_PIN      GPIO10

#define SWDIO_DIR_PORT JTAG_PORT
#define SWDIO_PORT     JTAG_PORT
#define SWCLK_PORT     JTAG_PORT
#define SWDIO_DIR_PIN  TMS_DIR_PIN
#define SWDIO_PIN      TMS_PIN
#define SWCLK_PIN      TCK_PIN

#define TRST_PORT       JTAG_PORT
#define NRST_PORT       JTAG_PORT
#define NRST_SENSE_PORT JTAG_PORT
#define TRST_PIN        GPIO6
#define NRST_PIN        GPIO3
#define NRST_SENSE_PIN  GPIO4

/*
 * These are the control output pin definitions for TPWR.
 * TPWR is sensed via PB0 by sampling ADC1's channel 8.
 */
#define PWR_BR_PORT TRST_PORT
#define PWR_BR_PIN  TRST_PIN
#define TPWR_PORT   GPIOB
#define TPWR_PIN    GPIO0

#define USB_PU_PORT GPIOA
#define USB_PU_PIN  GPIO8

/* For HW Rev 4 and older */
#define USB_VBUS_PORT GPIOB
#define USB_VBUS_PIN  GPIO13
/* IRQ stays the same for all hw revisions. */
#define USB_VBUS_IRQ NVIC_EXTI15_10_IRQ

#define LED_PORT      GPIOB
#define LED_PORT_UART GPIOB
#define LED_0         GPIO1
#define LED_1         GPIO2
#define LED_2         GPIO14
#define LED_UART      LED_0
#define LED_IDLE_RUN  LED_1
#define LED_ERROR     LED_2

#define SWD_CR       GPIO_CRL(SWDIO_PORT)
#define SWD_CR_SHIFT (2U << 2U)

#define TMS_SET_MODE()                                                                       \
	do {                                                                                     \
		gpio_set(TMS_DIR_PORT, TMS_DIR_PIN);                                                 \
		gpio_set_mode(TMS_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, TMS_PIN); \
	} while (0)

#define SWDIO_MODE_FLOAT()                        \
	do {                                          \
		uint32_t cr = SWD_CR;                     \
		cr &= ~(0xfU << SWD_CR_SHIFT);            \
		cr |= (0x4U << SWD_CR_SHIFT);             \
		GPIO_BRR(SWDIO_DIR_PORT) = SWDIO_DIR_PIN; \
		SWD_CR = cr;                              \
	} while (0)

#define SWDIO_MODE_DRIVE()                         \
	do {                                           \
		uint32_t cr = SWD_CR;                      \
		cr &= ~(0xfU << SWD_CR_SHIFT);             \
		cr |= (0x1U << SWD_CR_SHIFT);              \
		GPIO_BSRR(SWDIO_DIR_PORT) = SWDIO_DIR_PIN; \
		SWD_CR = cr;                               \
	} while (0)

#define UART_PIN_SETUP()                                                                                        \
	do {                                                                                                        \
		gpio_set_mode(USBUSART_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, USBUSART_TX_PIN); \
		gpio_set_mode(USBUSART_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, USBUSART_RX_PIN);             \
		gpio_set(USBUSART_PORT, USBUSART_RX_PIN);                                                               \
	} while (0)

#define USB_DRIVER st_usbfs_v1_usb_driver
#define USB_IRQ    NVIC_USB_LP_CAN_RX0_IRQ
#define USB_ISR(x) usb_lp_can_rx0_isr(x)
/*
 * Interrupt priorities. Low numbers are high priority.
 * TIM3 is used for traceswo capture and must be highest priority.
 */
#define IRQ_PRI_USB          (1U << 4U)
#define IRQ_PRI_USBUSART     (2U << 4U)
#define IRQ_PRI_USBUSART_DMA (2U << 4U)
#define IRQ_PRI_USB_VBUS     (14U << 4U)
#define IRQ_PRI_TRACE        (0U << 4U)
#define IRQ_PRI_SWO_DMA      (0U << 4U)

#define USBUSART        USBUSART3
#define USBUSART_IRQ    NVIC_USART3_IRQ
#define USBUSART_CLK    RCC_USART3
#define USBUSART_PORT   GPIOB
#define USBUSART_TX_PIN GPIO10
#define USBUSART_RX_PIN GPIO11

#define USBUSART_DMA_BUS       DMA1
#define USBUSART_DMA_CLK       RCC_DMA1
#define USBUSART_DMA_TX_CHAN   USBUSART3_DMA_TX_CHAN
#define USBUSART_DMA_RX_CHAN   USBUSART3_DMA_RX_CHAN
#define USBUSART_DMA_TX_IRQ    USBUSART3_DMA_TX_IRQ
#define USBUSART_DMA_RX_IRQ    USBUSART3_DMA_RX_IRQ
#define USBUSART_ISR(x)        USBUSART3_ISR(x)
#define USBUSART_DMA_TX_ISR(x) USBUSART3_DMA_TX_ISR(x)
#define USBUSART_DMA_RX_ISR(x) USBUSART3_DMA_RX_ISR(x)

#define USBUSART3               USART3
#define USBUSART3_IRQ           NVIC_USART3_IRQ
#define USBUSART3_ISR(x)        usart3_isr(x)
#define USBUSART3_DMA_TX_CHAN   DMA_CHANNEL2
#define USBUSART3_DMA_TX_IRQ    NVIC_DMA1_CHANNEL2_IRQ
#define USBUSART3_DMA_TX_ISR(x) dma1_channel2_isr(x)
#define USBUSART3_DMA_RX_CHAN   DMA_CHANNEL3
#define USBUSART3_DMA_RX_IRQ    NVIC_DMA1_CHANNEL3_IRQ
#define USBUSART3_DMA_RX_ISR(x) dma1_channel3_isr(x)

#define TRACE_TIM          TIM3
#define TRACE_TIM_CLK_EN() rcc_periph_clock_enable(RCC_TIM3)
#define TRACE_IRQ          NVIC_TIM3_IRQ
#define TRACE_ISR(x)       tim3_isr(x)

/* On F103, only USART1 is on AHB2 and can reach 4.5MBaud at 72 MHz. */
#define SWO_UART        USART1
#define SWO_UART_DR     USART1_DR
#define SWO_UART_CLK    RCC_USART1
#define SWO_UART_PORT   JTAG_PORT
#define SWO_UART_RX_PIN TDO_PIN

/* This DMA channel is set by the USART in use */
#define SWO_DMA_BUS    DMA1
#define SWO_DMA_CLK    RCC_DMA1
#define SWO_DMA_CHAN   DMA_CHANNEL5
#define SWO_DMA_IRQ    NVIC_DMA1_CHANNEL5_IRQ
#define SWO_DMA_ISR(x) dma1_channel5_isr(x)

#define SET_RUN_STATE(state)   running_status = (state)
#define SET_IDLE_STATE(state)  gpio_set_val(LED_PORT, LED_IDLE_RUN, state)
#define SET_ERROR_STATE(state) gpio_set_val(LED_PORT, LED_ERROR, state)

/* Use newlib provided integer-only stdio functions */

#ifdef sscanf
#undef sscanf
#endif
#define sscanf siscanf

#ifdef sprintf
#undef sprintf
#endif
#define sprintf siprintf

#ifdef vasprintf
#undef vasprintf
#endif
#define vasprintf vasiprintf

#ifdef snprintf
#undef snprintf
#endif
#define snprintf sniprintf

#endif /* PLATFORMS_REVP_PLATFORM_H */
