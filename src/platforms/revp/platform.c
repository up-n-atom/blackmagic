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

/* This file implements the platform specific functions for the native implementation. */

#include "general.h"
#include "usb.h"
#include "aux_serial.h"
#include "morse.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/scs.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/flash.h>

static void adc_init(void);
static void setup_vbus_irq(void);

/* This is defined by the linker script */
extern char vector_table;

#define REVP_HWVERSION_BYTE FLASH_OPTION_BYTE_2

int platform_hwversion(void)
{
	static int hwversion = -1;

	if (hwversion == -1) {
		if (REVP_HWVERSION_BYTE != 0xffffU && REVP_HWVERSION_BYTE != 0x00ffU) {
			/* Check if the data is valid. When valid it should only have values 4 and higher. */
			if ((REVP_HWVERSION_BYTE >> 8U) != (~REVP_HWVERSION_BYTE & 0xffU) || (REVP_HWVERSION_BYTE & 0xffU) < 4)
				return -2;
			hwversion = REVP_HWVERSION_BYTE & 0xffU;
		}
	}

	return hwversion;
}

void platform_init(void)
{
	SCS_DEMCR |= SCS_DEMCR_VC_MON_EN;

	rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

	/* Enable peripherals */
	rcc_periph_clock_enable(RCC_USB);
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_AFIO);
	rcc_periph_clock_enable(RCC_CRC);

	/* Setup GPIO ports */
	gpio_clear(USB_PU_PORT, USB_PU_PIN);
	gpio_set_mode(USB_PU_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, USB_PU_PIN);

	gpio_set_mode(JTAG_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, TMS_DIR_PIN | TCK_PIN | TDI_PIN);
	gpio_set_mode(JTAG_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_INPUT_FLOAT, TMS_PIN);

	/* This needs some fixing... */
	/* Toggle required to sort out line drivers... */
	gpio_port_write(GPIOA, 0x8102);
	gpio_port_write(GPIOB, 0x2000);

	gpio_port_write(GPIOA, 0x8152);

	gpio_set_mode(LED_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, LED_UART | LED_IDLE_RUN | LED_ERROR);

	/*
	 * Enable nRST output.
	 */
	platform_nrst_set_val(false);
	gpio_set_mode(NRST_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, NRST_PIN);
	gpio_set(NRST_SENSE_PORT, NRST_SENSE_PIN);
	gpio_set_mode(NRST_SENSE_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, NRST_SENSE_PIN);

	/*
	 * Enable internal pull-up on PWR_BR so that we don't drive
	 * TPWR locally or inadvertently supply power to the target.
	 */
	gpio_set(PWR_BR_PORT, PWR_BR_PIN);
	gpio_set_mode(PWR_BR_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_OPENDRAIN, PWR_BR_PIN);

	adc_init();

	/* Set up the NVIC vector table for the firmware */
	SCB_VTOR = (uint32_t)&vector_table; // NOLINT(clang-diagnostic-pointer-to-int-cast)

	platform_timing_init();
	blackmagic_usb_init();

	/*
	 * Don't enable UART if we're being debugged.
	 */
	if (!(SCS_DEMCR & SCS_DEMCR_TRCENA))
		aux_serial_init();

	setup_vbus_irq();
}

void platform_nrst_set_val(bool assert)
{
	gpio_set(TMS_PORT, TMS_PIN);
	gpio_set_val(NRST_PORT, NRST_PIN, assert);

	if (assert) {
		for (volatile size_t i = 0; i < 10000U; ++i)
			continue;
	}
}

bool platform_nrst_get_val(void)
{
	return gpio_get(NRST_SENSE_PORT, NRST_SENSE_PIN) != 0;
}

bool platform_target_get_power(void)
{
	return !gpio_get(PWR_BR_PORT, PWR_BR_PIN);
}

void platform_target_set_power(const bool power)
{
	gpio_set_val(PWR_BR_PORT, PWR_BR_PIN, !power);
}

static void adc_init(void)
{
	rcc_periph_clock_enable(RCC_ADC1);

	gpio_set_mode(TPWR_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, TPWR_PIN);

	adc_power_off(ADC1);
	adc_disable_scan_mode(ADC1);
	adc_set_single_conversion_mode(ADC1);
	adc_disable_external_trigger_regular(ADC1);
	adc_set_right_aligned(ADC1);
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_239DOT5CYC);

	adc_power_on(ADC1);

	/* Wait for the ADC to finish starting up */
	for (volatile size_t i = 0; i < 800000U; ++i)
		continue;

	adc_reset_calibration(ADC1);
	adc_calibrate(ADC1);
}

uint32_t platform_target_voltage_sense(void)
{
	/*
	 * Returns the voltage in tenths of a volt (so 33 means 3.3V)
	 */
	uint8_t channel = 8;
	adc_set_regular_sequence(ADC1, 1, &channel);

	adc_start_conversion_direct(ADC1);

	/* Wait for end of conversion. */
	while (!adc_eoc(ADC1))
		continue;

	uint32_t val = adc_read_regular(ADC1); /* 0-4095 */
	/* Clear EOC bit. The GD32F103 does not automatically reset it on ADC read. */
	ADC_SR(ADC1) &= ~ADC_SR_EOC;

	return (val * 99U) / 8191U;
}

const char *platform_target_voltage(void)
{
	static char ret[] = "0.0V";
	uint32_t val = platform_target_voltage_sense();
	ret[0] = '0' + val / 10U;
	ret[2] = '0' + val % 10U;

	return ret;
}

void platform_request_boot(void)
{
	/* Disconnect USB cable */
	gpio_set_mode(USB_PU_PORT, GPIO_MODE_INPUT, 0, USB_PU_PIN);

	/* Drive boot request pin */
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);
	gpio_clear(GPIOB, GPIO12);
}

void platform_target_clk_output_enable(bool enable)
{
	(void)enable;
}

void exti15_10_isr(void)
{
	if (gpio_get(USB_VBUS_PORT, USB_VBUS_PIN))
		/* Drive pull-up high if VBUS connected */
		gpio_set_mode(USB_PU_PORT, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, USB_PU_PIN);
	else
		/* Allow pull-up to float if VBUS disconnected */
		gpio_set_mode(USB_PU_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, USB_PU_PIN);

	exti_reset_request(USB_VBUS_PIN);
}

static void setup_vbus_irq(void)
{
	nvic_set_priority(USB_VBUS_IRQ, IRQ_PRI_USB_VBUS);
	nvic_enable_irq(USB_VBUS_IRQ);

	gpio_set(USB_VBUS_PORT, USB_VBUS_PIN);
	gpio_set(USB_PU_PORT, USB_PU_PIN);

	gpio_set_mode(USB_VBUS_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, USB_VBUS_PIN);

	/* Configure EXTI for USB VBUS monitor */
	exti_select_source(USB_VBUS_PIN, USB_VBUS_PORT);
	exti_set_trigger(USB_VBUS_PIN, EXTI_TRIGGER_BOTH);
	exti_enable_request(USB_VBUS_PIN);

	exti15_10_isr();
}
