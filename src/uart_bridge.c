// SPDX-License-Identifier: MIT
/*
 * Copyright 2021 Álvaro Fernández Rojas <noltari@gmail.com>
 */

#include "class/cdc/cdc.h"
#include <hardware/irq.h>
#include <hardware/pio.h>
#include <hardware/structs/sio.h>
#include <hardware/uart.h>
#include <pico/multicore.h>
#include <pico/stdlib.h>
#include <string.h>
#include <tusb.h>

#include "cli/cli.h"
#include "cli/alpha_power_modes.h"

#include "uart_rx.pio.h"
#include "uart_tx.pio.h"

void tud_task(void);

#if !defined(MIN)
#define MIN(a, b) ((a > b) ? b : a)
#endif /* MIN */

#define LED_PIN 25

#define BUFFER_SIZE 64

#define DEF_BIT_RATE 115200
#define DEF_STOP_BITS 1
#define DEF_PARITY 0
#define DEF_DATA_BITS 8

typedef struct {
	union {
		uart_inst_t *const inst;
		struct {
			PIO pio;
			int rx_sm;
			int tx_sm;
		};
	};
	uint8_t tx_pin;
	uint8_t rx_pin;
} uart_id_t;

typedef struct {
	cdc_line_coding_t usb_lc;
	cdc_line_coding_t uart_lc;
	mutex_t lc_mtx;
	uint8_t uart_buffer[BUFFER_SIZE];
	uint32_t uart_pos;
	mutex_t uart_mtx;
	uint8_t usb_buffer[BUFFER_SIZE];
	uint32_t usb_pos;
	mutex_t usb_mtx;
} uart_data_t;

#define NUM_USB_DEVICES_SUPPORTED CFG_TUD_CDC
#define NUM_UARTS_SUPPORTED ((NUM_USB_DEVICES_SUPPORTED) - 1)

#define NUM_HW_UARTS_SUPPORTED 2
#define NUM_PIO_UARTS_SUPPORTED ((NUM_UARTS_SUPPORTED) - (NUM_HW_UARTS_SUPPORTED))
#if (NUM_PIO_UARTS_SUPPORTED < 0)
	#error "More HW Uarts configured as supported than USB CDC devices exposed"
#endif

#define START_ITF_HW_UART   0
#define END_ITF_HW_UART     ((START_ITF_HW_UART) + (NUM_HW_UARTS_SUPPORTED))
#define START_ITF_PIO_UART  END_ITF_HW_UART
#define END_ITF_PIO_UART    ((START_ITF_PIO_UART) + (NUM_PIO_UARTS_SUPPORTED))
#define START_ITF_CLI       END_ITF_PIO_UART
#define END_ITF_CLI         START_ITF_CLI + 1


const uart_id_t UART_ID[NUM_USB_DEVICES_SUPPORTED] = {
	{
		.inst = uart0,
		.tx_pin = 0,
		.rx_pin = 1,
	}, {
		.inst = uart1,
		.tx_pin = 4,
		.rx_pin = 5,
	}, {
		.pio = pio0,
		.rx_sm = 0,
		.tx_sm = 1,
		.tx_pin = 8,
		.rx_pin = 9,
	}, {
		.inst = NULL,				// CLI data: deliberate bad values to make sure it isn't accidentally used as uart/pio
		.tx_pin = 255,				// CLI data: deliberate bad values to make sure it isn't accidentally used as uart/pio
		.rx_pin = 255,				// CLI data: deliberate bad values to make sure it isn't accidentally used as uart/pio
	}
};

uart_data_t UART_DATA[NUM_USB_DEVICES_SUPPORTED];

static inline uint databits_usb2uart(uint8_t data_bits)
{
	switch (data_bits) {
		case 5:
			return 5;
		case 6:
			return 6;
		case 7:
			return 7;
		default:
			return 8;
	}
}

static inline uart_parity_t parity_usb2uart(uint8_t usb_parity)
{
	switch (usb_parity) {
		case 1:
			return UART_PARITY_ODD;
		case 2:
			return UART_PARITY_EVEN;
		default:
			return UART_PARITY_NONE;
	}
}

static inline uint stopbits_usb2uart(uint8_t stop_bits)
{
	switch (stop_bits) {
		case 2:
			return 2;
		default:
			return 1;
	}
}

void update_uart_cfg(uint8_t itf)
{
	const uart_id_t *ui = &UART_ID[itf];
	uart_data_t *ud = &UART_DATA[itf];

	mutex_enter_blocking(&ud->lc_mtx);

	if (ud->usb_lc.bit_rate != ud->uart_lc.bit_rate) {
		uart_set_baudrate(ui->inst, ud->usb_lc.bit_rate);
		ud->uart_lc.bit_rate = ud->usb_lc.bit_rate;
	}

	if ((ud->usb_lc.stop_bits != ud->uart_lc.stop_bits) ||
	    (ud->usb_lc.parity != ud->uart_lc.parity) ||
	    (ud->usb_lc.data_bits != ud->uart_lc.data_bits)) {
		uart_set_format(ui->inst,
				databits_usb2uart(ud->usb_lc.data_bits),
				stopbits_usb2uart(ud->usb_lc.stop_bits),
				parity_usb2uart(ud->usb_lc.parity));
		ud->uart_lc.data_bits = ud->usb_lc.data_bits;
		ud->uart_lc.parity = ud->usb_lc.parity;
		ud->uart_lc.stop_bits = ud->usb_lc.stop_bits;
	}

	mutex_exit(&ud->lc_mtx);
}

void usb_read_bytes(uint8_t itf) {
	uint32_t len = tud_cdc_n_available(itf);

	if (len) {
		uart_data_t *ud = &UART_DATA[itf];

		mutex_enter_blocking(&ud->usb_mtx);

		len = MIN(len, BUFFER_SIZE - ud->usb_pos);
		if (len) {
			uint32_t count;

			count = tud_cdc_n_read(itf, ud->usb_buffer, len);
			ud->usb_pos += count;
		}

		mutex_exit(&ud->usb_mtx);
	}
}

void usb_write_bytes(uint8_t itf) {
	uart_data_t *ud = &UART_DATA[itf];

	if (ud->uart_pos) {
		uint32_t count;

		mutex_enter_blocking(&ud->uart_mtx);

		count = tud_cdc_n_write(itf, ud->uart_buffer, ud->uart_pos);
		if (count < ud->uart_pos)
			memcpy(ud->uart_buffer, &ud->uart_buffer[count],
			       ud->uart_pos - count);
		ud->uart_pos -= count;

		mutex_exit(&ud->uart_mtx);

		if (count)
			tud_cdc_n_write_flush(itf);
	}
}

void usb_cdc_process(uint8_t itf)
{
	uart_data_t *ud = &UART_DATA[itf];

	mutex_enter_blocking(&ud->lc_mtx);
	tud_cdc_n_get_line_coding(itf, &ud->usb_lc);
	mutex_exit(&ud->lc_mtx);

	usb_read_bytes(itf);
	usb_write_bytes(itf);
}

void core1_entry(void)
{
	tusb_init();

	while (1) {
		int itf;
		int con = 0;

		tud_task();

		for (itf = 0; itf < NUM_USB_DEVICES_SUPPORTED; itf++) {
			if (tud_cdc_n_connected(itf)) {
				con = 1;
				usb_cdc_process(itf);
			}
		}

		gpio_put(LED_PIN, con);
	}
}

void uart_read_bytes(uint8_t itf) {
	const uart_id_t *ui = &UART_ID[itf];

	if (uart_is_readable(ui->inst)) {
		uart_data_t *ud = &UART_DATA[itf];

		mutex_enter_blocking(&ud->uart_mtx);

		while (uart_is_readable(ui->inst) &&
			ud->uart_pos < BUFFER_SIZE) {
			ud->uart_buffer[ud->uart_pos] = uart_getc(ui->inst);
			ud->uart_pos++;
		}

		mutex_exit(&ud->uart_mtx);
	}
}

void uart_write_bytes(uint8_t itf) {
	uart_data_t *ud = &UART_DATA[itf];

	if (ud->usb_pos) {
		const uart_id_t *ui = &UART_ID[itf];

		mutex_enter_blocking(&ud->usb_mtx);

		uart_write_blocking(ui->inst, ud->usb_buffer, ud->usb_pos);
		ud->usb_pos = 0;

		mutex_exit(&ud->usb_mtx);
	}
}


void pio_uart_read_bytes(uint8_t itf) {
	const uart_id_t *ui = &UART_ID[itf];
	uart_data_t *ud = &UART_DATA[itf];

	if (uart_rx_program_hasdata(ui->pio, ui->rx_sm)) { 
		mutex_enter_blocking(&ud->uart_mtx);
		while (uart_rx_program_hasdata(ui->pio, ui->rx_sm) && (ud->uart_pos < BUFFER_SIZE)) {
			ud->uart_buffer[ud->uart_pos] = uart_rx_program_getc(ui->pio, ui->rx_sm);
			ud->uart_pos++;
		}
		mutex_exit(&ud->uart_mtx);
	}
}


void pio_uart_write_bytes(uint8_t itf) {
	uart_data_t *ud = &UART_DATA[itf];

	if (ud->usb_pos) {
		const uart_id_t *ui = &UART_ID[itf];

		mutex_enter_blocking(&ud->usb_mtx);

		ud->usb_buffer[ud->usb_pos] = 0;
		uart_tx_program_puts(ui->pio, ui->tx_sm, ud->usb_buffer);
		ud->usb_pos = 0;

		mutex_exit(&ud->usb_mtx);
	}
}


void run_cli() {
	int itf = START_ITF_CLI;

	uart_data_t *ud = &UART_DATA[itf];

	if (ud->usb_pos) {
		mutex_enter_blocking(&ud->usb_mtx);
		cli_putn((const char *)ud->usb_buffer, ud->usb_pos);
		ud->usb_pos = 0;
		mutex_exit(&ud->usb_mtx);

		cli_update();
	}
}


void init_uart_data(uint8_t itf) {
	const uart_id_t *ui = &UART_ID[itf];
	uart_data_t *ud = &UART_DATA[itf];

	/* Pinmux */
	gpio_set_function(ui->tx_pin, GPIO_FUNC_UART);
	gpio_set_function(ui->rx_pin, GPIO_FUNC_UART);

	/* UART start */
	uart_init(ui->inst, ud->usb_lc.bit_rate);
	uart_set_hw_flow(ui->inst, false, false);
	uart_set_format(ui->inst, databits_usb2uart(ud->usb_lc.data_bits),
			stopbits_usb2uart(ud->usb_lc.stop_bits),
			parity_usb2uart(ud->usb_lc.parity));
}


void init_pio_data(uint8_t itf) 
{
	const uart_id_t *ui = &UART_ID[itf];

    uint offset = pio_add_program(ui->pio, &uart_rx_program);
    uart_rx_program_init(ui->pio, ui->rx_sm, offset, ui->rx_pin, 115200);
    
	offset = pio_add_program(ui->pio, &uart_tx_program);
    uart_tx_program_init(ui->pio, ui->tx_sm, offset, ui->tx_pin, 115200);
}


void init_common_data(uint8_t itf)
{
	uart_data_t *ud = &UART_DATA[itf];

	/* USB CDC LC */
	ud->usb_lc.bit_rate = DEF_BIT_RATE;
	ud->usb_lc.data_bits = DEF_DATA_BITS;
	ud->usb_lc.parity = DEF_PARITY;
	ud->usb_lc.stop_bits = DEF_STOP_BITS;

	/* UART LC */
	ud->uart_lc.bit_rate = DEF_BIT_RATE;
	ud->uart_lc.data_bits = DEF_DATA_BITS;
	ud->uart_lc.parity = DEF_PARITY;
	ud->uart_lc.stop_bits = DEF_STOP_BITS;

	/* Buffer */
	ud->uart_pos = 0;
	ud->usb_pos = 0;

	/* Mutex */
	mutex_init(&ud->lc_mtx);
	mutex_init(&ud->uart_mtx);
	mutex_init(&ud->usb_mtx);
}


void send_char(char c)
{
	int itf = START_ITF_CLI;
	
	uart_data_t *ud = &UART_DATA[itf];

	mutex_enter_blocking(&ud->uart_mtx);
	if (ud->uart_pos < BUFFER_SIZE) {
		ud->uart_buffer[ud->uart_pos] = c;
		ud->uart_pos++;
	}
	mutex_exit(&ud->uart_mtx);
}


int main(void)
{
	apm_init(2, 3);
	for (int itf = 0; itf < NUM_USB_DEVICES_SUPPORTED; itf++) {
		init_common_data(itf);
	}

	for (int itf = 0; itf < NUM_HW_UARTS_SUPPORTED; itf++) {
		init_uart_data(itf);
	}

	for (int itf = NUM_HW_UARTS_SUPPORTED; itf < NUM_USB_DEVICES_SUPPORTED; itf++) {
		init_pio_data(itf);
	}

	cli_init(send_char);

	gpio_init(LED_PIN);
	gpio_set_dir(LED_PIN, GPIO_OUT);

	multicore_launch_core1(core1_entry);

	while (1) {
		for (int itf = START_ITF_HW_UART; itf < END_ITF_HW_UART; itf++) {
			update_uart_cfg(itf);
			uart_read_bytes(itf);
			uart_write_bytes(itf);
		}

		for (int itf = START_ITF_PIO_UART; itf < END_ITF_PIO_UART; itf++) {
			pio_uart_read_bytes(itf);
			pio_uart_write_bytes(itf);
		}

		run_cli();

	}

	return 0;
}
