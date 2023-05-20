#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "spi_flash_mmap.h"
#include "esp_efuse.h"
#include "esp_efuse_table.h" // or "esp_efuse_custom_table.h"
#include "tinyusb.h"
#include "tusb_hid_gamepad.h"
#include "tusb_cdc_acm.h"
#include "advanced_usb_control.h"
#include "esp_task_wdt.h"
#include "esp_log.h"
#include "soc/dedic_gpio_reg.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "soc/soc.h"
#include "soc/system_reg.h"
#include "soc/usb_reg.h"
#include "ulp_riscv.h"
#include "driver/rtc_io.h"
#include "driver/gpio.h"
#include "rom/gpio.h"
#include "soc/rtc.h"

#define SOC_DPORT_USB_BASE 0x60080000

struct SandboxStruct * g_SandboxStruct;

#define RX_BUF_SIZE 1024
#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_18)
#define UART UART_NUM_1 
static uint8_t buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1]; 

uint16_t tud_hid_get_report_cb(uint8_t itf,
							   uint8_t report_id,
							   hid_report_type_t report_type,
							   uint8_t* buffer,
							   uint16_t reqlen)
{
	if( report_id == 170 || report_id == 171 )
	{
		return handle_advanced_usb_control_get( reqlen, buffer );
	}
	else if( report_id == 172 )
	{
		return handle_advanced_usb_terminal_get( reqlen, buffer );
	}
	else if( report_id == 173 && g_SandboxStruct && g_SandboxStruct->fnAdvancedUSB )
	{
		return g_SandboxStruct->fnAdvancedUSB( buffer, reqlen, 1 );
	}
	else
	{
		return reqlen;
	}
}

void tud_hid_set_report_cb(uint8_t itf,
						   uint8_t report_id,
						   hid_report_type_t report_type,
						   uint8_t const* buffer,
						   uint16_t bufsize )
{
	if( report_id >= 170 && report_id <= 171 )
	{
		handle_advanced_usb_control_set( bufsize, buffer );
	}
	else if( report_id == 173 && g_SandboxStruct && g_SandboxStruct->fnAdvancedUSB )
	{
		g_SandboxStruct->fnAdvancedUSB( (uint8_t*)buffer, bufsize, 0 );
	}
}


void tinyusb_cdc_rx_callback(int itf, cdcacm_event_t *event)
{ 
    size_t rx_size = 0; 

    if (tinyusb_cdcacm_read(itf, buf, CONFIG_TINYUSB_CDC_RX_BUFSIZE, &rx_size) == ESP_OK) {
		uart_write_bytes(UART, buf, rx_size);
    } 
} 

void esp_sleep_enable_timer_wakeup();

volatile void * keep_symbols[] = { 0, uprintf, vTaskDelay, ulp_riscv_halt,
	ulp_riscv_timer_resume, ulp_riscv_timer_stop, ulp_riscv_load_binary,
	ulp_riscv_run, ulp_riscv_config_and_run, esp_sleep_enable_timer_wakeup,
	ulp_set_wakeup_period, rtc_gpio_init, rtc_gpio_set_direction,
	rtc_gpio_set_level, gpio_config, gpio_matrix_out, gpio_matrix_in,
	rtc_clk_cpu_freq_get_config, rtc_clk_cpu_freq_set_config_fast,
	rtc_clk_apb_freq_get };

extern struct SandboxStruct sandbox_mode;

static void rx_task(void *arg)
{ 
    uint8_t buf[1024];
	const char hello[] = "Hello world from cdc\r\n"; 
	bool status = true;

    while (1) {
		if(tud_cdc_n_connected(TINYUSB_CDC_ACM_0)) { 
			if(status) {
				tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, (uint8_t*)hello, strlen(hello));
				tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, 0); 
				status = false;
			}

			const int rx_size = uart_read_bytes(UART, buf, RX_BUF_SIZE, 100 / portTICK_RATE_MS);
			uint8_t *ptr = buf;
			if (rx_size > 0) {  
				for(int i = 0; i < rx_size; i += CONFIG_TINYUSB_CDC_RX_BUFSIZE) {
					int remainer = rx_size - i >= CONFIG_TINYUSB_CDC_RX_BUFSIZE ? CONFIG_TINYUSB_CDC_RX_BUFSIZE : rx_size - i;
					ptr = buf + i;
					tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, ptr, remainer);
					tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, 0);
				} 
			}
		} else {
			status = true;
			vTaskDelay(100);
		}
    } 
}

void app_main(void)
{
	printf("Hello world! Keep table at %p\n", &keep_symbols );

	const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

	uart_driver_install(UART, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART, &uart_config);
    uart_set_pin(UART, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

	g_SandboxStruct = &sandbox_mode;

	esp_log_set_vprintf( advanced_usb_write_log_printf );

	// esp_efuse_set_rom_log_scheme(ESP_EFUSE_ROM_LOG_ALWAYS_OFF);
	esp_efuse_set_rom_log_scheme(ESP_EFUSE_ROM_LOG_ALWAYS_ON);

    tinyusb_config_t tusb_cfg = {};
    tinyusb_driver_install(&tusb_cfg);

 
	printf("Minimum free heap size: %d bytes\n", (int)esp_get_minimum_free_heap_size());

	void sandbox_main();

	sandbox_main();
 
	const tinyusb_config_cdcacm_t acm_cfg = {
		.usb_dev = TINYUSB_USBDEV_0,
		.cdc_port = TINYUSB_CDC_ACM_0,
		.rx_unread_buf_sz = 64,
		.callback_rx = &tinyusb_cdc_rx_callback,
		.callback_rx_wanted_char = NULL,
		.callback_line_state_changed = NULL,
		.callback_line_coding_changed = NULL
	};
	tusb_cdc_acm_init(&acm_cfg);  
	xTaskCreate(rx_task, "uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL); 

	do
	{
		if( g_SandboxStruct && g_SandboxStruct->fnIdle ) { g_SandboxStruct->fnIdle(); }
		esp_task_wdt_reset();
		taskYIELD();
	} while( 1 );

//	printf("Restarting now.\n");
//	fflush(stdout);
//	esp_restart();
}
