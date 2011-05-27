#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_tim.h"


/* The on board LEDs are are on GPIOC; blue is on pin 8, green is on pin 9 */
#define LED_GPIO_PORT GPIOC
#define ENABLE_LED_GPIO_PERIPHERAL (RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE))
#define BLUE_LED (1 << 8)
#define GREEN_LED (1 << 9)
#define DEFINED_LEDS (BLUE_LED | GREEN_LED)
#define IGNORE_UNDEFINED_LEDS(leds) (leds &= DEFINED_LEDS)
typedef enum {OFF = 0, ON = 1} LED_state;

/* Configure one SPI to talk to another */
#define TRANSMITTING_SPI SPI1
#define TRANSMITTING_SPI_GPIO_PORT GPIOA
#define TRANSMITTING_SPI_MISO_PIN GPIO_Pin_6
#define TRANSMITTING_SPI_MOSI_PIN GPIO_Pin_7
#define TRANSMITTING_SPI_NSS_PIN GPIO_Pin_4
#define TRANSMITTING_SPI_SCLK_PIN GPIO_Pin_5
#define ENABLE_TRANSMITTING_SPI_GPIO_PERIPHERAL (RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE))
#define ENABLE_TRANSMITTING_SPI_PERIPHERAL (RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE))

#define RECEIVING_SPI SPI2
#define RECEIVING_SPI_GPIO_PORT GPIOB
#define RECEIVING_SPI_GPIO_PERIPHERAL RCC_APB2Periph_GPIOB
#define RECEIVING_SPI_MISO_PIN GPIO_Pin_14
#define RECEIVING_SPI_MOSI_PIN GPIO_Pin_15
#define RECEIVING_SPI_NSS_PIN GPIO_Pin_12
#define RECEIVING_SPI_SCLK_PIN GPIO_Pin_13
#define ENABLE_RECEIVING_SPI_GPIO_PERIPHERAL (RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE))
#define ENABLE_RECEIVING_SPI_PERIPHERAL (RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE));

/* a timer is used to provide a primitive sleep() function */
#define TIMER TIM6
#define ENABLE_TIMER_PERIPHERAL (RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE));

/* The state variable commicates to the main loop when to
   transmit more data over SPI. */
enum {IDLE, DATA_TO_TRANSMIT, TRANSMITTING_DATA} state = IDLE;

static void configure_LED_GPIO_pins(void);
static void configure_sleep_timer(void);
static void configure_SPI(void);
static void configure_system_ticks(void);
static void set_LED(uint16_t led, LED_state state);
static void sleep(uint16_t tenths_of_millisecond);
static void toggle_LED(uint16_t led);
static void transmit_and_receive_data(void);

int
main(int argc, char **argv) 
{
	configure_system_ticks();
	configure_sleep_timer();
	configure_LED_GPIO_pins();
	configure_SPI();

	while (true) {
		if (state == DATA_TO_TRANSMIT) {
			state = TRANSMITTING_DATA;
			transmit_and_receive_data();
			state = IDLE;
		}
	}
}

void
SysTick_Handler(void)
{
	toggle_LED(BLUE_LED);
	if (state == IDLE) {
		state = DATA_TO_TRANSMIT;
	}
}

/*
Configures the LED GPIO port.  Sets LED pins to be in push-pull output mode 
and enables the LED port peripheral.  The LEDs are initally off.
*/
static void
configure_LED_GPIO_pins(void)
{
	GPIO_InitTypeDef LED_GPIO_init = 
		{
			.GPIO_Pin = DEFINED_LEDS,
			.GPIO_Speed = GPIO_Speed_50MHz,
			.GPIO_Mode = GPIO_Mode_Out_PP
		};

	ENABLE_LED_GPIO_PERIPHERAL;
	GPIO_Init(LED_GPIO_PORT, &LED_GPIO_init);
	set_LED(DEFINED_LEDS, OFF);
}

static void
configure_sleep_timer(void)
{
	/* Enable the timer peripheral; it must be enabled before being configured */
	ENABLE_TIMER_PERIPHERAL;

	/* The timer is configured to be in one pulse mode; this means that it stops counting
	   when it reaches the preloaded value.
	*/
	TIM_SelectOnePulseMode(TIMER, TIM_OPMode_Single);

	/* The timer runs at the system clock; dividing the system clock by 10000 gives
	   .1 ms intervals. */
	TIM_PrescalerConfig(TIMER, SystemCoreClock/10000, TIM_PSCReloadMode_Update);
	
}

/*
Configures TRANSMITTING_SPI as a master device to communicate to RECEIVING_SPI as a slave device.
*/
static void
configure_SPI(void)
{
	/* Enable the GPIO peripheral; the GPIO peripheral must be enabled before it can be configured. */
	ENABLE_TRANSMITTING_SPI_GPIO_PERIPHERAL;
	ENABLE_RECEIVING_SPI_GPIO_PERIPHERAL;

	GPIO_InitTypeDef gpio_configuration;
	gpio_configuration.GPIO_Speed = GPIO_Speed_50MHz;

	/* Configure the GPIO pins for TRANSMITTING_SPI as a master device.  
	   MOSI, NSS, & SCLK are configured as alternate function push-pull outputs; 
	   MISO is a floating input. */
	gpio_configuration.GPIO_Pin = TRANSMITTING_SPI_MOSI_PIN | 
		TRANSMITTING_SPI_NSS_PIN | TRANSMITTING_SPI_SCLK_PIN;	
	gpio_configuration.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(TRANSMITTING_SPI_GPIO_PORT, &gpio_configuration);

	gpio_configuration.GPIO_Pin = TRANSMITTING_SPI_MISO_PIN;
	gpio_configuration.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(TRANSMITTING_SPI_GPIO_PORT, &gpio_configuration);

	/* Configurre the GPIO pins for RECEIVING_SPI as a slave device.
	   MOSI, NSS, & SCLK are configured as floating inputs;
	   MISO is an alternate function push-pull output. */
	gpio_configuration.GPIO_Pin = RECEIVING_SPI_MOSI_PIN |
		RECEIVING_SPI_NSS_PIN | RECEIVING_SPI_SCLK_PIN;
	gpio_configuration.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(RECEIVING_SPI_GPIO_PORT, &gpio_configuration);

	/* Enable the SPI Peripherals; SPI ports must be enabled before they can be configured. */
	ENABLE_TRANSMITTING_SPI_PERIPHERAL;
	ENABLE_RECEIVING_SPI_PERIPHERAL;

	/* Currently, both the TRANSMITTING_SPI and RECEIVING_SPI are configured
	   in 2 line full duplex mode.  The baud rate is the system clock divided by
	   256; at 24 MHz this results in a rate of 93.75 kHz. Frames are 8 bits, 
	   most significant bit first.  The clock is active high and data is sampled 
	   on the leading edge.  There is a physical pin allocated for slave select.
	   The CRC calculation is not used. */
	SPI_InitTypeDef spi_configuration;
	spi_configuration.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	spi_configuration.SPI_DataSize = SPI_DataSize_8b;
	spi_configuration.SPI_CPOL = SPI_CPOL_Low;
	spi_configuration.SPI_CPHA = SPI_CPHA_1Edge;
	spi_configuration.SPI_NSS = SPI_NSS_Hard;
	spi_configuration.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	spi_configuration.SPI_FirstBit = SPI_FirstBit_MSB;
	spi_configuration.SPI_CRCPolynomial = 0;
	
	/* Configure the TRANSMITTING_SPI as a master device. */
	spi_configuration.SPI_Mode = SPI_Mode_Master;
	SPI_Init(TRANSMITTING_SPI, &spi_configuration);

	/* Configure the RECEIVING_SPI as a slave device. */
	spi_configuration.SPI_Mode = SPI_Mode_Slave;
	SPI_Init(RECEIVING_SPI, &spi_configuration);
}


static void
configure_system_ticks(void) {
	/* configure the system tick handler to fire twice per second */
	SysTick_Config(SystemCoreClock/2);
}


/* 
Forces one (or more) LEDs to be in the specified state, either ON or OFF.
Any non-OFF LED_state is treated as ON.  Any specified LED that is not 
currently defined to be an LED is ignored.
*/
static void
set_LED(uint16_t led, LED_state state)
{
	IGNORE_UNDEFINED_LEDS(led);
	if (state == OFF) {
		GPIO_ResetBits(LED_GPIO_PORT, led);
	} else {
		GPIO_SetBits(LED_GPIO_PORT, led);
	}
}

/*
Toggles the specified LED(s) between ON & OFF.
Any specified LED that is not a defined LED is currently ignored.
*/
static void
toggle_LED(uint16_t led)
{
	IGNORE_UNDEFINED_LEDS(led);
	uint16_t current_port_state = GPIO_ReadOutputData(GPIOC);
	GPIO_ResetBits(LED_GPIO_PORT, led & current_port_state);
	GPIO_SetBits(LED_GPIO_PORT, led & ~current_port_state);
}

/* data array to transmit */
static uint8_t data[] = {0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00,
						 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02,
						 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00,
						 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02,
						 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00,
						 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02,
						 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00,
						 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02};

/* array to hold received result */
static uint8_t received_data[sizeof(data)/sizeof(*data)];

static void
transmit_and_receive_data(void)
{
	/* overwrite the received data array with 0s */
	 memset(received_data, 0, sizeof(received_data));

    SPI_SSOutputCmd(TRANSMITTING_SPI, ENABLE);
	sleep(5);

	SPI_Cmd(RECEIVING_SPI, ENABLE);
	SPI_Cmd(TRANSMITTING_SPI, ENABLE);

	for (unsigned int index = 0; index < sizeof(data)/sizeof(*data); index++) {
		while (!SPI_I2S_GetFlagStatus(TRANSMITTING_SPI, SPI_I2S_FLAG_TXE)) { }
		SPI_I2S_SendData(TRANSMITTING_SPI, data[index]);
		while (!SPI_I2S_GetFlagStatus(RECEIVING_SPI, SPI_I2S_FLAG_RXNE)) { }
		received_data[index] = SPI_I2S_ReceiveData(RECEIVING_SPI);
	}

	while (SPI_I2S_GetFlagStatus(TRANSMITTING_SPI, SPI_I2S_FLAG_BSY)) { }
	while (SPI_I2S_GetFlagStatus(RECEIVING_SPI, SPI_I2S_FLAG_BSY)) { }

	SPI_Cmd(TRANSMITTING_SPI, DISABLE);
	SPI_Cmd(RECEIVING_SPI, DISABLE);

	sleep(5);
	SPI_SSOutputCmd(TRANSMITTING_SPI, DISABLE);
		
	if (memcmp(data, received_data, sizeof(data)) == 0) {
		set_LED(GREEN_LED, OFF);
	} else {
		set_LED(GREEN_LED, ON);
	}
}

/*
Busy waits on a timer for the specified period of time.
*/
static void
sleep(uint16_t tenths_of_millisecond)
{
	/* The timer is configured to have a clock that ticks in .1 ms intervals.
	   The auto reload counter is loaded with the number of ticks desired, 
	   the clock is enabled, and then a loop busy waits on the pending interrupt
	   flag until the specified time has elapsed. */
	TIM_SetAutoreload(TIMER, tenths_of_millisecond);
	TIM_Cmd(TIMER, ENABLE);
	while (!TIM_GetFlagStatus(TIMER, TIM_FLAG_Update)) { }
}
