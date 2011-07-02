#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_adc.h"

/* The on board LEDs are are on GPIOC; blue is on pin 8, green is on pin 9 */
#define LED_GPIO_PORT GPIOC
#define ENABLE_LED_GPIO_PERIPHERAL (RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE))
#define BLUE_LED (1 << 8)
#define GREEN_LED (1 << 9)
#define DEFINED_LEDS (BLUE_LED | GREEN_LED)
#define IGNORE_UNDEFINED_LEDS(leds) (leds &= DEFINED_LEDS)
typedef enum {OFF = 0, ON = 1} LED_state;

/* Configure the SPI port */
#define TRANSMITTING_SPI SPI1
#define TRANSMITTING_SPI_GPIO_PORT GPIOA
#define TRANSMITTING_SPI_MISO_PIN GPIO_Pin_6
#define TRANSMITTING_SPI_MOSI_PIN GPIO_Pin_7
#define TRANSMITTING_SPI_NSS_PIN GPIO_Pin_4
#define TRANSMITTING_SPI_SCLK_PIN GPIO_Pin_5
#define ENABLE_TRANSMITTING_SPI_GPIO_PERIPHERAL (RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE))
#define ENABLE_TRANSMITTING_SPI_PERIPHERAL (RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE))

/* the following are the GPIO ports used for A to D inputs.
   ADC0 corresponds to channel 0; ADC1 to channel 1, etc.
   currently, ADC inputs 8 - 15 are used */
#define ADC0_GPIO_PORT GPIOB
#define ADC0_GPIO_PIN GPIO_Pin_0
#define ADC0_INPUT ADC_Channel_8
#define ADC0_ENABLE_GPIO_PERIPHERAL (RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE))

#define ADC1_GPIO_PORT GPIOB
#define ADC1_GPIO_PIN GPIO_Pin_1
#define ADC1_INPUT ADC_Channel_9
#define ADC1_ENABLE_GPIO_PERIPHERAL (RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE))

#define ADC2_GPIO_PORT GPIOC
#define ADC2_GPIO_PIN GPIO_Pin_0
#define ADC2_INPUT ADC_Channel_10
#define ADC2_ENABLE_GPIO_PERIPHERAL (RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE))

#define ADC3_GPIO_PORT GPIOC
#define ADC3_GPIO_PIN GPIO_Pin_1
#define ADC3_INPUT ADC_Channel_11
#define ADC3_ENABLE_GPIO_PERIPHERAL (RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE))

#define ADC4_GPIO_PORT GPIOC
#define ADC4_GPIO_PIN GPIO_Pin_2
#define ADC4_INPUT ADC_Channel_12
#define ADC4_ENABLE_GPIO_PERIPHERAL (RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE))

#define ADC5_GPIO_PORT GPIOC
#define ADC5_GPIO_PIN GPIO_Pin_3
#define ADC5_INPUT ADC_Channel_13
#define ADC5_ENABLE_GPIO_PERIPHERAL (RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE))

#define ADC6_GPIO_PORT GPIOC
#define ADC6_GPIO_PIN GPIO_Pin_4
#define ADC6_INPUT ADC_Channel_14
#define ADC6_ENABLE_GPIO_PERIPHERAL (RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE))

#define ADC7_GPIO_PORT GPIOC
#define ADC7_GPIO_PIN GPIO_Pin_5
#define ADC7_INPUT ADC_Channel_15
#define ADC7_ENABLE_GPIO_PERIPHERAL (RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE))



/* a timer is used to provide a primitive sleep() function */
#define TIMER TIM6
#define ENABLE_TIMER_PERIPHERAL (RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE));

/* The state variable commicates to the main loop when to
   transmit more data over SPI. */
enum {IDLE, TRANSMIT_FRAME, TRANSMITTING_FRAME} state = IDLE;

static uint8_t frame[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
						  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
						  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
						  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
						  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
						  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
						  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
						  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
			 
#define FRAME_LENGTH(frame) (sizeof(frame)/sizeof(frame[0]))

#define GREEN (0x01)
#define RED (0x02)
#define ORANGE (0x03)

/*
A channel stores the information needed to display a bar whose height 
indicates a value of some sort.  The current physical display cannot display 
more than 8 bars at a time.  It is up to the user to make sure that two 
channels don't use the same display column.
*/
#define NUMBER_OF_LEVELS_PER_CHANNEL (8)
typedef struct Channel {
	uint16_t levels[NUMBER_OF_LEVELS_PER_CHANNEL];
	uint8_t current_level_index;
	uint8_t display_column;
} Channel;

#define NUMBER_OF_CHANNELS (8)
#define DO_NOT_DISPLAY (0xFF)
static Channel channels[NUMBER_OF_CHANNELS];

static bool channel_is_configured(Channel *channel);
static void clear_frame(unsigned int frame_length, uint8_t frame[frame_length]);
static void configure_ADC(void);
static void configure_channel(Channel *channel, uint8_t display_column, uint16_t breaks[8]);
static void configure_LED_GPIO_pins(void);
static void configure_sleep_timer(void);
static void configure_SPI(void);
static void configure_system_ticks(void);
static void convert_analog(void);
static void draw_channel(Channel *channel, uint8_t frame[]);
static void draw_frame(unsigned int frame_length, uint8_t frame[frame_length]);
static void initialize_channel(Channel *channel);
static void initialize_channels(void);
static void set_channel_level(Channel *channel, uint16_t level);
static void set_LED(uint16_t led, LED_state state);
static void sleep(uint16_t tenths_of_millisecond);
static void toggle_LED(uint16_t led);
static void transmit_frame_to_matrix(unsigned int frame_length, uint8_t frame[frame_length]);

int
main(int argc, char **argv) 
{
	configure_system_ticks();
	configure_sleep_timer();
	configure_LED_GPIO_pins();
	configure_SPI();
	configure_ADC();
	initialize_channels();

	clear_frame(FRAME_LENGTH(frame), frame);

	uint16_t levels[NUMBER_OF_LEVELS_PER_CHANNEL] = {347, 640, 890, 1107, 1296, 1462, 1610, 1743};
	configure_channel(&channels[0], 0, levels);

	while (true) {
		if (state == TRANSMIT_FRAME) {
			state = TRANSMITTING_FRAME;
			convert_analog();
			draw_frame(FRAME_LENGTH(frame), frame);
			transmit_frame_to_matrix(FRAME_LENGTH(frame), frame);
			state = IDLE;
		}
	}
}

void
SysTick_Handler(void)
{
	toggle_LED(BLUE_LED);
	if (state == IDLE) {
		state = TRANSMIT_FRAME;
	}
}

/*
Returns true if the channel has been configured; false otherwise.
*/
static bool
channel_is_configured(Channel *channel)
{
	if (channel->display_column < NUMBER_OF_CHANNELS) {
		return true;
	}
	return false;
}

/*
Clears a frame by setting all bytes to 0
*/
static void
clear_frame(unsigned int frame_length, uint8_t frame[frame_length])
{
	memset(frame, 0, frame_length);
}

/*
Configures the #define'd GPIO pins into analog input mode,
calibrates the ADC, configures it to scan the active inputs,
and gets it to start converting.
*/
static void
configure_ADC(void)
{
	/* enable the ADC peripheral */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	/* enable the ports */
	ADC0_ENABLE_GPIO_PERIPHERAL;
	ADC1_ENABLE_GPIO_PERIPHERAL;
	ADC2_ENABLE_GPIO_PERIPHERAL;
	ADC3_ENABLE_GPIO_PERIPHERAL;
	ADC4_ENABLE_GPIO_PERIPHERAL;
	ADC5_ENABLE_GPIO_PERIPHERAL;
	ADC6_ENABLE_GPIO_PERIPHERAL;
	ADC7_ENABLE_GPIO_PERIPHERAL;

	/* configure the GPIO ports */
	GPIO_InitTypeDef gpio_configuration;
	GPIO_StructInit(&gpio_configuration); 
	gpio_configuration.GPIO_Mode = GPIO_Mode_AIN;

	gpio_configuration.GPIO_Pin = ADC0_GPIO_PIN;
	GPIO_Init(ADC0_GPIO_PORT, &gpio_configuration);

	gpio_configuration.GPIO_Pin = ADC1_GPIO_PIN;
	GPIO_Init(ADC1_GPIO_PORT, &gpio_configuration);

	gpio_configuration.GPIO_Pin = ADC2_GPIO_PIN;
	GPIO_Init(ADC2_GPIO_PORT, &gpio_configuration);

	gpio_configuration.GPIO_Pin = ADC3_GPIO_PIN;
	GPIO_Init(ADC3_GPIO_PORT, &gpio_configuration);
  
	gpio_configuration.GPIO_Pin = ADC4_GPIO_PIN;
	GPIO_Init(ADC4_GPIO_PORT, &gpio_configuration);

	gpio_configuration.GPIO_Pin = ADC5_GPIO_PIN;
	GPIO_Init(ADC5_GPIO_PORT, &gpio_configuration);

	gpio_configuration.GPIO_Pin = ADC6_GPIO_PIN;
	GPIO_Init(ADC6_GPIO_PORT, &gpio_configuration);

	gpio_configuration.GPIO_Pin = ADC7_GPIO_PIN;
	GPIO_Init(ADC7_GPIO_PORT, &gpio_configuration);
	
	/* do the ADC calibration (not sure this is doing anything at the moment.) */
	ADC_StartCalibration(ADC1);

	/* configure the ADC to be in scan mode and setup the ADC channels to
	   scan from ADC0 to ADC7 */
	ADC_InitTypeDef adc_configuration;
	ADC_StructInit(&adc_configuration);
	adc_configuration.ADC_NbrOfChannel = 8;
	ADC_Init(ADC1, &adc_configuration);
					  
	ADC_RegularChannelConfig(ADC1, ADC0_INPUT, 1, ADC_SampleTime_1Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC1_INPUT, 2, ADC_SampleTime_1Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC2_INPUT, 3, ADC_SampleTime_1Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC3_INPUT, 4, ADC_SampleTime_1Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC4_INPUT, 5, ADC_SampleTime_1Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC5_INPUT, 6, ADC_SampleTime_1Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC6_INPUT, 7, ADC_SampleTime_1Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC7_INPUT, 8, ADC_SampleTime_1Cycles5);

	/* wake up the ADC from power down by enabling it.
	   conversion will start the next time is it enabled */
	ADC_Cmd(ADC1, ENABLE);
}

/*
Configures a channel with the provided levels.  The provided levels must be 
non-decreasing and the display column must be in the range [0,NUMBER_OF_CHANNELS).
If these conditions aren't met, this routine stops without configuring the channel.
If the levels are configured, the current display level is set to 0.
*/
static void
configure_channel(Channel *channel, uint8_t display_column, uint16_t levels[NUMBER_OF_LEVELS_PER_CHANNEL])
{	
	if (display_column >= NUMBER_OF_CHANNELS) {
		return;
	}

	/* make sure the values are non-decreasing */
	uint16_t previous_level = levels[0];
	for (unsigned int index = 1; index < NUMBER_OF_LEVELS_PER_CHANNEL; index++) {
		if (previous_level > levels[index]) {
			return;
		}
		previous_level = levels[index];
	}

	/* copy the levels into the channel */
	memcpy(channel->levels, levels, sizeof(levels[0]) * NUMBER_OF_LEVELS_PER_CHANNEL);

	channel->display_column = display_column;
	channel->current_level_index = 0;
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
	TIM_PrescalerConfig(TIMER, SystemCoreClock/10000, TIM_PSCReloadMode_Immediate);
	
}

/*
Configures TRANSMITTING_SPI as a master device to communicate to RECEIVING_SPI as a slave device.
*/
static void
configure_SPI(void)
{
	/* Enable the GPIO peripheral; the GPIO peripheral must be enabled before it can be configured. */
	ENABLE_TRANSMITTING_SPI_GPIO_PERIPHERAL;

	GPIO_InitTypeDef gpio_configuration;
	gpio_configuration.GPIO_Speed = GPIO_Speed_50MHz;

	/* Configure the GPIO pins for TRANSMITTING_SPI as a master device.  
	   MOSI & SCLK are configured as alternate function push-pull outputs; 
	   MISO is a floating input. NSS is currently set as an normal output; the 
	   transmit routine controls it directly. */
	gpio_configuration.GPIO_Pin = TRANSMITTING_SPI_MOSI_PIN | TRANSMITTING_SPI_SCLK_PIN;	
	gpio_configuration.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(TRANSMITTING_SPI_GPIO_PORT, &gpio_configuration);

	gpio_configuration.GPIO_Pin = TRANSMITTING_SPI_MISO_PIN;
	gpio_configuration.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(TRANSMITTING_SPI_GPIO_PORT, &gpio_configuration);

	gpio_configuration.GPIO_Pin = TRANSMITTING_SPI_NSS_PIN;
	gpio_configuration.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(TRANSMITTING_SPI_GPIO_PORT, &gpio_configuration);

	/* Set the NSS pin high (it's active low) to indicate that the bus is free */
	GPIO_SetBits(TRANSMITTING_SPI_GPIO_PORT, TRANSMITTING_SPI_NSS_PIN);

	/* Enable the SPI Peripherals; SPI ports must be enabled before they can be configured. */
	ENABLE_TRANSMITTING_SPI_PERIPHERAL;

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
}


static void
configure_system_ticks(void)
{
	/* configure the system tick handler to fire sixty times per second */
	SysTick_Config(SystemCoreClock/60);
}

/*
Converts all of the analog channels
*/
static void
convert_analog(void)
{
	for (unsigned int index = 0; index < NUMBER_OF_CHANNELS; index++) {
		/* start the conversion */
		ADC_Cmd(ADC1, ENABLE);

		/* wait for the conversion to finish, then write the value
		   into the channel */
		while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET) {
			;
		}

		/* read the value into the channel */
		set_channel_level(&channels[index], ADC_GetConversionValue(ADC1));

		/* reset the end of conversion flag and the start flag*/
		ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
		ADC_ClearFlag(ADC1, ADC_FLAG_STRT);
	}
}

/*
Draws the provided channel on the given frame.
*/
static void
draw_channel(Channel *channel, uint8_t frame[])
{
	uint8_t current_color = RED;
	for (unsigned int row = 0; row < channel->current_level_index; row++) {
		if (row == 3) {
			current_color = ORANGE;
		}
		if (row == 6) {
			current_color = GREEN;
		}
		frame[row*8 + channel->display_column] = current_color;
	}
}

/*
Updates the frame (in place) to be the next frame to display
*/
static void
draw_frame(unsigned int frame_length, uint8_t frame[frame_length])
{
	clear_frame(frame_length, frame);
	for (unsigned int index = 0; index < NUMBER_OF_CHANNELS; index++) {
		draw_channel(&channels[index], frame);
	}
}

/*
Initializes (or resets) a channel to the default state; the default
state is all levels set to zero and the current_display_level set to zero.
*/
static void
initialize_channel(Channel *channel)
{
	memset(channel->levels, 0, sizeof(channel->levels));
	channel->current_level_index = 0;
	channel->display_column = DO_NOT_DISPLAY;
}

/*
Initializes all channels.
*/
static void
initialize_channels(void) 
{
	Channel *current_channel = channels;
	for (unsigned int index = 0; index < NUMBER_OF_CHANNELS; index++, current_channel++) {
		initialize_channel(current_channel);
	}
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
Sets the level of the provided channel.  If the channel has
not yet been configured, this routine does nothing.
*/
static void
set_channel_level(Channel *channel, uint16_t level)
{
	if (!channel_is_configured(channel)) {
		return;
	}

	uint8_t new_level_index = 0;
	while (channel->levels[new_level_index] < level &&
		   new_level_index < NUMBER_OF_LEVELS_PER_CHANNEL) {
		new_level_index++;
	}

	channel->current_level_index = new_level_index;
	return;
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


static void
transmit_frame_to_matrix(unsigned int frame_length, uint8_t frame[frame_length])
{
	/* assume control of the bus; it's active low. */
    GPIO_ResetBits(TRANSMITTING_SPI_GPIO_PORT, TRANSMITTING_SPI_NSS_PIN);
	sleep(5);

	SPI_SSOutputCmd(TRANSMITTING_SPI, ENABLE);
	SPI_Cmd(TRANSMITTING_SPI, ENABLE);

	for (unsigned int index = 0; index < frame_length; index++) {
		while (!SPI_I2S_GetFlagStatus(TRANSMITTING_SPI, SPI_I2S_FLAG_TXE)) { }
		SPI_I2S_SendData(TRANSMITTING_SPI, frame[index]);
	}

	while (!SPI_I2S_GetFlagStatus(TRANSMITTING_SPI, SPI_I2S_FLAG_TXE)) { }
	while (SPI_I2S_GetFlagStatus(TRANSMITTING_SPI, SPI_I2S_FLAG_BSY)) { }

	SPI_Cmd(TRANSMITTING_SPI, DISABLE);
	SPI_SSOutputCmd(TRANSMITTING_SPI, DISABLE);

	sleep(5);
	/* release control of the bus; it's active low. */
	GPIO_SetBits(TRANSMITTING_SPI_GPIO_PORT, TRANSMITTING_SPI_NSS_PIN);
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
	TIM_ClearFlag(TIMER, TIM_FLAG_Update);
	while (!TIM_GetFlagStatus(TIMER, TIM_FLAG_Update)) { }
}
