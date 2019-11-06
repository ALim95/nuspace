#include <math.h>

#include "lpc17xx_pinsel.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_ssp.h"
#include "lpc17xx_timer.h"

#include "temp.h"
#include "pca9532.h"
#include "acc.h"
#include "oled.h"
#include "rgb.h"
#include "led7seg.h"
#include "light.h"

typedef enum {
	STATIONARY_MODE,
	LAUNCH_MODE,
	RETURN_MODE,
	TEST_SIMUL_MODE,
	TEST_LIVE_MODE
} system_mode;

#define NORMAL_STATE 0x0
#define TEMP_WARNING 0x1
#define OBST_WARNING 0x2
#define ACC_WARNING 0x4

#define TEMP_HIGH_THRESHOLD 28
#define OBSTACLE_NEAR_THRESHOLD 3000
#define ACC_THRESHOLD 0.4
#define NUM_PERIODS 170

/**
 * Warning Messages for OLED and UART
 */
#define TEMP_WARNING_OLED "Temp. too high"
#define TEMP_WARNING_UART "Temp. too high. \r\n"
#define ACC_WARNING_OLED "Veer off course"
#define ACC_WARNING_UART "Veer off course. \r\n"
#define OBST_WARNING_OLED "Obstacle near"
#define OBST_WARNING_UART "Obstacle near. \r\n"
#define OBST_AVOIDED_UART "Obstacle Avoided. \r\n"
/**
 * UART Data Messages
 */
#define UART_VALID_RESPONSE "RPT\r\n"
#define LAUNCH_DATA_MESSAGE "Temp : %2.2f; ACC X : %05.2f, Y : %05.2f \r\n"
#define RETURN_DATA_MESSAGE "Obstacle distance : %d \r\n"

#define MESSAGE_INTERVAL 10000

#define STARTING_UART_MESSAGE "Entering STATIONARY Mode \r\n"
#define TOGGLE_MODE_MESSAGE "Entering %s Mode \r\n"

volatile uint32_t msTicks; // counter for 1ms SysTicks

/**
 * Flags used in interrupts
 */
volatile int sw3_flag = 0;
volatile int test_mode_flag = 0;
volatile int temp_flag = 0;
volatile int light_flag = 0;
volatile int rx_flag = 0;
volatile int message_flag = 0;
volatile int blinky = 0;
volatile int push_right = 0;
volatile int push_left = 0;
volatile int push_up = 0;
volatile int push_down = 0;

char data_rx[64] = {}; // Used as a buffer to receive UART transmission

/**
 * Character to be displayed on 7-segment display
 */
static uint8_t letter[16] = {
/* digits 0 - 9 */
0x24 , 0x7D , 0xE0 , 0x70 , 0x39 , 0x32 , 0x22 , 0x7C , 0x20 , 0x30 ,
/* digits A - F */
0x28 , 0x23 , 0xA6 , 0x61 , 0xA2 , 0xAA
 };

/**
 * Mask value to be used for 16 array LEDs
 */
static uint16_t led_on[17] = {
//Red + all green lit up
0xFFFF, 0xFF7F, 0xFF3F, 0xFF1F, 0xFF0F ,0xFF07, 0xFF03, 0xFF01,
//Green + no red
0xFF00, 0xFE00, 0xFC00, 0xF800, 0xF000, 0xE000, 0xC000, 0x8000,
// None
0x0000
};

system_mode current_mode = STATIONARY_MODE;
uint8_t current_state = NORMAL_STATE;
uint32_t countdown_time = 0;
volatile uint32_t last_SW3_time = 0;
volatile uint32_t start_period = 0;
volatile int32_t temp_value = 0;
volatile uint32_t period_count = 0;
volatile uint32_t period_time = 0;


/**
 * SysTick Handler will handle the SysTick Interrupt, which will trigger every 1ms
 */
void SysTick_Handler(void) {
	msTicks++;
	if (current_state != NORMAL_STATE && (current_mode != RETURN_MODE)){
		flash_warning();
	}
}

/**
 * Every 333ms, flash_warning() will cause the red and blue LED to turn on/off
 * depending on the blinky flag, and the warning state of the system.
 */
void flash_warning() {
	if (msTicks % 333 == 0) {
		if (blinky == 0 && ((current_state & 0x1))) {
		//If blinky = 0, and system has a TEMP_WARNING, turn on red LED, and turn off blue LED
			LPC_GPIO2 ->FIOPIN |= 1;
			LPC_GPIO0 ->FIOPIN &= ~(1 << 26);
		} else if (blinky == 1 && ((current_state >> 2 & 0x1))) {
		//If blinky = 1, and system has a ACC_WARNING, turn off red LED, and turn on blue LED
			LPC_GPIO0 ->FIOPIN |= 1 << 26;
			LPC_GPIO2 ->FIOPIN &= ~1;
		} else {
		//Otherwise, turn off both red and blue LED
			LPC_GPIO0 ->FIOPIN &= ~(1 << 26);
			LPC_GPIO2 ->FIOPIN &= ~1;
		}

		blinky = !blinky; //toggle the blinky flag
	}
}

uint32_t getTicks(void) {
	return msTicks;
}

/**
 * EINT0_IRQHandler will run whenever the SW3 button is pressed. It will set the sw3_flag to its values
 * respectively, and the system will change its mode based on the sw3_flag values.
 *
 * If the SW3 button has been pressed more than once between the previous 1 second, and system is in
 * LAUNCH_MODE or TEST_LIVE_MODE, sw3_flag = 2
 * Otherwise, sw3_flag = 1
 */
void EINT0_IRQHandler(void) {

	if((msTicks - last_SW3_time <= 1000) && (current_mode == LAUNCH_MODE || current_mode == TEST_LIVE_MODE)) {
		sw3_flag = 2;
	} else {
		sw3_flag = 1;
	}
	last_SW3_time = msTicks; // update last_SW3_time with current time
	LPC_SC->EXTINT |= 1;	// Clear EINT0 interrupt
}

/**
 * Returns the 10 x T(c), i.e. 10 times the temperature in Celcius.
 * Example: If the temperature is 22.4 degrees the returned value is 224.
 */
int32_t modified_temp_read (void){
	return ( (2*1000*period_time) / (NUM_PERIODS * 2) - 2731 );
}

/**
 * EINT3_IRQHandler will run whenever the temperature sensor's pin (Port 0, Pin 2),
 * goes from 1 to 0. Whenever EINT3 is triggered, period_count will increment by 1.
 * When period_count reaches NUM_PERIODS, the time taken for it to go from 0 to
 * NUM_PERIODS is recorded in period_time.
 * In all modes, except TEST_SIMUL_MODE, the period_time is used to calculate the
 * temperature of the surroundings.
 *
 * In TEST_SIMUL_MODE, the EINT3_IIRQHandler will also handle the interrupts triggered
 * by the joystick. It will set the flag of the joystick direction pushed to 1.
 */
void EINT3_IRQHandler(void) {

	if ((LPC_GPIOINT->IO0IntStatF >> 2) & 0x1){ // Temperature
		period_count++;
		if (period_count % NUM_PERIODS == 0){
			period_time = msTicks - start_period;
			period_count = 0;
			start_period = msTicks;
			if (current_mode != TEST_SIMUL_MODE){
				temp_value = modified_temp_read();
			}
		}
		// Clear GPIO Interrupt P0.2
		LPC_GPIOINT->IO0IntClr = 1<<2;
	}

	if ((LPC_GPIOINT->IO0IntStatF >> 17) & 0x1){ // joystick center button
		if (current_mode == STATIONARY_MODE || current_mode == TEST_SIMUL_MODE){
			test_mode_flag = !test_mode_flag;
		}
		// Clear GPIO Interrupt P0.17
		LPC_GPIOINT->IO0IntClr = 1<<17;
	}

	if ((LPC_GPIOINT->IO0IntStatF >> 16) & 0x1){ // joystick right button
		push_right = 1;
		// Clear GPIO Interrupt P0.16
		LPC_GPIOINT->IO0IntClr = 1<<16;
	}

	if ((LPC_GPIOINT->IO2IntStatF >> 4) & 0x1){ // joystick left button
		push_left = 1;
		// Clear GPIO Interrupt P2.4
		LPC_GPIOINT->IO2IntClr = 1<<4;
	}

	if ((LPC_GPIOINT->IO2IntStatF >> 3) & 0x1){ // joystick up button
		push_up = 1;
		// Clear GPIO Interrupt P2.3
		LPC_GPIOINT->IO2IntClr = 1<<3;
	}

	if ((LPC_GPIOINT->IO0IntStatF >> 15) & 0x1){ // joystick down button
		push_down = 1;
		// Clear GPIO Interrupt P0.15
		LPC_GPIOINT->IO0IntClr = 1<<15;
	}
}

/**
 * UART3_IRQHandler is triggered whenever there is a RDA or CTI interrupt. The character received from
 * UART_Receive() will be put into the char array, data_rx[].
 * Sets the rx_flag to 1 if data_rx[] is "RPT\r\n"
 */
void UART3_IRQHandler(void) {
	char rx_char;
	int len = strlen(data_rx);
	UART_Receive(LPC_UART3, &rx_char, 1, BLOCKING);
	data_rx[len++] = rx_char;
	if (rx_char == '\n' ||len >= sizeof(data_rx)){
		if (!strcmp(data_rx, UART_VALID_RESPONSE)) {
			rx_flag = 1;
		}
		memset(&data_rx[0], '\0', sizeof(data_rx)); //clears data_rx for next iteration
	}
}

void TIMER1_IRQHandler (void){
    if((LPC_TIM1->IR>>1 & 0x01) == 0x01) // if MR1 interrupt
    {
        LPC_TIM1->IR |= 1 << 1; // Clear MR1 interrupt flag
        message_flag = 1;
    }
}

static void init_ssp(void)
{
	SSP_CFG_Type SSP_ConfigStruct;
	PINSEL_CFG_Type PinCfg;

	/*
	 * Initialize SPI pin connect
	 * P0.7 - SCK;
	 * P0.8 - MISO
	 * P0.9 - MOSI
	 * P2.2 - SSEL - used as GPIO
	 */
	PinCfg.Funcnum = 2;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 7;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 8;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 9;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Funcnum = 0;
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 2;
	PINSEL_ConfigPin(&PinCfg);

	SSP_ConfigStructInit(&SSP_ConfigStruct);

	// Initialize SSP peripheral with parameter given in structure above
	SSP_Init(LPC_SSP1, &SSP_ConfigStruct);

	// Enable SSP peripheral
	SSP_Cmd(LPC_SSP1, ENABLE);

}

static void init_i2c(void)
{
	PINSEL_CFG_Type PinCfg;

	/* Initialize I2C2 pin connect */
	PinCfg.Funcnum = 2;
	PinCfg.Pinnum = 10;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 11;
	PINSEL_ConfigPin(&PinCfg);

	// Initialize I2C peripheral
	I2C_Init(LPC_I2C2, 100000);

	/* Enable I2C1 operation */
	I2C_Cmd(LPC_I2C2, ENABLE);
}

static void init_GPIO(void){
	PINSEL_CFG_Type PinCfg;

	// Initialize SW3 as EINT0
	PinCfg.Funcnum = 1;
	PinCfg.Pinnum = 10;
	PinCfg.Portnum = 2;
	PINSEL_ConfigPin(&PinCfg);
}

static void init_timer(void){
	TIM_TIMERCFG_Type TIM_ConfigStruct;
	TIM_MATCHCFG_Type TIM_MatchConfigStruct ;
	TIM_ConfigStruct.PrescaleOption = TIM_PRESCALE_USVAL;
	TIM_ConfigStruct.PrescaleValue	= 1000;

	// use channel 1, MR1
	TIM_MatchConfigStruct.MatchChannel = 1;
	// Enable interrupt when MR1 matches the value in TC register
	TIM_MatchConfigStruct.IntOnMatch   = TRUE;
	//Enable reset on MR1: TIMER will reset if MR1 matches it
	TIM_MatchConfigStruct.ResetOnMatch = TRUE;
	//Do not stop on MR1 if MR1 matches it
	TIM_MatchConfigStruct.StopOnMatch  = FALSE;
	//do nothing for external output
	TIM_MatchConfigStruct.ExtMatchOutputType =TIM_EXTMATCH_NOTHING;
	// Set Match value to be 10,000 ms = 10s
	TIM_MatchConfigStruct.MatchValue = MESSAGE_INTERVAL;

	// Set configuration for Tim_config and Tim_MatchConfig
	TIM_Init(LPC_TIM1, TIM_TIMER_MODE, &TIM_ConfigStruct);
	TIM_ConfigMatch(LPC_TIM1,&TIM_MatchConfigStruct);

	// Clearing interrupts and IRQ before enabling the IRQ
	LPC_TIM1->IR |= 1 << 1;
	NVIC_ClearPendingIRQ(TIMER1_IRQn);
	NVIC_EnableIRQ(TIMER1_IRQn);
}

static void pinsel_UART3(void){
    PINSEL_CFG_Type PinCfg;
    PinCfg.Funcnum = 2;
    PinCfg.Pinnum = 0;
    PinCfg.Portnum = 0;
    PINSEL_ConfigPin(&PinCfg);
    PinCfg.Pinnum = 1;
    PINSEL_ConfigPin(&PinCfg);
}

static void init_UART(void){
	UART_CFG_Type uartCfg;
	uartCfg.Baud_rate = 115200;
	uartCfg.Databits = UART_DATABIT_8;
	uartCfg.Parity = UART_PARITY_NONE;
	uartCfg.Stopbits = UART_STOPBIT_1;
	//pin select for uart3;
	pinsel_UART3();
	//supply power & setup working parameters for uart3
	UART_Init(LPC_UART3, &uartCfg);
	//enable transmit for uart3
	UART_TxCmd(LPC_UART3, ENABLE);
}

static void init_UARTinterrupt(void){
	// Initialize FIFO config
	UART_FIFO_CFG_Type UART_FIFOInitStruct;
	UART_FIFOConfigStructInit(&UART_FIFOInitStruct);
	UART_FIFOConfig(LPC_UART3, &UART_FIFOInitStruct);

	// Initialize UART int config
	UART_IntConfig(LPC_UART3, UART_INTCFG_RBR, ENABLE);

	//Enable UART3 interrupt
	NVIC_ClearPendingIRQ(UART3_IRQn);
	NVIC_EnableIRQ(UART3_IRQn);
}

static void init_EINTinterrupts(void){
	// Enable EINT0 interrupt for SW3
	LPC_SC->EXTINT |= 1;
	LPC_SC->EXTMODE |= 1; // set it to be edge-sensitive
	NVIC_ClearPendingIRQ(EINT0_IRQn);
	NVIC_EnableIRQ(EINT0_IRQn);

	// Enable EINT3 interrupts
	LPC_GPIOINT->IO0IntClr = 1<<2; // temp sensor
	LPC_GPIOINT->IO0IntEnF |= 1<<2;

	LPC_GPIOINT->IO0IntClr = 1<<17; // joystick center
	LPC_GPIOINT->IO0IntEnF |= 1<<17;

	LPC_GPIOINT->IO0IntClr = 1<<16; // joystick right
	LPC_GPIOINT->IO0IntEnF |= 1<<16;

	LPC_GPIOINT->IO2IntClr = 1<<4; // joystick left
	LPC_GPIOINT->IO2IntEnF |= 1<<4;

	LPC_GPIOINT->IO2IntClr = 1<<3; // joystick up
	LPC_GPIOINT->IO2IntEnF |= 1<<3;

	LPC_GPIOINT->IO0IntClr = 1<<15; // joystick down
	LPC_GPIOINT->IO0IntEnF |= 1<<15;

	NVIC_ClearPendingIRQ(EINT3_IRQn);
	NVIC_EnableIRQ(EINT3_IRQn);
}

/**
 * Sets the priority levels for the respective interrupts used.
 * Interrupt priority is shown below, in descending order:
 * SysTick > EINT3 > UART3 > TIMER1 > EINT0
 */
static void set_int_priority(void){
	NVIC_SetPriorityGrouping(2); //Priority Group 2 is used
	NVIC_SetPriority(SysTick_IRQn, 0); //SysTick Interrupt
	NVIC_SetPriority(EINT3_IRQn, 1); // Temperature Interrupt
	NVIC_SetPriority(UART3_IRQn, 2); // UART Interrupt
	NVIC_SetPriority(TIMER1_IRQn, 3); // Timer1 Interrupt
	NVIC_SetPriority(EINT0_IRQn, 4); // SW3 button
}

/**
 * Displays temperature value on OLED
 */
void display_temp(char *display) {
	sprintf(display, "Temp = %2.2f", temp_value/10.0);
	oled_putString(0,10, display, OLED_COLOR_WHITE,OLED_COLOR_BLACK);
}

/**
 * Displays accelerometer readings of X and Y on OLED
 */
void display_acc(char *display, int8_t x, int8_t y){
	sprintf(display, "X = %05.2f", x / 64.0);
	oled_putString(0,30, display, OLED_COLOR_WHITE,OLED_COLOR_BLACK);
	sprintf(display, "Y = %05.2f", y / 64.0);
	oled_putString(0,40, display, OLED_COLOR_WHITE,OLED_COLOR_BLACK);
}

/**
 * Displays countdown on 7-segment display
 */
void display_countdown(int countdown) {
	led7seg_setChar(letter[countdown], TRUE);
	countdown_time = msTicks;
}

/**
 * Checks for obstacle, using the lightValue as reference.
 * If lightValue is below OBSTACLE_NEAR_THRESHOLD, and system is not in OBST_WARNING,
 * it will display obstacle warning message on UART and OLED.
 *
 * If lightValue is greater than OBSTACLE_NEAR_THRESHOLD and system is in OBST_WARNING,
 * it will clear obstacle warning message on OLED, and send "Obstacle Avoided" message
 * on UART.
 */
void check_obstacle(uint32_t lightValue){
	if (lightValue < OBSTACLE_NEAR_THRESHOLD && (current_state>>1 & 0x1) == 0){
		current_state |= OBST_WARNING;
		oled_putString(0,10, OBST_WARNING_OLED, OLED_COLOR_WHITE,OLED_COLOR_BLACK);
		UART_SendString(LPC_UART3, OBST_WARNING_UART);
	} else if (lightValue >= OBSTACLE_NEAR_THRESHOLD && (current_state>>1 & 0x1)){
		current_state &= !OBST_WARNING;
		oled_fillRect(0,10,95,20,OLED_COLOR_BLACK);
		UART_SendString(LPC_UART3, OBST_AVOIDED_UART);
	}
}

/**
 * Changes the system mode, to the next respective mode. Initializes system for its next
 * mode, and clears OLED screen and displays the nextMode information. Also sends mode
 * change info through UART.
 *
 * Params:
 *   [in] nextMode - the next mode that the system is going to enter
 */
void toggle_mode(system_mode nextMode) {
	char bufferString[50];
	char nextModeName[20];

	switch(nextMode) {
	case STATIONARY_MODE:
	//Initializing system for STATIONARY_MODE
		strcpy(nextModeName, "STATIONARY");
		current_state &= !OBST_WARNING;
		led7seg_setChar(letter[15], TRUE);
		pca9532_setLeds(0x0, 0xFFFF);
		start_period = msTicks;
		period_count = 0;
		NVIC_ClearPendingIRQ(EINT3_IRQn);
		NVIC_ClearPendingIRQ(TIMER1_IRQn);
		NVIC_EnableIRQ(EINT3_IRQn);
		TIM_Cmd(LPC_TIM1,DISABLE);
		LPC_TIM1->TC = 0;
		LPC_TIM1->PC = 0;
		break;
	case LAUNCH_MODE:
	//Initializing system for LAUNCH_MODE
		strcpy(nextModeName, "LAUNCH");
		TIM_Cmd(LPC_TIM1,ENABLE);
		break;
	case RETURN_MODE:
	//Initializing system for RETURN_MODE
		strcpy(nextModeName, "RETURN");
		current_state = NORMAL_STATE;
		GPIO_ClearValue(2,1);
		GPIO_ClearValue(0,1<<26);
		NVIC_DisableIRQ(EINT3_IRQn);
		LPC_TIM1->TC = 0;
		LPC_TIM1->PC = 0;
		break;
	case TEST_SIMUL_MODE:
	//Initializing system for TEST_SIMUL_MODE
		strcpy(nextModeName, "Simulation Test");
		led7seg_setChar(letter[15], TRUE);
		test_mode_flag = 0;
		temp_value = 250;
		NVIC_ClearPendingIRQ(TIMER1_IRQn);
		break;
	case TEST_LIVE_MODE:
	//Initializing system for TEST_LIVE_MODE
		strcpy(nextModeName, "Live Test");
		test_mode_flag = 0;
		current_state = NORMAL_STATE;
		GPIO_ClearValue(2,1);
		GPIO_ClearValue(0,1<<26);
		last_SW3_time -= 1000;
		TIM_Cmd(LPC_TIM1,ENABLE);
		break;
	}

	current_mode = nextMode;
	sw3_flag = 0; //resets sw3_flag to 0, as mode change is complete

	oled_clearScreen(OLED_COLOR_BLACK);
	oled_putString(0,0,nextModeName, OLED_COLOR_WHITE,OLED_COLOR_BLACK);
	sprintf(bufferString, TOGGLE_MODE_MESSAGE, nextModeName);
    UART_SendString(LPC_UART3, bufferString);

}

/**
 * Initialize all peripherals
 */
void init_peripherals() {
	pca9532_init();
	rgb_init();
	acc_init();
	oled_init();
	led7seg_init();
	temp_init(getTicks);
	light_enable();
	light_setRange(LIGHT_RANGE_4000);
}

/**
 * Initialize all pin configurations
 */
void init_pinconfig() {
	init_i2c();
	init_ssp();
	init_GPIO();
	init_UART();
}

/**
 * Clears the warning displays on OLED, and turns off both red and blue LEDs
 */
void clear_warning() {
	current_state = NORMAL_STATE;
	oled_fillRect(0, 20, 95, 30, OLED_COLOR_BLACK);
	oled_fillRect(0, 50, 95, 60, OLED_COLOR_BLACK);
	GPIO_ClearValue(2, 1);
	GPIO_ClearValue(0, 1 << 26);
}

/**
 * Transmits temperature and accelerometer readings of X and Y through UART
 */
void transmit_launch_data(char data_buf[50], int8_t x, int8_t y) {
	sprintf(data_buf, LAUNCH_DATA_MESSAGE, temp_value / 10.0, x / 64.0, y / 64.0);
	UART_SendString(LPC_UART3, data_buf);
}

/**
 * Transmits light sensor value through UART
 */
void transmit_return_data(char data_buf[50], uint32_t lightValue) {
	sprintf(data_buf, RETURN_DATA_MESSAGE, lightValue);
	UART_SendString(LPC_UART3, data_buf);
}

/**
 * Toggles the current state to the warning state and outputs the message to OLED and transmit through UART
 */
void toggle_warning(uint8_t warning_state) {
	current_state |= warning_state;

	if (warning_state == TEMP_WARNING){
		oled_putString(0, 20, TEMP_WARNING_OLED, OLED_COLOR_WHITE,
				OLED_COLOR_BLACK);
		if (current_mode != STATIONARY_MODE) {
			//Only sends temp warning message through UART if system is not in STATIONARY_MODE
			UART_SendString(LPC_UART3, TEMP_WARNING_UART);
		}
	} else if (warning_state == ACC_WARNING){
		oled_putString(0,50, ACC_WARNING_OLED, OLED_COLOR_WHITE,OLED_COLOR_BLACK);
		UART_SendString(LPC_UART3, ACC_WARNING_UART);
	}
}

int main (void) {

	// Offset to account for any errors in reading of accelerometer
    int32_t xoff = 0;
    int32_t yoff = 0;
    int32_t zoff = 0;

    // Accelerometer reading to be displayed on OLED and transmitted through UART
    int8_t x = 0;
    int8_t y = 0;
    int8_t z = 0;

    int lightValue; // Used for light sensor reading

    int countdown = 15; // Used for character in array
    char display[40] = {}; // Used to hold string to display in OLED
    char data_buf[50] = {}; // Used to hold string to be transmitted through UART

    SysTick_Config(SystemCoreClock / 1000);

    // Initializes necessary configuration for peripherals and interrupts
	init_pinconfig();
	init_peripherals();
	init_timer();
    init_EINTinterrupts();
    init_UARTinterrupt();
    set_int_priority();

    start_period = msTicks;

    // Initializes 16-array LEDs to be offed at the start, and set 7-segment display to 'F'
    pca9532_setLeds(0x0, 0xFFFF);
    led7seg_setChar(letter[15], TRUE);

	// Assume base board in zero-g position when reading first value.
    acc_read(&x, &y, &z);
    xoff = 0-x;
    yoff = 0-y;
    zoff = 64-z;

    // Initializes OLED screen to display "STATIONARY"
    oled_clearScreen(OLED_COLOR_BLACK);
    oled_putString(0,0, "STATIONARY", OLED_COLOR_WHITE, OLED_COLOR_BLACK);

    // Transmit starting UART message
    UART_SendString(LPC_UART3, STARTING_UART_MESSAGE);
    while (1)
    {

		// Stationary mode
    	if (current_mode == STATIONARY_MODE){

			if (current_state == TEMP_WARNING){
			//If system has TEMP_WARNING, reset 7-segment display to F, and cancel countdown
				countdown = 15;
				sw3_flag = 0;
				led7seg_setChar(letter[15], TRUE);
			} else if (test_mode_flag == 1){
			//If system is not in TEMP_WARNING and joystick center button is pressed, system will go to TEST_SIMUL_MODE
				countdown = 15;
				x = 13;
				y = 13;
				toggle_mode(TEST_SIMUL_MODE);
			} else if (sw3_flag == 1){
			//If system is not in TEMP_WARNING, and SW3 is pressed, update 7-segment display countdown every 1 second.
				if(msTicks - countdown_time >= 1000){
					display_countdown(--countdown);
					if (countdown == 0){
					//If 7-segment displays reaches 0, system will go into LAUNCH_MODE
						toggle_mode(LAUNCH_MODE);
					}
				}
			}
		}

		// Test Simulation mode
		if (current_mode == TEST_SIMUL_MODE){
			display_acc(display, x, y);
			if (test_mode_flag == 0){
				if (push_right == 1){ //increases x by 1 if joystick is pushed to the right
					x++;
					push_right = 0;
				} else if (push_left == 1){ //decreases x by 1 if joystick is pushed to the left
					x--;
					push_left = 0;
				} else if (push_up == 1){ //increases y by 1 if joystick is pushed upwards
					y++;
					push_up = 0;
				} else if (push_down == 1){ //decreases y by 1 if joystick is pushed downwards
					y--;
					push_down = 0;
				}
			} else if (test_mode_flag == 1){
				if (push_right == 1 || push_up == 1){ //increases temperature by 1 if joystick is pushed to the right/up
					temp_value += 10;
					push_right = 0;
					push_up = 0;
				} else if (push_left == 1 || push_down == 1){ //decreases temperature by 1 if joystick is pushed to the left/down
					temp_value -= 10;
					push_left = 0;
					push_down = 0;
				}
			}

			//If magnitude of accelerometer reading X or Y is beyond ACC_THRESHOLD, set system to ACC_WARNING
			if ((fabs(x/64.0) > ACC_THRESHOLD || fabs(y/64.0) > ACC_THRESHOLD) && ((current_state & ACC_WARNING) == 0)){
				toggle_warning(ACC_WARNING);
			}

			//If SW3 is pressed, system is now in TEST_LIVE_MODE
			if (sw3_flag == 1){
				toggle_mode(TEST_LIVE_MODE);
			}
		}
		
    	// Launch and Test Live Mode
    	if ((current_mode == LAUNCH_MODE) || (current_mode == TEST_LIVE_MODE)){
    		acc_read(&x, &y, &z);
			x = x+xoff;
			y = y+yoff;
			display_acc(display, x, y);

			if (rx_flag == 1){
			//If "RPT\r\n" is received from UART, transmit temperature and accelerometer readings through UART
				transmit_launch_data(data_buf, x, y);
				rx_flag = 0; //set rx_flag back to 0
			}

			if (message_flag == 1){
			//If the 10 seconds Timer Interrupt occurs, transmit temperature and accelerometer readings through UART
				transmit_launch_data(data_buf, x, y);
				message_flag = 0; //set message_flag back to 0
			}

			//If magnitude of accelerometer reading X or Y is beyond ACC_THRESHOLD, set system to ACC_WARNING
			if ((fabs(x/64.0) > 0.4 || fabs(y/64.0) > 0.4) && ((current_state & ACC_WARNING) == 0)){
				toggle_warning(ACC_WARNING);
			}

			/**
			 * If SW3 is pressed more than once in 1 second, and current mode is LAUNCH_MODE, enter RETURN_MODE
			 * If current mode is TEST_LIVE_MODE, enter STATIONARY_MODE
			 */
			if (sw3_flag == 2){
				if (current_mode == LAUNCH_MODE){
					toggle_mode(RETURN_MODE);
				} else if (current_mode == TEST_LIVE_MODE){
					current_state = NORMAL_STATE;
					GPIO_ClearValue(2,1);
					GPIO_ClearValue(0,1<<26);
					toggle_mode(STATIONARY_MODE);
				}
			}
    	}


		//Reads and displays temperature on OLED
    	//Clears warning if SW4 is pressed
		//Sets temperature warning if temperature exceeds threshold and current state is not in TEMP_WARNING
		if ((current_mode != RETURN_MODE)){
	    	// CLEAR_WARNING with SW4
			if ((GPIO_ReadValue(1)>>31 & 0x01) == 0){
				clear_warning();
			}
			display_temp(display);
			if (temp_value > TEMP_HIGH_THRESHOLD*10 && ((current_state & TEMP_WARNING) == 0)){
				toggle_warning(TEMP_WARNING);
			}
		}

    	//Return mode
    	if (current_mode == RETURN_MODE){
    		lightValue = light_read();
    		check_obstacle(lightValue);
    		//Sets the number of PCA9532 Light Arrays to be lit up, according to lightValue.
    		pca9532_setLeds(led_on[lightValue/243], 0xFFFF);

    		//If "RPT\r\n" is received from UART, transmit light sensor value through UART
    		if (rx_flag == 1){
    			transmit_return_data(data_buf, lightValue);
				rx_flag = 0;
    		}

    		//If the 10 seconds Timer Interrupt occurs, transmit light sensor value through UART
    		if (message_flag == 1){
				transmit_return_data(data_buf, lightValue);
				message_flag = 0;
    		}

    		//If SW3 is pressed, system will go to STATIONARY_MODE
    		if (sw3_flag == 1){
				countdown = 15;
				toggle_mode(STATIONARY_MODE);
    		}
    	}

        Timer0_Wait(1);
    }

}

void check_failed(uint8_t *file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while(1);
}

