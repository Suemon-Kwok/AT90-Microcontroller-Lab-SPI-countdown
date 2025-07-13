#     AT90 Microcontroller Lab SPI countdown

Lab Week 11 Countdown (SPI, RTC)

Exercise

Design and implement a countdown timer using SPI communication with the DS1305 real-time clock.

The program must meet the following requirements:

Button Functions:

1. PA0: Increment by 1 second.

2. PA1: Start the countdown.

3. PA2: Clear/Reset the timer.

Display & Behaviour

1. Display: PORTC shows the current countdown value. The countdown ranges from 0-99 decimal.

It is ok to let the LED display on PORTC show it in hexadecimal.

2. Setup: Press PA0 to increment the timer. After setting a valid time (0–99 seconds), press PA1 to

begin the countdown.

3. Completion: When the timer reaches zero, the buzzer and PORTC7:0 are turned on and off at a

1 second rate (i.e. 0.5 seconds on, 0.5 seconds off).

4. Reset: Press PA2 to reset the display to 00 and silence the buzzer. The program then repeats.



/*

*	GccApplication1.c
*	
 *

*	Created: 28/05/2025 4:56:54 pm
*	
*	Author : 
*	
 */ 

#define F_CPU 8000000UL  // 8MHz clock speed

#include <avr/io.h>

#include <util/delay.h>

#include <avr/interrupt.h>

#include <stdbool.h>

// Suppress delay optimization warning

#pragma GCC diagnostic ignored "-Wcpp"

// DS1305 Register Addresses

/*

These are constants that define the memory addresses used by the DS1305 RTC for timekeeping. Write and read commands are also specified.

*/

#define DS1305_SECONDS_REG    0x00

#define DS1305_MINUTES_REG    0x01

#define DS1305_HOURS_REG      0x02

#define DS1305_CONTROL_REG    0x0F

#define DS1305_STATUS_REG     0x10

// DS1305 Commands

#define DS1305_READ_CMD       0x00

#define DS1305_WRITE_CMD      0x80

// Button definitions (using example's method)

/*These macros check if a button is pressed (PA0, PA1, PA2) using bitwise operations.

*/

#define isButton0 (!(PINA & 0b00000001))  // PA0

#define isButton1 (!(PINA & 0b00000010))  // PA1

#define isButton2 (!(PINA & 0b00000100))  // PA2

// SPI control (using example's method)

/*

ceOn and ceOff control Chip Enable (CE) for SPI communication.

spiFinished checks if an SPI transmission is complete.

*/

#define ceOn  (PORTB |= (1 << 0))

#define ceOff (PORTB &= ~(1 << 0))

#define spiFinished (SPSR & (1 << 7))

// State definitions

/*

Defines three states for the countdown timer:

STATE_SETUP - Set countdown value.

STATE_COUNTDOWN - Begin countdown process.

STATE_ALARM - Buzzer and LED blinking when time reaches 0

*/

typedef enum {

STATE_SETUP,

STATE_COUNTDOWN,

STATE_ALARM

} timer_state_t;

// Global variables

/*

volatile ensures that variables persist between interrupts. current_state: Tracks whether the timer is in setup, countdown, or alarm mode. countdown_value: Holds the countdown duration. last_second: Stores the last recorded second (for timing accuracy).

alarm_toggle: Controls the alarm blinking state

*/

volatile timer_state_t current_state = STATE_SETUP; volatile uint8_t countdown_value = 0; volatile uint8_t last_second = 0; volatile bool alarm_toggle = false;

// Function prototypes

/*

Defines function prototypes for setup, button handling, SPI communication, RTC timekeeping, buzzer control, and time conversion.

*/

void setup(void);

void writeRTC(char address, char data); unsigned char readRTC(char address); void reset_ds1305_time(void);

uint8_t get_current_second(void);

void handle_buttons(void); void update_display(void); void buzzer_on(void); void buzzer_off(void);

uint8_t bcd_to_decimal(uint8_t bcd); uint8_t decimal_to_bcd(uint8_t decimal);

/*

Calls setup functions and starts a loop that continuously reads button states and updates countdown.

Manages state transitions:

Setup Mode - Allows user input.

Countdown Mode - Decreases timer value.

Alarm Mode - Flashes LEDs and buzzer.

*/ int main(void) {

// Initialize all systems setup();

// Reset to known state reset_ds1305_time(); update_display();

while (1) { handle_buttons();

switch (current_state) { case STATE_SETUP:

// In setup mode, just wait for button presses update_display(); break;

case STATE_COUNTDOWN:

{

// Visual indicator that we're in countdown mode

// Briefly flash the highest bit every few cycles to show we're counting static uint8_t flash_counter = 0; flash_counter++; if (flash_counter > 100) { flash_counter = 0;

uint8_t temp_display = PORTC;

PORTC |= 0x80;  // Set highest bit

_delay_ms(50);

PORTC = temp_display;  // Restore display

}

// Check if a second has passed uint8_t current_second = get_current_second();

if (current_second != last_second) { last_second = current_second;

if (countdown_value > 0) { countdown_value--; update_display();

if (countdown_value == 0) { current_state = STATE_ALARM; alarm_toggle = false;

}

}

}

break;

}

case STATE_ALARM:

{

// Handle alarm blinking (1 second rate) uint8_t alarm_second = get_current_second();

if (alarm_second != last_second) { last_second = alarm_second; alarm_toggle = !alarm_toggle;

if (alarm_toggle) {

buzzer_on();

PORTC = 0xFF;  // All LEDs on

} else { buzzer_off();

PORTC = 0x00;  // All LEDs off

}

}

break;

}

}

_delay_ms(10);  // Small delay to prevent excessive polling

}

return 0;

}

/*

Configures GPIO pins for buttons and display.

Initializes SPI for DS1305 communication.

Sets up buzzer control.

*/

void setup(void) {

// Setup LED strip (from example)

DDRC = 0b11111111;  // Configure all bits of port C as output

PORTC = 0b00000000; // Turn off all LEDs

// Setup board to use push buttons (from example)

DDRE = 0b00000011;  // Configure PE0-1 as outputs, others as inputs

PORTE = 0b00000001; // Set PE0 high

DDRA = 0b00000000;  // Configure all PORTA as inputs (for buttons)

// Note: Internal pull-ups are automatically enabled for buttons

// Setup SPI (from example, modified for DS1305)

SPCR = 0b01010111; // SPI enable, AT90 Master, correct polarity and clock rate

SPSR = 0x00;       // No double speed

DDRB = 0b00000111; // Configure SPI pins: MOSI, SCK, SS as outputs

// Configure buzzer pin (assuming PE2 like in example)

DDRE |= (1 << 2);  // PE2 as output for buzzer

PORTE &= ~(1 << 2); // Buzzer off initially

// Initialize DS1305

_delay_ms(100);  // Allow DS1305 to stabilize

// Enable RTC and configure (similar to example)

writeRTC(0x8f, 0b10000000); // Control register: disable alarms initially writeRTC(DS1305_STATUS_REG, 0x00); // Clear status register

_delay_ms(10);

}

// Transfer a single byte to the RTC (from example)

/*

Sends a command to RTC via SPI.

*/

void writeRTC(char address, char data) { ceOn;

SPDR = address;      // Initiate transfer of the address to the RTC while(!spiFinished); // Wait until it has arrived in the RTC SPDR = data;         // Initiate transfer of the data byte to the RTC while(!spiFinished); // Wait until it has arrived in the RTC ceOff;

}

// Read a single byte from the RTC (from example)

/*

Reads a value from RTC.

*/

unsigned char readRTC(char address) { char data, rubbish = 0x00; ceOn;

SPDR = address;      // Initiate transfer of the address to the RTC while(!spiFinished); // Wait until it has arrived in the RTC

// The RTC will now store the result in its shift register, // ready to be picked up by the master.

// Write any byte, to generate the clock cycles to transfer the result

// from the RTC to the microcontroller SPDR = rubbish;

while(!spiFinished); // Wait until the byte has arrived in the microcontroller

data = SPDR;         // Store the received byte in a variable ceOff;               // Done with the single byte transfer return data;

}

void reset_ds1305_time(void) {

// Enable RTC first (like in the example)

writeRTC(0x8f, 0b00000000); // Control register: EOSC (enable oscillator) _delay_ms(10);

// Reset time to 00:00:00 (using write command)

writeRTC(DS1305_WRITE_CMD | DS1305_SECONDS_REG, 0x00); writeRTC(DS1305_WRITE_CMD | DS1305_MINUTES_REG, 0x00); writeRTC(DS1305_WRITE_CMD | DS1305_HOURS_REG, 0x00);

_delay_ms(10); last_second = 0;

}

uint8_t get_current_second(void) { uint8_t seconds_bcd = readRTC(DS1305_READ_CMD | DS1305_SECONDS_REG);

return bcd_to_decimal(seconds_bcd);

}

/*

Manages button inputs for incrementing, starting, and resetting the timer.

*/

void handle_buttons(void) { static uint8_t button_debounce = 0;

// Simple debouncing counter if (button_debounce > 0) { button_debounce--; return;

}

if (isButton0) {  // PA0 - Increment if (current_state == STATE_SETUP) { countdown_value++; if (countdown_value > 99) { countdown_value = 0;

}

update_display();

// Visual feedback

PORTC = 0b00000001; _delay_ms(100); update_display();

}

button_debounce = 50; // Debounce delay

}

if (isButton1) {  // PA1 - Start if (current_state == STATE_SETUP && countdown_value > 0) { current_state = STATE_COUNTDOWN; reset_ds1305_time(); last_second = 0;

// Visual feedback - flash all LEDs to show start PORTC = 0xFF;

_delay_ms(200);

PORTC = 0x00; _delay_ms(200); update_display();

} else if (current_state == STATE_SETUP && countdown_value == 0) {

// Flash error pattern if trying to start with 0 for(int i = 0; i < 3; i++) {

PORTC = 0b10101010;

_delay_ms(100);

PORTC = 0b01010101;

_delay_ms(100);

}

PORTC = 0x00;

}

button_debounce = 50; // Debounce delay

}

if (isButton2) {  // PA2 - Reset current_state = STATE_SETUP; countdown_value = 0; buzzer_off(); update_display(); reset_ds1305_time();

// Visual feedback

PORTC = 0b00000100; _delay_ms(100); update_display();

button_debounce = 50; // Debounce delay

}

}

void update_display(void) {

// For values 0-99, we'll show them in a readable format

// Since we only have 8 LEDs, we'll use a pattern to represent decimal values if (countdown_value <= 9) {

// For 0-9, show the binary representation

PORTC = countdown_value;

} else if (countdown_value <= 99) {

// For 10-99, show tens digit in upper 4 bits, ones digit in lower 4 bits uint8_t tens = countdown_value / 10; uint8_t ones = countdown_value % 10;

PORTC = (tens << 4) | ones;  // This creates BCD-like display

}

}

void buzzer_on(void) {

// Turn on buzzer (using PE2 like in example)

PORTE |= (1 << 2);

}

void buzzer_off(void) {

// Turn off buzzer

PORTE &= ~(1 << 2);

}

uint8_t bcd_to_decimal(uint8_t bcd) { return ((bcd >> 4) * 10) + (bcd & 0x0F);

}

uint8_t decimal_to_bcd(uint8_t decimal) { return ((decimal / 10) << 4) | (decimal % 10);

}




