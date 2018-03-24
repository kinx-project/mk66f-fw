#include "usb_device_config.h"
#include "usb.h"
#include "usb_device.h"

#include "usb_device_class.h"
#include "usb_device_hid.h"

#include "usb_device_ch9.h"
#include "usb_device_descriptor.h"

#include "composite.h"

#include "hid_keyboard.h"

#include "fsl_gpio.h"
#include "pin_mux.h"

extern usb_device_composite_struct_t g_UsbDeviceComposite; // composite.c

#define KEY_NONE 0
#define KEY_PRG 0

// The following defines allow us to use the shorter identifiers from the
// usb_hid_keys.h file used in the kinX bare-metal firmware. Shorter
// identifiers are easier to deal with when changing the table, and
// compatibility helps share it across the two firmware versions.
#define KEY_EQUAL      KEY_EQUAL_PLUS
#define KEY_CAPSLOCK   KEY_CAPS_LOCK
#define KEY_1          KEY_1_EXCLAMATION_MARK
#define KEY_GRAVE      KEY_GRAVE_ACCENT_AND_TILDE
#define KEY_2          KEY_2_AT
#define KEY_3          KEY_3_NUMBER_SIGN
#define KEY_LEFT       KEY_LEFTARROW
#define KEY_LEFTCTRL   KEY_LEFTCONTROL
#define KEY_END        KEY_END1
#define KEY_4          KEY_4_DOLLAR
#define KEY_5          KEY_5_PERCENT
#define KEY_RIGHT      KEY_RIGHTARROW
#define KEY_6          KEY_6_CARET
#define KEY_UP         KEY_UPARROW
#define KEY_7          KEY_7_AMPERSAND
#define KEY_8          KEY_8_ASTERISK
#define KEY_COMMA      KEY_COMMA_AND_LESS
#define KEY_DOWN       KEY_DOWNARROW
#define KEY_RIGHTCTRL  KEY_RIGHTCONTROL
#define KEY_9          KEY_9_OPARENTHESIS
#define KEY_DOT        KEY_DOT_GREATER
#define KEY_LEFTBRACE  KEY_OBRACKET_AND_OBRACE
#define KEY_SPACE      KEY_SPACEBAR
#define KEY_0          KEY_0_CPARENTHESIS
#define KEY_SEMICOLON  KEY_SEMICOLON_COLON
#define KEY_SLASH      KEY_SLASH_QUESTION
#define KEY_RIGHTBRACE KEY_CBRACKET_AND_CBRACE
#define KEY_MINUS      KEY_MINUS_UNDERSCORE
#define KEY_BACKSLASH  KEY_BACKSLASH_VERTICAL_BAR
#define KEY_APOSTROPHE KEY_SINGLE_AND_DOUBLE_QUOTE
#define KEY_ESC        KEY_ESCAPE
#define KEY_SYSRQ      KEY_PRINTSCREEN
#define KEY_SCROLLLOCK KEY_SCROLL_LOCK
#define KEY_NUMLOCK    KEY_KEYPAD_NUM_LOCK_AND_CLEAR
#define KEY_LEFTMETA   KEY_LEFT_GUI

uint8_t keyvalhid[15][7] = {
{KEY_EQUAL, KEY_TAB,       KEY_CAPSLOCK,    KEY_LEFTSHIFT,  KEY_NONE,       KEY_NONE,         KEY_NONE},
{KEY_1,     KEY_Q,         KEY_A,           KEY_Z,          KEY_GRAVE,      KEY_NONE,         KEY_NONE},
{KEY_2,     KEY_W,         KEY_S,           KEY_X,          KEY_INSERT,     KEY_LEFTMETA/*KEY_HOME*/, KEY_BACKSPACE},
{KEY_3,     KEY_E,         KEY_D,           KEY_C,          KEY_LEFT,       KEY_LEFTALT,      KEY_LEFTCTRL},
{KEY_4,     KEY_R,         KEY_F,           KEY_V,          KEY_NONE,       KEY_END,          KEY_NONE},
{KEY_5,     KEY_T,         KEY_G,           KEY_B,          KEY_RIGHT,      KEY_NONE,         KEY_ESC/*KEY_DELETE*/},
{KEY_6,     KEY_Y,         KEY_H,           KEY_N,          KEY_UP,         KEY_NONE,         KEY_RIGHTALT},
{KEY_7,     KEY_U,         KEY_J,           KEY_M,          KEY_NONE,       KEY_ENTER,        KEY_NONE},
{KEY_8,     KEY_I,         KEY_K,           KEY_COMMA,      KEY_DOWN,       KEY_RIGHTCTRL,    KEY_PAGEUP},
{KEY_9,     KEY_O,         KEY_L,           KEY_DOT,        KEY_LEFTBRACE,  KEY_SPACE,        KEY_PAGEDOWN},
{KEY_0,     KEY_P,         KEY_SEMICOLON,   KEY_SLASH,      KEY_RIGHTBRACE, KEY_NONE,         KEY_NONE},
{KEY_MINUS, KEY_BACKSLASH, KEY_APOSTROPHE,  KEY_RIGHTSHIFT, KEY_NONE,       KEY_NONE,         KEY_NONE},
{KEY_F3,    KEY_F6,        KEY_DELETE/*KEY_ESC*/, KEY_F9,   KEY_F12,        KEY_PAUSE,        KEY_NONE},
{KEY_F4,    KEY_F7,        KEY_F1,          KEY_F10,        KEY_SYSRQ,      KEY_PRG,          KEY_NONE},
{KEY_F5,    KEY_F8,        KEY_F2,          KEY_F11,        KEY_SCROLLLOCK, KEY_NUMLOCK,      KEY_NONE},
};

typedef struct row {
	GPIO_Type *gpio;
	uint32_t pin;
} row_t;

const row_t rows[15] = {
    {BOARD_INITPINS_ROW_EQL_GPIO, BOARD_INITPINS_ROW_EQL_PIN},
    {BOARD_INITPINS_ROW_1_GPIO,   BOARD_INITPINS_ROW_1_PIN},
    {BOARD_INITPINS_ROW_2_GPIO,   BOARD_INITPINS_ROW_2_PIN},
    {BOARD_INITPINS_ROW_3_GPIO,   BOARD_INITPINS_ROW_3_PIN},
    {BOARD_INITPINS_ROW_4_GPIO,   BOARD_INITPINS_ROW_4_PIN},
    {BOARD_INITPINS_ROW_5_GPIO,   BOARD_INITPINS_ROW_5_PIN},
    {BOARD_INITPINS_ROW_6_GPIO,   BOARD_INITPINS_ROW_6_PIN},
    {BOARD_INITPINS_ROW_7_GPIO,   BOARD_INITPINS_ROW_7_PIN},
    {BOARD_INITPINS_ROW_8_GPIO,   BOARD_INITPINS_ROW_8_PIN},
    {BOARD_INITPINS_ROW_9_GPIO,   BOARD_INITPINS_ROW_9_PIN},
    {BOARD_INITPINS_ROW_0_GPIO,   BOARD_INITPINS_ROW_0_PIN},
    {BOARD_INITPINS_ROW_MIN_GPIO, BOARD_INITPINS_ROW_MIN_PIN},
    {BOARD_INITPINS_ROW_ESC_GPIO, BOARD_INITPINS_ROW_ESC_PIN},
    {BOARD_INITPINS_ROW_F1_GPIO,  BOARD_INITPINS_ROW_F1_PIN},
    {BOARD_INITPINS_ROW_F2_GPIO,  BOARD_INITPINS_ROW_F2_PIN},
};

volatile uint32_t systick_millis_count = 0;
volatile uint32_t systick_micros_count = 0;
void SysTick_Handler(void) {
	systick_micros_count++;
	if (systick_micros_count % 1000 == 0) {
		systick_millis_count++;
	}
}

static inline uint32_t millis(void) __attribute__((always_inline, unused));
static inline uint32_t millis(void) {
    // Reading a volatile variable to another volatile
    // seems redundant, but isn't for some cases.
    // Eventually this should probably be replaced by a
    // proper memory barrier or other technique.  Please
    // do not remove this "redundant" code without
    // carefully verifying the case mentioned here:
    //
    // https://forum.pjrc.com/threads/17469-millis%28%29-on-teensy-3?p=104924&viewfull=1#post104924
    //
    volatile uint32_t ret = systick_millis_count; // single aligned 32 bit is atomic
    return ret;
}

// delay_inline sleeps for |cycles| (e.g. sleeping for F_CPU will sleep 1s).
// delay_inline assumes the cycle counter has already been
// initialized and should not be modified, i.e. it is safe to call from scan.
static void delay_inline(const uint32_t cycles) {
	const uint32_t start = DWT->CYCCNT;
	while ((DWT->CYCCNT - start) < cycles) {
		// busy-loop until time has passed
	}
}

enum {
	STATE_NOT_PRESSED = 0,
	STATE_PRESSED_BOUNCING = 1, // freeze for 5ms, consider pressed
	STATE_PRESSED = 2,
	STATE_RELEASING_BOUNCING = 3, // freeze for 5ms
};

typedef struct {
	uint8_t state;
	// Store the last instant so that we can compare durations, see
	// https://arduino.stackexchange.com/a/12588
	uint32_t last_millis;
} debounce_status_t;

static debounce_status_t debounce[15][7];

// The Cherry MX datasheet specifies 5ms, but that seems overly optimistic, at
// least for key switches which have seen a few years of usage.
static const uint32_t keyswitch_debounce_time = 10 /* ms */;

static bool debounce_tick(const int row, const int col, const bool pressed) {
	debounce_status_t *status = &(debounce[row][col]);
	switch (status->state) {
		case STATE_NOT_PRESSED:
		if (!pressed) {
			return false;
		}
		// state transition from not pressed to pressed
		status->state = STATE_PRESSED_BOUNCING;
		status->last_millis = millis();
		return true;

		case STATE_PRESSED_BOUNCING:
		if ((millis() - status->last_millis) < keyswitch_debounce_time) {
			return true; // key bouncing, ignore
		}
		status->state = STATE_PRESSED;
		return true;

		case STATE_PRESSED:
		if (pressed) {
			return true;
		}
		// state transition from pressed to not pressed
		status->state = STATE_RELEASING_BOUNCING;
		status->last_millis = millis();
		return false;

		case STATE_RELEASING_BOUNCING:
		if ((millis() - status->last_millis) < keyswitch_debounce_time) {
			return false; // key bouncing, ignore
		}
		status->state = STATE_NOT_PRESSED;
		return false;
	}

	return false; // unreached
}

// kbd_report_t represents a HID keyboard report
typedef struct __attribute__((packed)) {
	uint8_t modifier;
	uint8_t reserved;
	uint8_t keys[6];
} kbd_report_t;

USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
static kbd_report_t last_report;

#ifdef MEASURE_SCAN_LATENCY
static uint32_t longest_scan_cycles = 0;
#endif

void kinx_scan(void) {
	kbd_report_t report = {
			.modifier = KEY_NONE,
			.reserved = '\0',
			.keys = {KEY_NONE, KEY_NONE, KEY_NONE, KEY_NONE, KEY_NONE, KEY_NONE},
	};


#ifdef MEASURE_SCAN_LATENCY
	// Measure cycles for scan()
	__asm volatile ("cpsid i"); // disable interrupts
#endif
	DWT->CYCCNT = 0;

	int row, col, keynum = 0;
	for (row = 0; row < 15; row++) {
		rows[row].gpio->PDOR &= ~(1 << rows[row].pin); // set low

		// wait to ensure the signal propagated
		delay_inline(40); // measured max: 21 cycles

		// All 7 columns are connected to port D so that we can easily read all
		// values into one byte. Our matrix is “active low”, so invert the reading
		// to translate to a logical high (easier to work with in C).
		const uint8_t colval = (~GPIOD->PDIR & 0x7F /* 01111111b */);

		// Immediately set the row pin to high again to give it some time while we
		// process the reading:
		rows[row].gpio->PDOR |= (1 << rows[row].pin); // set high

		for (col = 0; col < 7; col++) {
			if (!debounce_tick(row, col, colval & (1 << col))) {
				continue; // not pressed
			}

			switch (keyvalhid[row][col]) {
			   case KEY_LEFTCONTROL:
				 report.modifier |= MODIFERKEYS_LEFT_CTRL;
				 break;

			   case KEY_LEFTSHIFT:
				 report.modifier |= MODIFERKEYS_LEFT_SHIFT;
				 break;

			   case KEY_LEFTALT:
				 report.modifier |= MODIFERKEYS_LEFT_ALT;
				 break;

			   case KEY_LEFT_GUI:
				 report.modifier |= MODIFERKEYS_LEFT_GUI;
				 break;

			   case KEY_RIGHTCONTROL:
				 report.modifier |= MODIFERKEYS_RIGHT_CTRL;
				 break;

			   case KEY_RIGHTSHIFT:
				 report.modifier |= MODIFERKEYS_RIGHT_SHIFT;
				 break;

			   case KEY_RIGHTALT:
				 report.modifier |= MODIFERKEYS_RIGHT_ALT;
				 break;

			   case KEY_RIGHT_GUI:
				 report.modifier |= MODIFERKEYS_RIGHT_GUI;
				 break;

			   case KEY_NONE:
				 break;

			   default:
				 report.keys[keynum++] = keyvalhid[row][col];
				 break;
			}
		}

		// Wait for the scan row to quiesce, i.e. wait for the column register to
		// read no activated rows again. We cannot poll the column register to
		// detect this state, because bouncing keys result in a seemingly-quiesced
		// column register, only to trigger ghost keys in the next scan.
		delay_inline(40); // measured max: 29 cycles
	}

	#ifdef MEASURE_SCAN_LATENCY
	const uint32_t scan_cycles = DWT->CYCCNT;
        __asm volatile ("cpsie i"); // re-enable interrupts

	if (scan_cycles > longest_scan_cycles) {
	  longest_scan_cycles = scan_cycles;
	  // longest scan: 20158 cycles = 111 μs
	  DbgConsole_Printf("longest scan: %d cycles\r\n", longest_scan_cycles);
	}
	#endif

	if (report.modifier == last_report.modifier &&
			report.keys[0] == last_report.keys[0] &&
			report.keys[1] == last_report.keys[1] &&
			report.keys[2] == last_report.keys[2] &&
			report.keys[3] == last_report.keys[3] &&
			report.keys[4] == last_report.keys[4] &&
			report.keys[5] == last_report.keys[5]) {
		return;
	}

	// TODO: before touching last_report, wait until USB endpoint 1 is no longer
	// transmitting, i.e. check the OWN flag in the current BDT.

	last_report.modifier = report.modifier;
	last_report.keys[0] = report.keys[0];
	last_report.keys[1] = report.keys[1];
	last_report.keys[2] = report.keys[2];
	last_report.keys[3] = report.keys[3];
	last_report.keys[4] = report.keys[4];
	last_report.keys[5] = report.keys[5];

	// Retry for up to 125μs, which is the duration of one USB poll (in High Speed mode).
	for (int retry = 0; retry < 125; retry++) {
	  const usb_status_t status = USB_DeviceHidSend(
		  g_UsbDeviceComposite.hidKeyboardHandle, USB_HID_KEYBOARD_ENDPOINT_IN,
		  (uint8_t*)&last_report, USB_HID_KEYBOARD_REPORT_LENGTH);
	  if (status == kStatus_USB_Success) {
		  break;
	  }
	  DbgConsole_Printf("[%d] retrying USB (status %d)\r\n", millis(), status);
	  delay_inline(180); // about 1μs
	}
}

void kinx_set_leds(const uint8_t led_report) {
	GPIO_PinWrite(BOARD_INITPINS_LED_CAPS_LOCK_GPIO, BOARD_INITPINS_LED_CAPS_LOCK_PIN, !(led_report & (1 << 1)));
	GPIO_PinWrite(BOARD_INITPINS_LED_NUM_LOCK_GPIO, BOARD_INITPINS_LED_NUM_LOCK_PIN, !(led_report & (1 << 5)));
	GPIO_PinWrite(BOARD_INITPINS_LED_SCROLL_LOCK_GPIO, BOARD_INITPINS_LED_SCROLL_LOCK_PIN, !(led_report & (1 << 2)));
}

void kinx_init(void) {
    // Configure the SysTick interrupt for millis() for debouncing:
    SysTick_Config(SystemCoreClock / 1000 / 1000);

    // Configure LEDs (all on during initialization):
    GPIO_PinWrite(BOARD_INITPINS_LED_CAPS_LOCK_GPIO, BOARD_INITPINS_LED_CAPS_LOCK_PIN, 0);
    GPIO_PinWrite(BOARD_INITPINS_LED_NUM_LOCK_GPIO, BOARD_INITPINS_LED_NUM_LOCK_PIN, 0);
    GPIO_PinWrite(BOARD_INITPINS_LED_SCROLL_LOCK_GPIO, BOARD_INITPINS_LED_SCROLL_LOCK_PIN, 0);
    GPIO_PinWrite(BOARD_INITPINS_LED_NUMPAD_GPIO, BOARD_INITPINS_LED_NUMPAD_PIN, 0);

    BOARD_INITPINS_LED_CAPS_LOCK_GPIO->PDDR |= (1U << BOARD_INITPINS_LED_CAPS_LOCK_PIN);
    BOARD_INITPINS_LED_NUM_LOCK_GPIO->PDDR |= (1U << BOARD_INITPINS_LED_CAPS_LOCK_PIN);
    BOARD_INITPINS_LED_SCROLL_LOCK_GPIO->PDDR |= (1U << BOARD_INITPINS_LED_SCROLL_LOCK_PIN);
    BOARD_INITPINS_LED_NUMPAD_GPIO->PDDR |= (1U << BOARD_INITPINS_LED_NUMPAD_PIN);

    // Configure the cycle counter for delay_inline() in matrix scanning:
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= (DWT_CTRL_CYCCNTENA_Msk << DWT_CTRL_CYCCNTENA_Pos);
    DWT->CYCCNT = 0;

    // Configure the matrix pins:
    // set COL* to input:
    GPIOD->PDDR &= ~((1 << 0) |
    		(1 << 1) |
			(1 << 2) |
			(1 << 3) |
			(1 << 4) |
			(1 << 5) |
			(1 << 6));
    for (int row = 0; row < 15; row++) {
    	rows[row].gpio->PDDR |= (1 << rows[row].pin); // set as output
    	rows[row].gpio->PDOR |= (1 << rows[row].pin); // set high
    }
}
