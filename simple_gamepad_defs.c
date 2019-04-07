/* Teensy Simple Gamepad
 * A Gamepad device with one 2-axis D-pad and 1 or more buttons
 * Copyright (C) 2013 Robert Byam, robertbyam.com
 *
 * Derived from keyboard and serial examples for Teensy USB Development Board
 * http://www.pjrc.com/teensy/usb_keyboard.html
 * http://www.pjrc.com/teensy/usb_serial.html
 * Copyright (c) 2008 PJRC.COM, LLC
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/* ===========================================================================
   This software implements a generic gamepad with one 2-axis (four direction)
   D-Pad and 1 or more button.  The intended usage is that all of these files
   remain constant, and only the file simple_gamepad_config.h is modified to
   specify the configurable parameters for the gamepad

   simple_gamepad_defs.c
   This file contains all the code that relies on the settings found in
   simple_gamepad_config.h
   ======================================================================== */


/* This file creates structures and sends USB messages based on the
   settings in simple_gamepad_def.h */

#include "simple_gamepad_defs.h"
#include "simple_gamepad_usb.h"
#include <string.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>



const uint8_t GAMEPAD_HID_REPORT_DESC_SIZE = sizeof(gamepad_hid_report_desc);
/* define the global gamepad state object instance */
gamepad_state g_gamepadState[CONTROLLER_COUNT];


/* These macros and definitions implement the button to port mappings */

#if BUTTON_COUNT == 0 || BUTTON_COUNT >= 21
#error BUTTON_COUNT must be 1 to 20
#endif

// Port array index definitions
#define INDEX_B     0
#define INDEX_C     1
#define INDEX_D     2
#define INDEX_E     3
#define INDEX_F     4

// Button Mappings
#define BUTTON_UP_INDEX     INDEX_C
#define BUTTON_UP_SHIFT     6 // C6
#define BUTTON_DOWN_INDEX   INDEX_D
#define BUTTON_DOWN_SHIFT   3 // D3
#define BUTTON_LEFT_INDEX   INDEX_D
#define BUTTON_LEFT_SHIFT   2 // D2
#define BUTTON_RIGHT_INDEX  INDEX_D
#define BUTTON_RIGHT_SHIFT  1 // D1
#define BUTTON_1_INDEX      INDEX_D
#define BUTTON_1_SHIFT      0 // D0
#define BUTTON_2_INDEX      INDEX_B
#define BUTTON_2_SHIFT      7 // B7
#define BUTTON_3_INDEX      INDEX_B
#define BUTTON_3_SHIFT      3 // B3
#define BUTTON_4_INDEX      INDEX_B
#define BUTTON_4_SHIFT      2 // B2
#define BUTTON_5_INDEX      INDEX_B
#define BUTTON_5_SHIFT      0 // B0
#define BUTTON_6_INDEX      INDEX_B
#define BUTTON_6_SHIFT      1 // B1

#define BUTTON_UP_P2_INDEX    INDEX_F
#define BUTTON_UP_P2_SHIFT    0 // F0
#define BUTTON_DOWN_P2_INDEX  INDEX_F
#define BUTTON_DOWN_P2_SHIFT  1 // F1
#define BUTTON_LEFT_P2_INDEX  INDEX_F
#define BUTTON_LEFT_P2_SHIFT  4 // F4
#define BUTTON_RIGHT_P2_INDEX INDEX_F
#define BUTTON_RIGHT_P2_SHIFT 5 // F5
#define BUTTON_1_P2_INDEX     INDEX_F
#define BUTTON_1_P2_SHIFT     6 // F6
#define BUTTON_2_P2_INDEX     INDEX_F
#define BUTTON_2_P2_SHIFT     7 // F7
#define BUTTON_3_P2_INDEX     INDEX_B
#define BUTTON_3_P2_SHIFT     6 // B6
#define BUTTON_4_P2_INDEX     INDEX_B
#define BUTTON_4_P2_SHIFT     5 // B5
#define BUTTON_5_P2_INDEX     INDEX_D
#define BUTTON_5_P2_SHIFT     7 // D7
#define BUTTON_6_P2_INDEX     INDEX_B
#define BUTTON_6_P2_SHIFT     4 // B4

// ease of use macros for passing shift/index to functions
static const uint8_t BUTTON_UP[CONTROLLER_COUNT][2] =
{
    { BUTTON_UP_INDEX, BUTTON_UP_SHIFT },
    { BUTTON_UP_P2_INDEX, BUTTON_UP_P2_SHIFT }
};

static const uint8_t BUTTON_DOWN[CONTROLLER_COUNT][2] =
{
    { BUTTON_DOWN_INDEX, BUTTON_DOWN_SHIFT },
    { BUTTON_DOWN_P2_INDEX, BUTTON_DOWN_P2_SHIFT }
};

static const uint8_t BUTTON_LEFT[CONTROLLER_COUNT][2] =
{
    { BUTTON_LEFT_INDEX, BUTTON_LEFT_SHIFT },
    { BUTTON_LEFT_P2_INDEX, BUTTON_LEFT_P2_SHIFT }
};

static const uint8_t BUTTON_RIGHT[CONTROLLER_COUNT][2] =
{
    { BUTTON_RIGHT_INDEX, BUTTON_RIGHT_SHIFT },
    { BUTTON_RIGHT_P2_INDEX, BUTTON_RIGHT_P2_SHIFT }
};

static const uint8_t BUTTON_BTN[CONTROLLER_COUNT][20][2] =
{
    {
        { BUTTON_1_INDEX, BUTTON_1_SHIFT },
        { BUTTON_2_INDEX, BUTTON_2_SHIFT },
        { BUTTON_3_INDEX, BUTTON_3_SHIFT },
        { BUTTON_4_INDEX, BUTTON_4_SHIFT },
        { BUTTON_5_INDEX, BUTTON_5_SHIFT },
        { BUTTON_6_INDEX, BUTTON_6_SHIFT }
    },
    {
        { BUTTON_1_P2_INDEX, BUTTON_1_P2_SHIFT },
        { BUTTON_2_P2_INDEX, BUTTON_2_P2_SHIFT },
        { BUTTON_3_P2_INDEX, BUTTON_3_P2_SHIFT },
        { BUTTON_4_P2_INDEX, BUTTON_4_P2_SHIFT },
        { BUTTON_5_P2_INDEX, BUTTON_5_P2_SHIFT },
        { BUTTON_6_P2_INDEX, BUTTON_6_P2_SHIFT },
    }
};

/* this function checks the inputs read via READ_ALL_INPUTS */
static inline uint8_t
INPUT_ACTIVE(uint8_t portArray[5], uint8_t index, uint8_t shift)
{
    // pull-up resistors make buttons active LOW
    return ((portArray[index] & (1 << shift)) == 0 ? 1 : 0);
}


static inline void
SET_AS_INPUT(uint8_t portArray[5], uint8_t index, uint8_t shift)
{
    // clear the bit to make it an input
    portArray[index] &= ~(1 << shift);
}


static inline void
READ_ALL_INPUTS(uint8_t portArray[5])
{
    portArray[INDEX_B] = PINB;
    portArray[INDEX_D] = PIND;
    portArray[INDEX_C] = PINC;
    portArray[INDEX_F] = PINF;
    portArray[INDEX_E] = PINE;
}


/* this function reads the gamepad state from the hardware */
uint8_t
simple_gampad_read_buttons(int8_t controller)
{
    uint8_t inPorts[5];
    uint8_t i;
    uint8_t btnArrayIndex;
    uint8_t btnArrayShift;
    gamepad_state prevState;

    // save previous state
    prevState = g_gamepadState[controller];

    // read all values from hardware into local array
    READ_ALL_INPUTS(inPorts);

    // set y axis
    if (INPUT_ACTIVE(inPorts, BUTTON_UP[controller][0], BUTTON_UP[controller][1]))
        g_gamepadState[controller].y_axis = Y_AXIS_UP;
    else if (INPUT_ACTIVE(inPorts, BUTTON_DOWN[controller][0], BUTTON_DOWN[controller][1]))
        g_gamepadState[controller].y_axis = Y_AXIS_DOWN;
    else
        g_gamepadState[controller].y_axis = AXIS_CENTER;

    // set x axis
    if (INPUT_ACTIVE(inPorts, BUTTON_LEFT[controller][0], BUTTON_LEFT[controller][1]))
        g_gamepadState[controller].x_axis = X_AXIS_LEFT;
    else if (INPUT_ACTIVE(inPorts, BUTTON_RIGHT[controller][0], BUTTON_RIGHT[controller][1]))
        g_gamepadState[controller].x_axis = X_AXIS_RIGHT;
    else
        g_gamepadState[controller].x_axis = AXIS_CENTER;

    // clear old button settings
    for (i = 0; i < BUTTON_ARRAY_SIZE; i++)
    {
        g_gamepadState[controller].buttons[i] = 0;
    }

    // set all the buttons - one bit for each button

    for (i = 0; i < BUTTON_COUNT; i++)
    {
        btnArrayIndex = i / 8;
        btnArrayShift = i % 8;
        g_gamepadState[controller].buttons[btnArrayIndex] |=
                INPUT_ACTIVE(inPorts, BUTTON_BTN[controller][i][0], BUTTON_BTN[controller][i][1]) << btnArrayShift;
    }

    return (memcmp(&g_gamepadState[controller], &prevState, sizeof(prevState)) == 0 ? 0 : 1);
}


/* this function transmits the state report */
int8_t
usb_simple_gamepad_send(int8_t controller)
{
    uint8_t intr_state, timeout, i;

    if (!usb_configuration)
        return -1;

    intr_state = SREG;
    cli();
    UENUM = GAMEPAD_ENDPOINT;
    timeout = UDFNUML + 50;
    for (;;)
    {
        // are we ready to transmit?
        if (UEINTX & (1<<RWAL))
            break;
        SREG = intr_state;
        // has the USB gone offline?
        if (!usb_configuration)
            return -1;
        // have we waited too long?
        if (UDFNUML == timeout)
            return -1;
        // get ready to try checking again
        intr_state = SREG;
        cli();
        UENUM = GAMEPAD_ENDPOINT;
    }

    // Report ID
    UEDATX = controller + 1; // +1 as Report ID start at 1, not 0
    // transmit axis
    UEDATX = (uint8_t)g_gamepadState[controller].x_axis;
    UEDATX = (uint8_t)g_gamepadState[controller].y_axis;
    // transmit each button
    for (i = 0; i < BUTTON_ARRAY_SIZE; i++)
    {
        UEDATX = g_gamepadState[controller].buttons[i];
    }

    UEINTX = 0x3A;
    SREG = intr_state;
    return 0;
}


/* this function configures the hardware for the desired usage */
void
simple_gamepad_configure(void)
{
    uint8_t i;
    // default all to outputs
    uint8_t ddrValues[5] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

    for (uint8_t controller = 0; controller < CONTROLLER_COUNT; controller++) {
        // d-pad buttons
        SET_AS_INPUT(ddrValues, BUTTON_UP[controller][0], BUTTON_UP[controller][1]);
        SET_AS_INPUT(ddrValues, BUTTON_DOWN[controller][0], BUTTON_DOWN[controller][1]);
        SET_AS_INPUT(ddrValues, BUTTON_LEFT[controller][0], BUTTON_LEFT[controller][1]);
        SET_AS_INPUT(ddrValues, BUTTON_RIGHT[controller][0], BUTTON_RIGHT[controller][1]);
        // set each button
        for (i = 0; i < BUTTON_COUNT; i++)
        {
            SET_AS_INPUT(ddrValues, BUTTON_BTN[controller][i][0], BUTTON_BTN[controller][i][1]);
        }
    }

    // write to the DDR registers
    DDRB = ddrValues[INDEX_B];
    DDRC = ddrValues[INDEX_C];
    DDRD = ddrValues[INDEX_D];
    DDRE = ddrValues[INDEX_E];
    DDRF = ddrValues[INDEX_F];

#ifdef USE_INTERNAL_PULL_UPS
    // if using internal pull-up resistors, write a 1 to each input
    // and a 0 to all outputs to set them low
    PORTB = ~ddrValues[INDEX_B];
    PORTC = ~ddrValues[INDEX_C];
    PORTD = ~ddrValues[INDEX_D];
    PORTE = ~ddrValues[INDEX_E];
    PORTF = ~ddrValues[INDEX_F];
#else
    // set all ports to no internal pull-up and outputs to low
    PORTB = 0;
    PORTC = 0;
    PORTD = 0;
    PORTE = 0;
    PORTF = 0;
#endif
}





