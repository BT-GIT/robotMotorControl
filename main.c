// HARDWARE TARGET:
//    Target Platform: EK-TM4C123GXL Evaluation Board
//    Target uC:       TM4C123GH6PM
//    System Clock:    40 MHz
//    PWM Divisor:     64
//    Motor Driver:    SN754410

// GPIO CONFIGURATION:
//    Red LED:                 PF1 (Internal to board.)
//    Green LED:               PF3 (Internal to board.)
//    Blue LED:                PF2 (Internal to board.)
//    SW1 Pushbutton:          PF4 (Internal to board.)
//    Stepper Motor A1 PWM:    PB5 Green wire
//    Stepper Motor A2 PWM:    PB6 Blue wire
//    Stepper Motor B1 PWM:    PE4 Orange wire
//    Stepper Motor B2 PWM:    PE5 White wire

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"

// Hardware bit-band addresses.
#define RED_LED     ( *( ( volatile uint32_t* )( 0x42000000 + ( 0x400253FC - 0x40000000 ) * 32 + 1 * 4 ) ) )
#define GREEN_LED   ( *( ( volatile uint32_t* )( 0x42000000 + ( 0x400253FC - 0x40000000 ) * 32 + 3 * 4 ) ) )
#define BLUE_LED    ( *( ( volatile uint32_t* )( 0x42000000 + ( 0x400253FC - 0x40000000 ) * 32 + 2 * 4 ) ) )
#define PUSH_BUTTON ( *( ( volatile uint32_t *)( 0x42000000 + ( 0x400253FC - 0x40000000 ) * 32 + 4 * 4 ) ) )

// Hardware PWM comparator addresses.
#define MOTOR_PIN_1 PWM0_1_CMPB_R // Green wire  - PB5
#define MOTOR_PIN_2 PWM0_0_CMPA_R // Blue wire   - PB6 (PD0)
#define MOTOR_PIN_3 PWM0_2_CMPA_R // Orange wire - PE4
#define MOTOR_PIN_4 PWM0_2_CMPB_R // White wire  - PE5

// Discretes.
#define MAXSPEED 900    // Duty cycle numerator out of 1023.
#define ON       1
#define OFF      0

// ----------------------------------------------------------------------------

void init_hw() {
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | ( 4 << SYSCTL_RCC_SYSDIV_S ) |
            SYSCTL_RCC_USEPWMDIV | SYSCTL_RCC_PWMDIV_64; // PWM = sysclock / 2
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOB | SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOF;

    // Configure LED pins.
    GPIO_PORTF_DIR_R  |= 0x04;  // make bit 4 an output
    GPIO_PORTF_DR2R_R |= 0x04; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R  |= 0x04;  // enable LED
    GPIO_PORTF_DEN_R  |= 0x10;  // enable push button
    GPIO_PORTF_PUR_R  |= 0x10;  // enable internal pull-up for push button

    // Configure the four PWM drive lines.
    GPIO_PORTB_DIR_R   |= 0x60;   // Make bits 5 and 6 outputs
    GPIO_PORTB_DR2R_R  |= 0x60;  // Set drive strength to 2mA
    GPIO_PORTB_DEN_R   |= 0x60;   // Enable bits 5 and 6 for digital
    GPIO_PORTB_AFSEL_R |= 0x60; // Select auxilary function (PWM) for bits 5 and 6
    
    // Enable PWM on bits 5 and 6.
    GPIO_PORTB_PCTL_R = GPIO_PCTL_PB5_M0PWM3 | GPIO_PCTL_PB6_M0PWM0;

    GPIO_PORTE_DIR_R   |= 0x30;   // Make bits 4 and 5 outputs
    GPIO_PORTE_DR2R_R  |= 0x30;  // Set drive strength to 2mA
    GPIO_PORTE_DEN_R   |= 0x30;   // Enable bits 4 and 5 for digital
    GPIO_PORTE_AFSEL_R |= 0x30; // Select auxilary function for bits 4 and 5
    // Enable PWM on bits 4 and 5.
    GPIO_PORTE_PCTL_R = GPIO_PCTL_PE4_M0PWM4 | GPIO_PCTL_PE5_M0PWM5;

    // Configure the PWM generators to drive the motor lines.
    SYSCTL_RCGC0_R |= SYSCTL_RCGC0_PWM0; // Turn-on PWM0 module.
    __asm(" NOP"); // Wait 3 clocks+ for safe hardware config (see data sheet).
    __asm(" NOP");
    __asm(" NOP");
    wait_microsecond( 100 );

    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R0;   // Reset PWM modules.
    SYSCTL_SRPWM_R = 0;                 // Leave reset state.
    PWM0_0_CTL_R = 0;                   // Turn off PWM0 generators.
    PWM0_1_CTL_R = 0;
    PWM0_2_CTL_R = 0;
    PWM0_0_GENA_R = PWM_0_GENA_ACTCMPAD_ZERO | PWM_0_GENA_ACTLOAD_ONE; // output 0 on PWM0, gen 1a, cmpa
    PWM0_1_GENB_R = PWM_0_GENB_ACTCMPBD_ZERO | PWM_0_GENB_ACTLOAD_ONE; // output 3 on PWM0, gen 1b, cmpb
    PWM0_2_GENA_R = PWM_0_GENA_ACTCMPAD_ZERO | PWM_0_GENA_ACTLOAD_ONE; // output 4 on PWM0, gen 2a, cmpa
    PWM0_2_GENB_R = PWM_0_GENB_ACTCMPBD_ZERO | PWM_0_GENB_ACTLOAD_ONE;// output 5 on PWM0, gen 2b, cmpb
    PWM0_0_LOAD_R = 1024; // Set period to 40 MHz sys clock / 64 = 625000; 625000 / 1024 = 610.351 kHz.
    PWM0_1_LOAD_R = 1024;
    PWM0_2_LOAD_R = 1024;

    // Invert outputs for duty cycle increases with increasing compare values.
    PWM0_INVERT_R = PWM_INVERT_PWM3INV | PWM_INVERT_PWM4INV | PWM_INVERT_PWM5INV | PWM_INVERT_PWM0INV;
    MOTOR_PIN_1 = 0; // Turn off all generators by default (0=always low, 1023=always high).
    MOTOR_PIN_2 = 0;
    MOTOR_PIN_3 = 0;
    MOTOR_PIN_4 = 0;

    PWM0_0_CTL_R = PWM_0_CTL_ENABLE; // Turn on the PWM generators now configured.
    PWM0_1_CTL_R = PWM_0_CTL_ENABLE;
    PWM0_2_CTL_R = PWM_0_CTL_ENABLE;
    PWM0_ENABLE_R = PWM_ENABLE_PWM3EN | PWM_ENABLE_PWM4EN | PWM_ENABLE_PWM5EN | PWM_ENABLE_PWM0EN;
}

// Alert of the boot sequence.
inline void boot_driver( void ) {
    GREEN_LED = ON;
    wait_microsecond( 500000 );
    GREEN_LED = OFF;
    wait_microsecond( 500000 );
    GREEN_LED = ON;
    wait_microsecond( 500000 );
    GREEN_LED = OFF;
    wait_microsecond( 500000 );
    GREEN_LED = ON;
    wait_microsecond( 500000 );
    GREEN_LED = OFF;
}

// Alert of button pushed.
inline void reset_button( void ) {
    BLUE_LED = ON;
    wait_microsecond( 500000 );
    BLUE_LED = OFF;
    wait_microsecond( 500000 );
    BLUE_LED = ON;
    wait_microsecond( 500000 );
    BLUE_LED = OFF;
    wait_microsecond( 500000 );
    BLUE_LED = ON;
    wait_microsecond( 500000 );
    BLUE_LED = OFF;
}

// Blocking function for SW1.
void wait_pressed( void ) {
    static uint8_t debounce = 0U;
    while( debounce < 10 ) {
        if( !PUSH_BUTTON ) {
            ++debounce;
        }
        else {
            debounce = 0;
        }
        wait_microsecond( 5 );
    }
    debounce = 0;
}

void wait_microsecond( uint32_t us ) {
    __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*2 (speculative, so P=1)
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             NOP");                  // 1
    __asm("             B    WMS_LOOP0");       // 1*2 (speculative, so P=1)
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}

//-----------------------------------------------------------------------------

// Left motor control functions.
void leftForward (void) {
    MOTOR_PIN_1 = OFF;
    MOTOR_PIN_2 = MAXSPEED;
}
void leftReverse (void) {
    MOTOR_PIN_1 = MAXSPEED;
    MOTOR_PIN_2 = OFF;
}
void leftStop (void) {
    MOTOR_PIN_1 = MOTOR_PIN_2 = OFF;
}

// Right motor control functions.
void rightForward (void) {
    MOTOR_PIN_3 = OFF;
    MOTOR_PIN_4 = MAXSPEED;
}
void rightReverse (void) {
    MOTOR_PIN_3 = MAXSPEED;
    MOTOR_PIN_4 = OFF;
}
void rightStop (void) {
    MOTOR_PIN_3 = MOTOR_PIN_4 = OFF;
}

//-----------------------------------------------------------------------------

int main(void) {
    // Initialize the hardware for this driver
    init_hw();
    boot_driver();
    // Default to motors off.
    leftStop();
    rightStop();

    // Enter main loop.
    while(1) {
        // TEST SEQUENCE #1
        wait_pressed(); // Blocking function for button.
        reset_button(); // 2.5s delay.
        // Reverse for 1 sec.
        leftReverse();
        rightReverse();
        wait_microsecond(1 * 1000000);
        // Sit for 1 sec.
        leftStop();
        rightStop();
        wait_microsecond(1* 1000000);
        // Forward indefinitely.
        leftForward();
        rightForward();
        RED_LED = ON;
        
        // TEST SEQUENCE #2
        wait_pressed(); // Blocking function for button.
        reset_button(); // 2.5s delay.
        // Stop both motors.
        leftStop();
        rightStop();
        RED_LED = OFF;
    }
    return 0;
}
