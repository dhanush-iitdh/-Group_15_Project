#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include <stdio.h>  // For snprintf


// ADC0 Sequence 3 Interrupt Handler
void ADC0Seq3_Handler(void) {
    // Clear the ADC interrupt flag
    ADC0_ISC_R = (1 << 3); // Clear interrupt flag for Sequence 3

    // Add your logic for handling the ADC interrupt
}

// UART0 Interrupt Handler
void UART0_Handler(void) {
    // Clear the UART interrupt flag
    UART0_ICR_R = (1 << 4); // Clear interrupt flag for UART0

    // Add your logic for handling the UART interrupt
}

// System Clock Frequency
#define SYSTEM_CLOCK 16000000

// LM35 Configuration
#define LM35_CHANNEL 0    // ADC channel 0 (PE3)

// PID Controller Parameters
float Kp = 2.0, Ki = 0.5, Kd = 1.0;
float prev_error = 0, integral = 0;
float setpoint = 35.0;    // Desired temperature (in Celsius)

// Function Prototypes
void UART0_Init(void);
void UART0_SendString(char *str);
void ADC0_Init(void);
uint16_t ADC0_Read(void);
void PWM_Init(void);
void PWM_SetDutyCycle(uint8_t dutyCycle);
void delay_ms(int ms);
float readTemperature(void);
float computePID(float current_temp);

int main(void) {
    float current_temp = 0, output = 0;
    char buffer[50];

    // Initialize peripherals
    UART0_Init();
    ADC0_Init();
    PWM_Init();

    UART0_SendString("PID Temperature Control Initialized\r\n");

    while (1) {
        // Read the current temperature from LM35
        current_temp = readTemperature();

        // Compute the PID output
        output = computePID(current_temp);

        // Set the PWM duty cycle
        PWM_SetDutyCycle((uint8_t)output);

        // Transmit data over UART
        snprintf(buffer, sizeof(buffer), "Temp: %.2f C, PWM: %.2f%%\r\n", current_temp, output);
        UART0_SendString(buffer);

        // Small delay for stability
        delay_ms(500);
    }
}

void UART0_Init(void) {
    SYSCTL_RCGCUART_R |= (1 << 0);  // Enable UART0
    SYSCTL_RCGCGPIO_R |= (1 << 0);  // Enable GPIOA

    // Configure UART0 Pins: PA0 (RX) and PA1 (TX)
    GPIO_PORTA_AFSEL_R |= 0x03;
    GPIO_PORTA_PCTL_R |= 0x11;
    GPIO_PORTA_DEN_R |= 0x03;

    UART0_CTL_R &= ~(1 << 0);       // Disable UART0
    UART0_IBRD_R = 104;             // Baud rate = 9600 (16 MHz clock)
    UART0_FBRD_R = 11;
    UART0_LCRH_R = (0x3 << 5);      // 8-bit, no parity, 1-stop bit
    UART0_CTL_R |= (1 << 0) | (1 << 8) | (1 << 9); // Enable UART0, TX, RX
}

void UART0_SendString(char *str) {
    while (*str) {
        while ((UART0_FR_R & (1 << 5)) != 0); // Wait until TXFF is empty
        UART0_DR_R = *str++;
    }
}

void ADC0_Init(void) {
    SYSCTL_RCGCADC_R |= 1;              // Enable ADC0
    SYSCTL_RCGCGPIO_R |= (1 << 4);      // Enable GPIOE
    GPIO_PORTE_AFSEL_R |= (1 << 3);     // Enable alternate function for PE3
    GPIO_PORTE_DEN_R &= ~(1 << 3);      // Disable digital function
    GPIO_PORTE_AMSEL_R |= (1 << 3);     // Enable analog function

    ADC0_ACTSS_R &= ~(1 << 3);          // Disable SS3
    ADC0_EMUX_R &= ~0xF000;             // Software trigger conversion
    ADC0_SSMUX3_R = LM35_CHANNEL;       // Set channel for SS3
    ADC0_SSCTL3_R = 0x06;               // Single-ended, end-of-sequence
    ADC0_ACTSS_R |= (1 << 3);           // Enable SS3
}

uint16_t ADC0_Read(void) {
    ADC0_PSSI_R = (1 << 3);             // Start conversion on SS3
    while ((ADC0_RIS_R & (1 << 3)) == 0); // Wait for conversion to complete
    uint16_t result = ADC0_SSFIFO3_R & 0xFFF; // Read 12-bit result
    ADC0_ISC_R = (1 << 3);              // Clear completion flag
    return result;
}

float readTemperature(void) {
    uint16_t adc_value = ADC0_Read();
    return (adc_value * 3.3 / 4096) * 100; // Convert ADC value to temperature
}

void PWM_Init(void) {
    SYSCTL_RCGCPWM_R |= 1;              // Enable PWM Module 0
    SYSCTL_RCGCGPIO_R |= (1 << 1);      // Enable GPIOB
    SYSCTL_RCC_R &= ~(1 << 20);         // Use system clock for PWM

    GPIO_PORTB_AFSEL_R |= (1 << 6);     // Enable alternate function for PB6
    GPIO_PORTB_PCTL_R |= (4 << 24);     // Configure PB6 for PWM
    GPIO_PORTB_DEN_R |= (1 << 6);       // Enable digital function for PB6

    PWM0_0_CTL_R = 0;                   // Disable PWM generator 0
    PWM0_0_GENA_R = 0xC8;               // Configure PWM output
    PWM0_0_LOAD_R = SYSTEM_CLOCK / 1000 - 1; // Set frequency (1 kHz)
    PWM0_0_CMPA_R = 0;                  // Set initial duty cycle
    PWM0_0_CTL_R |= 1;                  // Enable PWM generator 0
    PWM0_ENABLE_R |= (1 << 0);          // Enable PWM output
}

void PWM_SetDutyCycle(uint8_t dutyCycle) {
    if (dutyCycle > 100) dutyCycle = 100; // Limit to 100%
    PWM0_0_CMPA_R = ((100 - dutyCycle) * (SYSTEM_CLOCK / 1000)) / 100;
}

float computePID(float current_temp) {
    float error = setpoint - current_temp;
    integral += error;
    float derivative = error - prev_error;
    prev_error = error;

    float output = Kp * error + Ki * integral + Kd * derivative;

    if (output > 100) output = 100;  // Limit PWM to 100%
    if (output < 0) output = 0;      // Limit PWM to 0%

    return output;
}

void delay_ms(int ms) {
    int i; // Declare 'i' here
    for (i = 0; i < ms * 3180; i++) {
        __asm("NOP");
    }
}
