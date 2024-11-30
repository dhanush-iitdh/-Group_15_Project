#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "adc.h"

// Define GPIO pin to control MOSFET gate (PB0)
#define MOSFET_GATE_PIN 0x01  // PB0 (bit 0)

float voltage; // Global declaration

// Function to initialize ADC on PE3
void ADC_Init_PE3(void) {
    SYSCTL_RCGCADC_R |= 0x01;  // Enable ADC0 clock
    SYSCTL_RCGCGPIO_R |= 0x10; // Enable clock for Port E
    while ((SYSCTL_PRGPIO_R & 0x10) == 0); // Wait for GPIOE to be ready

    GPIO_PORTE_AFSEL_R |= 0x08;    // Enable alternate function on PE3
    GPIO_PORTE_DEN_R &= ~0x08;     // Disable digital functionality on PE3
    GPIO_PORTE_AMSEL_R |= 0x08;    // Enable analog mode on PE3

    ADC0_ACTSS_R &= ~0x08;         // Disable SS3 during configuration
    ADC0_EMUX_R &= ~0xF000;        // Processor trigger for SS3
    ADC0_SSMUX3_R = 0;             // Set AIN0 (PE3) as input channel
    ADC0_SSCTL3_R = 0x06;          // Enable IE0 and END0
    ADC0_ACTSS_R |= 0x08;          // Re-enable SS3
}

// Function to read ADC value from PE3
uint16_t ADC_Read(void) {
    ADC0_PSSI_R = 0x08;                // Start ADC sampling on SS3
    while ((ADC0_RIS_R & 0x08) == 0);  // Wait for conversion to complete
    uint16_t result = ADC0_SSFIFO3_R & 0xFFF; // Read 12-bit ADC value
    ADC0_ISC_R = 0x08;                 // Clear completion flag
    return result;
}

// Function to convert ADC value to voltage
float ConvertToVoltage(uint16_t adcValue) {
    return adcValue * 3.3 / 4095;  // Scale ADC counts to voltage
}

void GPIO_PORT_C_init(void) {
    SYSCTL_RCGCGPIO_R |= 0x04;                // Enable clock for GPIOC
    SYSCTL_RCGCUART_R |= 0x08;                // Enable clock for UART3

    GPIO_PORTC_DEN_R |= 0xC0;                 // Digital enable for PC6 and PC7
    GPIO_PORTC_AFSEL_R |= 0xC0;               // Enable alternate function on PC6, PC7
    GPIO_PORTC_PCTL_R = (GPIO_PORTC_PCTL_R & 0x00FFFFFF) | 0x11000000; // Set PC6, PC7 for UART functionality

    UART3_CTL_R &= ~0x01;                     // Disable UART3 during setup
    UART3_IBRD_R = 104;                       // Set integer part of baud rate (9600 baud)
    UART3_FBRD_R = 11;                        // Set fractional part of baud rate
    UART3_LCRH_R = 0x62; // 8-bit data, odd parity, 1 stop bit

    UART3_CC_R = 0x00;                        // Use system clock

    UART3_CTL_R |= 0x301;                     // Enable UART3, RX and TX
}

void UART3_WRITE(char data) {
    while (UART3_FR_R & 0x20);  // Wait until TX FIFO is not full
    UART3_DR_R = data;          // Write data to UART data register
}

void UART3_SEND_STRING(const char* str) {
    while (*str) {
        UART3_WRITE(*str++);  // Send each character of the string
    }
}

void GPIO_Init(void) {
    SYSCTL_RCGCGPIO_R |= 0x02;  // Enable clock for Port B
    while ((SYSCTL_PRGPIO_R & 0x02) == 0); // Wait for GPIOB to be ready

    GPIO_PORTB_DIR_R |= MOSFET_GATE_PIN;  // Set PB0 as output
    GPIO_PORTB_DEN_R |= MOSFET_GATE_PIN;  // Enable digital functionality on PB0
}

void Control_MOSFET(float voltage) {
    float turnOnVoltage = 2.730f;  // Turn MOSFET on below this voltage
    float turnOffVoltage = 2.750f; // Turn MOSFET off above this voltage

    if (voltage < turnOnVoltage) {
        GPIO_PORTB_DATA_R &= ~MOSFET_GATE_PIN;  // Pull PB0 low (MOSFET on)
    } else if (voltage > turnOffVoltage) {
        GPIO_PORTB_DATA_R |= MOSFET_GATE_PIN;   // Pull PB0 high (MOSFET off)
    }
}

int main(void) {
    ADC_Init_PE3();        // Initialize ADC
    GPIO_Init();           // Initialize GPIO for MOSFET control
    GPIO_PORT_C_init();    // Initialize UART3 (PC6, PC7)

    char buffer[50];

    while (1) {

        UART3_SEND_STRING("Embbedded");


        uint16_t adcValue = ADC_Read();         // Read ADC value
        voltage = ConvertToVoltage(adcValue);  // Convert ADC value to voltage


        snprintf(buffer, sizeof(buffer), "Voltage: %.3f V\n", voltage);

        UART3_SEND_STRING(buffer);             // Send the string via UART
        volatile int delay;
                              for ( delay = 0; delay < 100000; delay++); // Simple delay
        Control_MOSFET(voltage);               // Control MOSFET based on voltage
    }
}
