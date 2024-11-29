#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "adc.h"

// Define GPIO pin to control MOSFET gate (PB0)
#define MOSFET_GATE_PIN 0x01  // PB0 (bit 0)

// Function to initialize ADC on PE3
void ADC_Init_PE3(void) {
    // Enable clocks for ADC0 and GPIOE
    SYSCTL_RCGCADC_R |= 0x01;  // Enable ADC0 clock
    SYSCTL_RCGCGPIO_R |= 0x10; // Enable clock for Port E
    while ((SYSCTL_PRGPIO_R & 0x10) == 0); // Wait for GPIOE to be ready

    // Configure PE3 as an analog input
    GPIO_PORTE_AFSEL_R |= 0x08;    // Enable alternate function on PE3
    GPIO_PORTE_DEN_R &= ~0x08;     // Disable digital functionality on PE3
    GPIO_PORTE_AMSEL_R |= 0x08;    // Enable analog mode on PE3

    // Configure ADC0
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

// Function to initialize GPIO pin for controlling the P-channel MOSFET gate (PB0)
void GPIO_Init(void) {
    SYSCTL_RCGCGPIO_R |= 0x02;  // Enable clock for Port B
    while ((SYSCTL_PRGPIO_R & 0x02) == 0); // Wait for GPIOB to be ready

    GPIO_PORTB_DIR_R |= MOSFET_GATE_PIN;  // Set PB0 as output
    GPIO_PORTB_DEN_R |= MOSFET_GATE_PIN;  // Enable digital functionality on PB0
}

// Function to control the P-channel MOSFET gate
void Control_MOSFET(float voltage) {
    // Define threshold values for turning on and off the MOSFET
    float turnOnVoltage = 2.730f;  // MOSFET turns on when voltage is less than 2.730V
    float turnOffVoltage = 2.750f; // MOSFET turns off when voltage is more than 2.750V

    // If voltage is less than 2.730V, turn MOSFET on (pull gate low)
    if (voltage < turnOnVoltage) {
        GPIO_PORTB_DATA_R &= ~MOSFET_GATE_PIN;  // Pull PB0 low (MOSFET on)
    }
    // If voltage is greater than 2.750V, turn MOSFET off (pull gate high)
    else if (voltage > turnOffVoltage) {
        GPIO_PORTB_DATA_R |= MOSFET_GATE_PIN;   // Pull PB0 high (MOSFET off)
    }
}

float voltage; // Global declaration

int main(void) {
    // Initialize peripherals
    ADC_Init_PE3();
    GPIO_Init();

    while (1) {
        // Read ADC value and convert to voltage
        uint16_t adcValue = ADC_Read();
        voltage =  ConvertToVoltage(adcValue)  ;

        // Control the MOSFET based on voltage
        Control_MOSFET(voltage);

    }
}
