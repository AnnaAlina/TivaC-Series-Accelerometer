******************************************************
Chip type : ARM TM4C123GH6PM
Program type : Firmware
Core Clock frequency    : 80.000000 MHz
*******************************************************/

#define ACCEL_W 0x3A            // Addresses for the accelerometer
#define ACCEL_R 0x3B
#define ACCEL_ADDR 0x1D

// Define needed for pin_map.h
#define PART_TM4C123GH6PM

#include <stdbool.h>
#include <stdint.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "inc/hw_i2c.h"
#include "driverlib/i2c.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"

void Accel_int();     // Function prototype to initialize the Accelerometer

signed int Accel_read();    // Function prototype to read the Accelerometer

void main(void) {

    signed short int LED_value = 1;

    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);                     //setup clock

    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);        // Enable I2C hardware
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);    // Enable Pin hardware

    GPIOPinConfigure(GPIO_PB3_I2C0SDA);        // Configure GPIO pin for I2C Data line
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);        // Configure GPIO Pin for I2C clock line

    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_3); // Set Pin Type

    // Enable Peripheral ports for output
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);                           //PORTC
    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_6|GPIO_PIN_7); // LED 1 LED 2

    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_5);         // LED 4

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);             //PORT D
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_6);         // LED 3

    //setup the I2C
GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);        // SDA MUST BE STD
GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_OD);        // SCL MUST BE OPEN DRAIN
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);                                     // The False sets the controller to 100kHz communication
    Accel_int();        // Function to initialize the Accelerometer

    while(1){
    // Fill in this section to read data from the Accelerometer and move the LEDs according to the X axis

        LED_value = LED_value + Accel_read();

        if(LED_value <= 1){
            // Cycle through the LEDs on the Orbit board
            GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6|GPIO_PIN_7, 0x40); 
// LED 1 on LED 2 Off
            GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0x00);  
// LED 3 off, Note different PORT
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, 0x00);      // LED 4 off
            LED_value = 1;    // reset value to maintain range
        }

        else if(LED_value == 2){
                            // Cycle through the LEDs on the Orbit board
            GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6|GPIO_PIN_7, 0x80);
                            // LED 1 off LED 2 on
            GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0x00);
                            // LED 3 off, Note different PORT
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, 0x00); // LED 4 on
        }
        else if(LED_value == 3){
                            // Cycle through the LEDs on the Orbit board
            GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6|GPIO_PIN_7, 0x00);
                                // LED 1 off LED 2 off
            GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0x40);
                                // LED 3 on, Note different PORT
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, 0x00); // LED 4 0ff
        }
        else if(LED_value >= 4){
                        // Cycle through the LEDs on the Orbit board
            GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6|GPIO_PIN_7, 0x00);
                                        // LED 1 off LED 2 Off
            GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0x00);
                                // LED 3 off, Note different PORT
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, 0x20); // LED 4 on
            LED_value = 4;    // reset value to maintain range
        }

    }
}

void Accel_int(){        // Function to initialize the Accelerometer

    I2CMasterSlaveAddrSet(I2C0_BASE, ACCEL_ADDR, false); // false means transmit

    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);// Send Start condition

    I2CMasterDataPut(I2C0_BASE, 0x2D);         // Writing to the Accel control reg
    SysCtlDelay(20000);                // Delay for first transmission
    I2CMasterDataPut(I2C0_BASE, 0x08);        // Send Value to control Register

    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);// Send Stop condition

while(I2CMasterBusBusy(I2C0_BASE)){};// Wait for I2C controller to finish operations

}

signed int Accel_read() { // Function to read the Accelerometer

    signed int data;
    signed short value = 0;            // value of y

    unsigned char MSB;
    unsigned char LSB;

    I2CMasterSlaveAddrSet(I2C0_BASE, ACCEL_ADDR, false); // false means transmit

    I2CMasterDataPut(I2C0_BASE, 0x34);               //Y axis Data 0
    SysCtlDelay(20000);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);    //Request LSB of Y Axis
    SysCtlDelay(2000000);                // Delay for first transmission

    I2CMasterSlaveAddrSet(I2C0_BASE, ACCEL_ADDR, true); // false means transmit

    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE); //Request LSB of Y Axis
    SysCtlDelay(20000);

    LSB = I2CMasterDataGet(I2C0_BASE);
    SysCtlDelay(20000);

    I2CMasterSlaveAddrSet(I2C0_BASE, ACCEL_ADDR, false);     // false means transmit
    I2CMasterDataPut(I2C0_BASE, 0x35);                //Y axis Data 1

    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);    //Request LSB of Y Axis
    SysCtlDelay(2000000);                        // Delay for first transmission

    I2CMasterSlaveAddrSet(I2C0_BASE, ACCEL_ADDR, true);     // false means transmit

    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE); //Request MSB of Y Axis
    SysCtlDelay(20000);

    MSB = I2CMasterDataGet(I2C0_BASE);
    value = (MSB << 8 | LSB);
    if(value < -125 ){                // testing axis for value
        data = -1;
    }
    else if (value > 125){
        data = 1;
    }
    else{
        data = 0;
    }
    SysCtlDelay(20000);

    return data;                    // return value
}
